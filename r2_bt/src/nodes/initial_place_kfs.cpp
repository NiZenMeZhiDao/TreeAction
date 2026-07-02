#include "r2_bt/nodes/actions/initial_place_kfs.hpp"

#include <cmath>

namespace r2_bt
{

InitialPlaceKFS::InitialPlaceKFS(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList InitialPlaceKFS::providedPorts()
{
  return {
    BT::InputPort<bool>("enabled", true, "If false, skip initial place"),
    BT::InputPort<std::string>("deck_topic", "/aruco_comm/tx_id", "Deck Int32 topic"),
    BT::InputPort<int>("default_command", 5, "Default layer-2 deck command"),
    BT::InputPort<std::string>("prepare_action_name", "/prepare_kfs", "PrepareKFS action name"),
    BT::InputPort<double>("prepare_timeout_sec", 30.0, "PrepareKFS timeout"),
    BT::InputPort<bool>("forward_enabled", false, "Run timed forward approach before place"),
    BT::InputPort<std::string>("forward_velocity_topic", "/t0x0111_pid", "Velocity topic"),
    BT::InputPort<double>("forward_speed_mps", 0.15, "Forward velocity command"),
    BT::InputPort<double>("forward_duration_sec", 0.5, "Forward duration"),
    BT::InputPort<double>("forward_publish_rate_hz", 50.0, "Forward publish rate"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus InitialPlaceKFS::onStart()
{
  if (!configure())
  {
    return BT::NodeStatus::FAILURE;
  }
  if (!enabled_)
  {
    setOutput("message", "InitialPlaceKFS disabled");
    return BT::NodeStatus::SUCCESS;
  }
  if (!send_prepare())
  {
    return BT::NodeStatus::FAILURE;
  }
  config().blackboard->set("active_action", std::string{"InitialPlaceKFS"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  return BT::NodeStatus::RUNNING;
}

bool InitialPlaceKFS::configure()
{
  if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
  {
    setOutput("message", "Missing ros_node on blackboard");
    return false;
  }

  enabled_ = getInput<bool>("enabled").value_or(true);
  deck_topic_ = getInput<std::string>("deck_topic").value_or("/aruco_comm/tx_id");
  default_command_ = getInput<int>("default_command").value_or(5);
  if (override_commands_.find(default_command_) == override_commands_.end())
  {
    default_command_ = 5;
  }
  pending_command_ = default_command_;
  prepare_action_name_ =
      getInput<std::string>("prepare_action_name").value_or("/prepare_kfs");
  prepare_timeout_sec_ = getInput<double>("prepare_timeout_sec").value_or(30.0);
  forward_enabled_ = getInput<bool>("forward_enabled").value_or(false);
  velocity_topic_ =
      getInput<std::string>("forward_velocity_topic").value_or("/t0x0111_pid");
  forward_speed_mps_ = getInput<double>("forward_speed_mps").value_or(0.15);
  forward_duration_sec_ = getInput<double>("forward_duration_sec").value_or(0.5);
  forward_publish_rate_hz_ = getInput<double>("forward_publish_rate_hz").value_or(50.0);

  (void)config().blackboard->get("final_place_action_service_name", tool_service_name_);
  (void)config().blackboard->get("final_place_mid_action", place_action_);
  (void)config().blackboard->get("final_place_action_timeout_sec", place_timeout_sec_);
  (void)config().blackboard->get("final_place_action_arg0", place_args_[0]);
  (void)config().blackboard->get("final_place_action_arg1", place_args_[1]);
  (void)config().blackboard->get("final_place_action_arg2", place_args_[2]);
  (void)config().blackboard->get("final_place_action_arg3", place_args_[3]);

  if (!deck_sub_)
  {
    deck_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        deck_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&InitialPlaceKFS::deck_callback, this, std::placeholders::_1));
  }
  if (!prepare_client_)
  {
    prepare_client_ = rclcpp_action::create_client<PrepareAction>(node_, prepare_action_name_);
  }
  if (!move_client_)
  {
    rclcpp_action::Client<MoveAction>::SharedPtr shared_client;
    if (config().blackboard->get("move_to_pose_client", shared_client) && shared_client)
    {
      move_client_ = shared_client;
    }
    else
    {
      move_client_ = rclcpp_action::create_client<MoveAction>(node_, "move_to_pose");
    }
  }
  if (!tool_client_ || tool_client_->get_service_name() != tool_service_name_)
  {
    rclcpp::Client<ToolService>::SharedPtr shared_client;
    std::string shared_name;
    if (config().blackboard->get("tool_action_client", shared_client) &&
        config().blackboard->get("tool_action_client_name", shared_name) &&
        shared_client && shared_name == tool_service_name_)
    {
      tool_client_ = shared_client;
    }
    else
    {
      tool_client_ = node_->create_client<ToolService>(tool_service_name_);
    }
  }
  if (!velocity_pub_ || velocity_pub_->get_topic_name() != velocity_topic_)
  {
    velocity_pub_ = node_->create_publisher<VelocityMsg>(velocity_topic_, 10);
  }

  return true;
}

bool InitialPlaceKFS::send_prepare()
{
  if (!prepare_client_->action_server_is_ready())
  {
    return fail("PrepareKFS action server not available: " + prepare_action_name_) ==
           BT::NodeStatus::SUCCESS;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    prepare_goal_handle_.reset();
    prepare_response_received_ = false;
    prepare_accepted_ = false;
    prepare_done_ = false;
    prepare_status_ = BT::NodeStatus::FAILURE;
    prepare_message_.clear();
  }

  auto goal = PrepareAction::Goal();
  goal.command = PrepareAction::Goal::CMD_PREPARE_FIRST;
  goal.deck_command = default_command_;
  goal.timeout_sec = static_cast<float>(prepare_timeout_sec_);

  auto options = rclcpp_action::Client<PrepareAction>::SendGoalOptions();
  options.goal_response_callback =
    [this](const std::shared_ptr<PrepareGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      prepare_response_received_ = true;
      prepare_goal_handle_ = goal_handle;
      prepare_accepted_ = static_cast<bool>(goal_handle);
      if (!goal_handle)
      {
        prepare_done_ = true;
        prepare_message_ = "PrepareKFS goal rejected";
      }
    };
  options.result_callback =
    [this](const PrepareGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(mutex_);
      prepare_done_ = true;
      if (result.result)
      {
        prepare_message_ = result.result->message;
      }
      prepare_status_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                         result.result && result.result->success)
                            ? BT::NodeStatus::SUCCESS
                            : BT::NodeStatus::FAILURE;
    };

  stage_ = Stage::WAIT_PREPARE_RESPONSE;
  stage_start_time_ = std::chrono::steady_clock::now();
  prepare_client_->async_send_goal(goal, options);
  RCLCPP_INFO(node_->get_logger(), "[InitialPlaceKFS] PrepareKFS sent");
  return true;
}

BT::NodeStatus InitialPlaceKFS::onRunning()
{
  if (stage_ == Stage::WAIT_PREPARE_RESPONSE)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!prepare_response_received_)
    {
      return BT::NodeStatus::RUNNING;
    }
    if (!prepare_accepted_)
    {
      return fail(prepare_message_.empty() ? "PrepareKFS goal rejected" : prepare_message_);
    }
    stage_ = Stage::WAIT_PREPARE_RESULT;
    return BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::WAIT_PREPARE_RESULT)
  {
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - stage_start_time_).count();
    if (prepare_timeout_sec_ > 0.0 && elapsed > prepare_timeout_sec_)
    {
      if (prepare_client_ && prepare_goal_handle_)
      {
        prepare_client_->async_cancel_goal(prepare_goal_handle_);
      }
      return fail("InitialPlaceKFS prepare timed out");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!prepare_done_)
      {
        return BT::NodeStatus::RUNNING;
      }
      if (prepare_status_ != BT::NodeStatus::SUCCESS)
      {
        return fail(prepare_message_.empty() ? "PrepareKFS failed" : prepare_message_);
      }
    }
    return send_motion(default_command_) ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }

  if (stage_ == Stage::WAIT_MOVE_RESPONSE)
  {
    (void)maybe_apply_override();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!move_response_received_)
    {
      return BT::NodeStatus::RUNNING;
    }
    if (!move_accepted_)
    {
      return fail(move_message_.empty() ? "Initial place motion rejected" : move_message_);
    }
    stage_ = Stage::WAIT_MOVE_RESULT;
    return BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::WAIT_MOVE_RESULT)
  {
    if (maybe_apply_override())
    {
      return BT::NodeStatus::RUNNING;
    }
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - stage_start_time_).count();
    if (active_target_.timeout_sec > 0.0 && elapsed > active_target_.timeout_sec)
    {
      if (move_client_ && move_goal_handle_)
      {
        move_client_->async_cancel_goal(move_goal_handle_);
      }
      return fail("Initial place motion timed out");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!move_done_)
      {
        return BT::NodeStatus::RUNNING;
      }
      if (move_status_ != BT::NodeStatus::SUCCESS)
      {
        return fail(move_message_.empty() ? "Initial place motion failed" : move_message_);
      }
    }
    start_forward_or_place();
    return BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::FORWARD_APPROACH)
  {
    return tick_forward();
  }

  if (stage_ == Stage::WAIT_PLACE)
  {
    if (!place_sent_)
    {
      return fail(error_msg_.empty() ? "Initial place ToolAction not sent" : error_msg_);
    }
    if (!place_future_.valid() ||
        place_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - stage_start_time_).count();
      if (place_timeout_sec_ > 0.0 && elapsed > place_timeout_sec_)
      {
        return fail("Initial place ToolAction timed out");
      }
      return BT::NodeStatus::RUNNING;
    }
    const auto response = place_future_.get();
    if (response && response->success && response->ret == 0)
    {
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      setOutput("message", response->message);
      return BT::NodeStatus::SUCCESS;
    }
    return fail(response ? response->message : "Initial place ToolAction failed");
  }

  return BT::NodeStatus::RUNNING;
}

bool InitialPlaceKFS::send_motion(int command)
{
  if (!move_client_->action_server_is_ready())
  {
    return fail("/move_to_pose action server not available") == BT::NodeStatus::SUCCESS;
  }

  Target target;
  std::string error;
  if (!read_target(command, target, error))
  {
    return fail(error) == BT::NodeStatus::SUCCESS;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_target_ = target;
    move_goal_handle_.reset();
    move_response_received_ = false;
    move_accepted_ = false;
    move_done_ = false;
    move_status_ = BT::NodeStatus::FAILURE;
    move_message_.clear();
    ++motion_generation_;
  }
  const int generation = motion_generation_;

  auto goal = MoveAction::Goal();
  goal.x = target.x;
  goal.y = target.y;
  goal.yaw_deg = target.yaw * 180.0 / M_PI;
  goal.pid_profile = static_cast<uint8_t>(target.pid_profile);
  goal.max_vel = target.max_vel;
  goal.max_wz = target.max_wz;

  auto options = rclcpp_action::Client<MoveAction>::SendGoalOptions();
  options.goal_response_callback =
    [this, generation](const std::shared_ptr<MoveGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != motion_generation_) return;
      move_response_received_ = true;
      move_goal_handle_ = goal_handle;
      move_accepted_ = static_cast<bool>(goal_handle);
      if (!goal_handle)
      {
        move_done_ = true;
        move_message_ = "Initial place motion rejected";
      }
    };
  options.result_callback =
    [this, generation](const MoveGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != motion_generation_) return;
      move_done_ = true;
      if (result.result)
      {
        move_message_ = result.result->message;
      }
      move_status_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                      result.result && result.result->success)
                         ? BT::NodeStatus::SUCCESS
                         : BT::NodeStatus::FAILURE;
    };

  stage_ = Stage::WAIT_MOVE_RESPONSE;
  stage_start_time_ = std::chrono::steady_clock::now();
  move_client_->async_send_goal(goal, options);
  RCLCPP_INFO(node_->get_logger(), "[InitialPlaceKFS] Motion to %s command=%d",
              target.key.c_str(), command);
  return true;
}

bool InitialPlaceKFS::maybe_apply_override()
{
  int pending = default_command_;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending = pending_command_;
  }
  if (pending == active_target_.command)
  {
    return false;
  }
  if (move_client_ && move_goal_handle_)
  {
    move_client_->async_cancel_goal(move_goal_handle_);
  }
  RCLCPP_INFO(node_->get_logger(), "[InitialPlaceKFS] Override target: %d -> %d",
              active_target_.command, pending);
  return send_motion(pending);
}

void InitialPlaceKFS::start_forward_or_place()
{
  if (!forward_enabled_ || forward_duration_sec_ <= 0.0 ||
      std::abs(forward_speed_mps_) <= 1e-6)
  {
    start_place();
    return;
  }
  stage_ = Stage::FORWARD_APPROACH;
  stage_start_time_ = std::chrono::steady_clock::now();
  forward_next_publish_ = stage_start_time_;
}

BT::NodeStatus InitialPlaceKFS::tick_forward()
{
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration<double>(now - stage_start_time_).count();
  if (elapsed >= forward_duration_sec_)
  {
    publish_velocity(0.0, 0.0, 0.0);
    start_place();
    return BT::NodeStatus::RUNNING;
  }
  if (now >= forward_next_publish_)
  {
    publish_velocity(forward_speed_mps_, 0.0, 0.0);
    const double hz = forward_publish_rate_hz_ > 0.0 ? forward_publish_rate_hz_ : 50.0;
    forward_next_publish_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(1.0 / hz));
  }
  return BT::NodeStatus::RUNNING;
}

void InitialPlaceKFS::start_place()
{
  if (!tool_client_->service_is_ready())
  {
    error_msg_ = "ToolAction service not available: " + tool_service_name_;
    place_sent_ = false;
    stage_ = Stage::WAIT_PLACE;
    return;
  }
  auto request = std::make_shared<ToolService::Request>();
  request->action = place_action_;
  request->args[0] = static_cast<float>(place_args_[0]);
  request->args[1] = static_cast<float>(place_args_[1]);
  request->args[2] = static_cast<float>(place_args_[2]);
  request->args[3] = static_cast<float>(place_args_[3]);
  stage_ = Stage::WAIT_PLACE;
  stage_start_time_ = std::chrono::steady_clock::now();
  place_future_ = tool_client_->async_send_request(request);
  place_sent_ = true;
}

void InitialPlaceKFS::publish_velocity(double vx, double vy, double wz)
{
  if (!velocity_pub_)
  {
    return;
  }
  VelocityMsg msg;
  msg.data = {static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(wz)};
  velocity_pub_->publish(msg);
}

void InitialPlaceKFS::deck_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  const int value = msg->data;
  if (override_commands_.find(value) == override_commands_.end())
  {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pending_command_ = value;
}

bool InitialPlaceKFS::read_target(int command, Target& target, std::string& error) const
{
  target.command = command;
  target.key = target_key_for_command(command);
  const std::string prefix = "final_target_" + target.key + "_";
  if (!read_blackboard_double(prefix + "target_x", target.x) ||
      !read_blackboard_double(prefix + "target_y", target.y) ||
      !read_blackboard_double(prefix + "target_yaw", target.yaw) ||
      !read_blackboard_int(prefix + "pid_profile", target.pid_profile) ||
      !read_blackboard_double(prefix + "timeout_sec", target.timeout_sec))
  {
    error = "Missing initial place target config: " + target.key;
    return false;
  }
  (void)read_blackboard_double(prefix + "max_vel", target.max_vel);
  (void)read_blackboard_double(prefix + "max_wz", target.max_wz);
  return true;
}

std::string InitialPlaceKFS::target_key_for_command(int command) const
{
  if (command == 4) return "2_left";
  if (command == 6) return "2_right";
  return "2_mid";
}

bool InitialPlaceKFS::read_blackboard_double(const std::string& key, double& value) const
{
  return config().blackboard->get(key, value);
}

bool InitialPlaceKFS::read_blackboard_int(const std::string& key, int& value) const
{
  return config().blackboard->get(key, value);
}

BT::NodeStatus InitialPlaceKFS::fail(const std::string& message)
{
  error_msg_ = message;
  publish_velocity(0.0, 0.0, 0.0);
  setOutput("message", error_msg_);
  config().blackboard->set("last_error", error_msg_);
  config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
  if (node_)
  {
    RCLCPP_ERROR(node_->get_logger(), "[InitialPlaceKFS] %s", error_msg_.c_str());
  }
  stage_ = Stage::IDLE;
  return BT::NodeStatus::FAILURE;
}

void InitialPlaceKFS::onHalted()
{
  publish_velocity(0.0, 0.0, 0.0);
  if (prepare_client_ && prepare_goal_handle_)
  {
    prepare_client_->async_cancel_goal(prepare_goal_handle_);
  }
  if (move_client_ && move_goal_handle_)
  {
    move_client_->async_cancel_goal(move_goal_handle_);
  }
  stage_ = Stage::IDLE;
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[InitialPlaceKFS] Halted");
  }
}

}  // namespace r2_bt
