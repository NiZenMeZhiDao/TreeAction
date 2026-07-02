#include "r2_bt/nodes/actions/move_to_pose.hpp"

#include <cmath>

namespace r2_bt
{

MoveToPose::MoveToPose(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
  , goal_response_received_(false)
  , goal_accepted_(false)
  , goal_done_(false)
  , feedback_received_(false)
  , early_success_(false)
  , result_status_(BT::NodeStatus::FAILURE)
  , timeout_sec_(30.0)
  , early_success_distance_(0.0)
  , early_success_yaw_tolerance_(0.0)
  , latest_distance_error_(0.0)
  , latest_yaw_error_(0.0)
{
}

BT::PortsList MoveToPose::providedPorts()
{
  return {
    BT::InputPort<bool>("enabled", true, "If false, skip the action and return SUCCESS"),
    BT::InputPort<double>("target_x", "Target X in map frame (m)"),
    BT::InputPort<double>("target_y", "Target Y in map frame (m)"),
    BT::InputPort<double>("target_yaw", "Target yaw angle in map frame (rad)"),
    BT::InputPort<int>("pid_profile", MoveToPoseAction::Goal::PID_PROFILE_FAST,
                       "PID profile: 0=slow, 1=fast"),
    BT::InputPort<double>("max_vel", 0.0,
                          "Per-goal max linear speed override in m/s; <=0 uses PID profile"),
    BT::InputPort<double>("max_wz", 0.0,
                          "Per-goal max yaw angular speed override in rad/s; <=0 uses PID profile"),
    BT::InputPort<double>("timeout_sec", 30.0, "Abort action after this many seconds"),
    BT::InputPort<double>("early_success_distance", 0.0,
                          "Return SUCCESS early when distance error is below this value"),
    BT::InputPort<double>("early_success_yaw_tolerance", 0.0,
                          "Return SUCCESS early when absolute yaw error is below this value"),
    BT::InputPort<std::string>("frame_id", "map", "Coordinate frame for target pose"),
    BT::OutputPort<std::string>("error_msg", "Error description on failure"),
  };
}

BT::NodeStatus MoveToPose::onStart()
{
  const bool enabled = getInput<bool>("enabled").value_or(true);
  if (!enabled)
  {
    error_msg_.clear();
    setOutput("error_msg", std::string{});
    return BT::NodeStatus::SUCCESS;
  }

  goal_done_ = false;
  goal_response_received_ = false;
  goal_accepted_ = false;
  feedback_received_ = false;
  early_success_ = false;
  result_status_ = BT::NodeStatus::FAILURE;
  error_msg_.clear();
  goal_handle_.reset();

  if (!node_)
  {
    if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
    {
      error_msg_ = "Missing ros_node on blackboard";
      return BT::NodeStatus::FAILURE;
    }
  }

  auto res_x = getInput<double>("target_x");
  auto res_y = getInput<double>("target_y");
  auto res_yaw = getInput<double>("target_yaw");
  const int pid_profile =
      getInput<int>("pid_profile").value_or(MoveToPoseAction::Goal::PID_PROFILE_FAST);
  const double max_vel = getInput<double>("max_vel").value_or(0.0);
  const double max_wz = getInput<double>("max_wz").value_or(0.0);
  timeout_sec_ = getInput<double>("timeout_sec").value_or(30.0);
  early_success_distance_ =
      getInput<double>("early_success_distance").value_or(0.0);
  early_success_yaw_tolerance_ =
      getInput<double>("early_success_yaw_tolerance").value_or(0.0);
  latest_distance_error_ = 0.0;
  latest_yaw_error_ = 0.0;

  if (!res_x || !res_y || !res_yaw)
  {
    error_msg_ = "Missing required input port (target_x/y/yaw)";
    RCLCPP_ERROR(node_->get_logger(), "[MoveToPose] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  double target_x = res_x.value();
  double target_y = res_y.value();
  double target_yaw = res_yaw.value();
  std::string frame_id = getInput<std::string>("frame_id").value_or("map");

  if (pid_profile != MoveToPoseAction::Goal::PID_PROFILE_SLOW &&
      pid_profile != MoveToPoseAction::Goal::PID_PROFILE_FAST)
  {
    error_msg_ = "Invalid pid_profile for MoveToPose; expected 0(slow) or 1(fast)";
    RCLCPP_ERROR(node_->get_logger(), "[MoveToPose] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (std::isnan(target_x) || std::isnan(target_y) || std::isnan(target_yaw))
  {
    error_msg_ = "Target pose contains NaN";
    RCLCPP_ERROR(node_->get_logger(), "[MoveToPose] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(max_vel) || max_vel < 0.0)
  {
    error_msg_ = "Invalid max_vel for MoveToPose; expected finite value >= 0";
    RCLCPP_ERROR(node_->get_logger(), "[MoveToPose] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(max_wz) || max_wz < 0.0)
  {
    error_msg_ = "Invalid max_wz for MoveToPose; expected finite value >= 0";
    RCLCPP_ERROR(node_->get_logger(), "[MoveToPose] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(early_success_distance_) || early_success_distance_ < 0.0)
  {
    error_msg_ = "Invalid early_success_distance for MoveToPose; expected finite value >= 0";
    RCLCPP_ERROR(node_->get_logger(), "[MoveToPose] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(early_success_yaw_tolerance_) ||
      early_success_yaw_tolerance_ < 0.0)
  {
    error_msg_ = "Invalid early_success_yaw_tolerance for MoveToPose; expected finite value >= 0";
    RCLCPP_ERROR(node_->get_logger(), "[MoveToPose] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto goal = MoveToPoseAction::Goal();
  goal.x = target_x;
  goal.y = target_y;
  goal.yaw_deg = target_yaw * 180.0 / M_PI;
  goal.pid_profile = static_cast<uint8_t>(pid_profile);
  goal.max_vel = max_vel;
  goal.max_wz = max_wz;

  if (!action_client_)
  {
    rclcpp_action::Client<MoveToPoseAction>::SharedPtr shared_client;
    if (config().blackboard->get("move_to_pose_client", shared_client) && shared_client)
    {
      action_client_ = shared_client;
    }
    else
    {
      action_client_ = rclcpp_action::create_client<MoveToPoseAction>(node_, "move_to_pose");
    }
  }

  if (!action_client_->action_server_is_ready())
  {
    error_msg_ = "/move_to_pose action server not available";
    RCLCPP_ERROR(node_->get_logger(), "[MoveToPose] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto send_goal_options = rclcpp_action::Client<MoveToPoseAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const std::shared_ptr<GoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_response_received_ = true;
      goal_handle_ = goal_handle;
      goal_accepted_ = static_cast<bool>(goal_handle);
      if (!goal_handle)
      {
        goal_done_ = true;
        result_status_ = BT::NodeStatus::FAILURE;
        error_msg_ = "Goal rejected by action server";
      }
    };
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (early_success_)
      {
        return;
      }
      goal_done_ = true;
      result_status_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                        result.result && result.result->success)
                           ? BT::NodeStatus::SUCCESS
                           : BT::NodeStatus::FAILURE;
      if (result_status_ == BT::NodeStatus::FAILURE && result.result)
      {
        error_msg_ = result.result->message;
      }
    };
  send_goal_options.feedback_callback =
    [this](std::shared_ptr<GoalHandle>,
           const std::shared_ptr<const MoveToPoseAction::Feedback> feedback) {
      if (!feedback)
      {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      feedback_received_ = true;
      latest_distance_error_ = feedback->distance_error;
      latest_yaw_error_ = std::abs(feedback->yaw_error_deg) * M_PI / 180.0;
    };

  start_time_ = std::chrono::steady_clock::now();
  config().blackboard->set("active_action", std::string{"MoveToPose"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  action_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
	              "[MoveToPose] goal sent: x=%.3f y=%.3f yaw_deg=%.1f "
	              "pid_profile=%u max_vel=%.3f max_wz=%.3f frame=%s "
	              "early=(dist<=%.3f yaw<=%.3f)",
	              target_x, target_y, goal.yaw_deg, goal.pid_profile,
	              goal.max_vel, goal.max_wz, frame_id.c_str(),
	              early_success_distance_, early_success_yaw_tolerance_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPose::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!goal_response_received_)
  {
    return BT::NodeStatus::RUNNING;
  }
  if (!goal_accepted_)
  {
    setOutput("error_msg", error_msg_);
    config().blackboard->set("last_error", error_msg_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (timeout_sec_ > 0.0 && elapsed > timeout_sec_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
    error_msg_ = "MoveToPose timed out";
    setOutput("error_msg", error_msg_);
    config().blackboard->set("last_error", error_msg_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_done_)
  {
    const bool distance_triggered =
        early_success_distance_ > 0.0 &&
        latest_distance_error_ <= early_success_distance_;
    const bool yaw_triggered =
        early_success_yaw_tolerance_ > 0.0 &&
        latest_yaw_error_ <= early_success_yaw_tolerance_;
    if (feedback_received_ && goal_handle_ && (distance_triggered || yaw_triggered))
    {
      action_client_->async_cancel_goal(goal_handle_);
      early_success_ = true;
      goal_done_ = true;
      result_status_ = BT::NodeStatus::SUCCESS;
      error_msg_.clear();
      setOutput("error_msg", std::string{});
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      RCLCPP_INFO(node_->get_logger(),
                  "[MoveToPose] early success by %s: distance_error=%.3f yaw_error=%.3f",
                  distance_triggered ? "distance" : "yaw",
                  latest_distance_error_, latest_yaw_error_);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
  config().blackboard->set(
    "execution_state",
    result_status_ == BT::NodeStatus::SUCCESS ? std::string{"ACTION_SUCCESS"}
                                              : std::string{"ACTION_FAILED"});
  if (result_status_ == BT::NodeStatus::FAILURE)
  {
    setOutput("error_msg", error_msg_);
    config().blackboard->set("last_error", error_msg_);
  }
  return result_status_;
}

void MoveToPose::onHalted()
{
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[MoveToPose] Halted, cancelling goal");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_done_ = true;
  result_status_ = BT::NodeStatus::FAILURE;
}

}  // namespace r2_bt
