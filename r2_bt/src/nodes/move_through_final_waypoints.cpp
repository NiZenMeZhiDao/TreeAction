#include "r2_bt/nodes/actions/move_through_final_waypoints.hpp"

#include <cmath>

namespace r2_bt
{

MoveThroughFinalWaypoints::MoveThroughFinalWaypoints(
    const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList MoveThroughFinalWaypoints::providedPorts()
{
  return {
    BT::InputPort<std::string>("height_topic", "t0x0112_final",
                               "Float32MultiArray chassis height topic"),
    BT::InputPort<double>("height_wp1", 60.0,
                          "Height to publish before waypoint 1"),
    BT::InputPort<double>("height_wp3", 20.0,
                          "Height to publish before waypoint 3"),
    BT::InputPort<double>("settle_sec", 0.2,
                          "Seconds to wait after publishing height"),
    BT::InputPort<int>("retry_attempts", 3,
                       "Attempts per waypoint before failing"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus MoveThroughFinalWaypoints::onStart()
{
  if (!node_)
  {
    if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
    {
      setOutput("message", "Missing ros_node on blackboard");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!config().blackboard->get("final_standby_waypoints", waypoints_) ||
      !waypoints_ || waypoints_->empty())
  {
    setOutput("message", "Missing final standby waypoint list");
    return BT::NodeStatus::FAILURE;
  }

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
    setOutput("message", "/move_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }

  height_topic_ = getInput<std::string>("height_topic").value_or("t0x0112_final");
  height_wp1_ = getInput<double>("height_wp1").value_or(60.0);
  height_wp3_ = getInput<double>("height_wp3").value_or(20.0);
  settle_sec_ = getInput<double>("settle_sec").value_or(0.2);
  if (settle_sec_ < 0.0)
  {
    settle_sec_ = 0.0;
  }
  retry_attempts_ = getInput<int>("retry_attempts").value_or(3);
  if (retry_attempts_ < 1)
  {
    retry_attempts_ = 1;
  }

  rclcpp::Publisher<HeightMsg>::SharedPtr shared_publisher;
  std::string shared_topic;
  if (config().blackboard->get("final_chassis_height_publisher", shared_publisher) &&
      config().blackboard->get("final_chassis_height_topic", shared_topic) &&
      shared_publisher && shared_topic == height_topic_)
  {
    height_publisher_ = shared_publisher;
  }
  else
  {
    height_publisher_ = node_->create_publisher<HeightMsg>(height_topic_, 10);
  }

  waypoint_index_ = 0;
  attempt_ = 0;
  error_msg_.clear();
  config().blackboard->set("active_action", std::string{"MoveThroughFinalWaypoints"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  RCLCPP_INFO(node_->get_logger(),
              "[MoveThroughFinalWaypoints] starting %zu waypoint(s)",
              waypoints_->size());
  return begin_current_attempt();
}

BT::NodeStatus MoveThroughFinalWaypoints::begin_current_attempt()
{
  if (!waypoints_ || waypoint_index_ >= waypoints_->size())
  {
    config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
    return BT::NodeStatus::SUCCESS;
  }

  ++attempt_;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_response_received_ = false;
    goal_accepted_ = false;
    goal_done_ = false;
    goal_handle_.reset();
    result_status_ = BT::NodeStatus::FAILURE;
    error_msg_.clear();
  }

  if (waypoint_index_ == 0)
  {
    publish_height(height_wp1_);
    stage_ = Stage::SETTLING_HEIGHT;
    stage_start_time_ = std::chrono::steady_clock::now();
    return settle_sec_ > 0.0 ? BT::NodeStatus::RUNNING : send_current_goal();
  }
  if (waypoint_index_ == 2)
  {
    publish_height(height_wp3_);
    stage_ = Stage::SETTLING_HEIGHT;
    stage_start_time_ = std::chrono::steady_clock::now();
    return settle_sec_ > 0.0 ? BT::NodeStatus::RUNNING : send_current_goal();
  }

  return send_current_goal();
}

BT::NodeStatus MoveThroughFinalWaypoints::send_current_goal()
{
  const auto& waypoint = (*waypoints_)[waypoint_index_];
  if (!std::isfinite(waypoint.target_x) ||
      !std::isfinite(waypoint.target_y) ||
      !std::isfinite(waypoint.target_yaw))
  {
    return finish_failed_attempt("Final waypoint contains non-finite pose");
  }
  if (waypoint.pid_profile != MoveToPoseAction::Goal::PID_PROFILE_SLOW &&
      waypoint.pid_profile != MoveToPoseAction::Goal::PID_PROFILE_FAST)
  {
    return finish_failed_attempt("Invalid pid_profile for final waypoint");
  }

  auto goal = MoveToPoseAction::Goal();
  goal.x = waypoint.target_x;
  goal.y = waypoint.target_y;
  goal.yaw_deg = waypoint.target_yaw * 180.0 / M_PI;
  goal.pid_profile = static_cast<uint8_t>(waypoint.pid_profile);

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
        error_msg_ = "Goal rejected by /move_to_pose";
        result_status_ = BT::NodeStatus::FAILURE;
      }
    };
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(mutex_);
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

  stage_ = Stage::WAITING_GOAL_RESPONSE;
  goal_start_time_ = std::chrono::steady_clock::now();
  action_client_->async_send_goal(goal, send_goal_options);
  RCLCPP_INFO(node_->get_logger(),
              "[MoveThroughFinalWaypoints] wp%zu/%zu attempt %d/%d: "
              "x=%.3f y=%.3f yaw_deg=%.1f pid=%u timeout=%.1f",
              waypoint_index_ + 1, waypoints_->size(),
              attempt_, retry_attempts_,
              waypoint.target_x, waypoint.target_y, goal.yaw_deg,
              goal.pid_profile, waypoint.timeout_sec);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveThroughFinalWaypoints::onRunning()
{
  if (stage_ == Stage::SETTLING_HEIGHT)
  {
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - stage_start_time_).count();
    return elapsed >= settle_sec_ ? send_current_goal() : BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::WAITING_GOAL_RESPONSE)
  {
    bool response_received = false;
    bool goal_accepted = false;
    std::string message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      response_received = goal_response_received_;
      goal_accepted = goal_accepted_;
      message = error_msg_;
    }
    if (!response_received)
    {
      return BT::NodeStatus::RUNNING;
    }
    if (!goal_accepted)
    {
      return finish_failed_attempt(
          message.empty() ? "Goal rejected by /move_to_pose" : message);
    }
    stage_ = Stage::WAITING_RESULT;
    return BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::WAITING_RESULT)
  {
    const auto& waypoint = (*waypoints_)[waypoint_index_];
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - goal_start_time_).count();
    if (waypoint.timeout_sec > 0.0 && elapsed > waypoint.timeout_sec)
    {
      std::shared_ptr<GoalHandle> goal_handle;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_handle = goal_handle_;
      }
      if (action_client_ && goal_handle)
      {
        action_client_->async_cancel_goal(goal_handle);
      }
      return finish_failed_attempt("MoveThroughFinalWaypoints timed out");
    }

    BT::NodeStatus result = BT::NodeStatus::RUNNING;
    std::string message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!goal_done_)
      {
        return BT::NodeStatus::RUNNING;
      }
      result = result_status_;
      message = error_msg_;
    }

    if (result == BT::NodeStatus::SUCCESS)
    {
      attempt_ = 0;
      ++waypoint_index_;
      return begin_current_attempt();
    }
    return finish_failed_attempt(message.empty() ? "MoveToPose failed" : message);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveThroughFinalWaypoints::finish_failed_attempt(
    const std::string& message)
{
  error_msg_ = message;
  if (attempt_ < retry_attempts_)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[MoveThroughFinalWaypoints] wp%zu failed: %s; retrying",
                waypoint_index_ + 1, error_msg_.c_str());
    return begin_current_attempt();
  }

  setOutput("message", error_msg_);
  config().blackboard->set("last_error", error_msg_);
  config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
  RCLCPP_ERROR(node_->get_logger(),
               "[MoveThroughFinalWaypoints] wp%zu failed after %d attempt(s): %s",
               waypoint_index_ + 1, attempt_, error_msg_.c_str());
  return BT::NodeStatus::FAILURE;
}

void MoveThroughFinalWaypoints::publish_height(double height)
{
  if (!height_publisher_)
  {
    return;
  }
  HeightMsg msg;
  msg.data.assign(4, static_cast<float>(height));
  height_publisher_->publish(msg);
  RCLCPP_INFO(node_->get_logger(),
              "[MoveThroughFinalWaypoints] height %.1f -> %s",
              height, height_topic_.c_str());
}

void MoveThroughFinalWaypoints::onHalted()
{
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[MoveThroughFinalWaypoints] halted");
  }
  std::shared_ptr<GoalHandle> goal_handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle = goal_handle_;
  }
  if (action_client_ && goal_handle)
  {
    action_client_->async_cancel_goal(goal_handle);
  }
  stage_ = Stage::IDLE;
}

}  // namespace r2_bt
