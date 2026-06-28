#include "r2_bt/nodes/actions/go_to_pose.hpp"

#include <cmath>

namespace r2_bt
{

GoToPose::GoToPose(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList GoToPose::providedPorts()
{
  return {
    BT::InputPort<double>("target_x", "Target X in map frame (m)"),
    BT::InputPort<double>("target_y", "Target Y in map frame (m)"),
    BT::InputPort<double>("target_yaw", "Target yaw in map frame (rad)"),
    BT::InputPort<double>("timeout_sec", 0.0, "Nav2 timeout, 0 = no timeout"),
    BT::InputPort<std::string>("frame_id", "map", "Target frame id"),
    BT::InputPort<std::string>("server_name", "/go_to_pose", "GoToPose action name"),
    BT::OutputPort<std::string>("message", "Result or error message"),
  };
}

BT::NodeStatus GoToPose::onStart()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    goal_response_received_ = false;
    goal_accepted_ = false;
    goal_done_ = false;
    result_status_ = BT::NodeStatus::FAILURE;
    result_message_.clear();
  }

  if (!node_)
  {
    if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
    {
      result_message_ = "Missing ros_node on blackboard";
      setOutput("message", result_message_);
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto res_x = getInput<double>("target_x");
  const auto res_y = getInput<double>("target_y");
  const auto res_yaw = getInput<double>("target_yaw");
  if (!res_x || !res_y || !res_yaw)
  {
    result_message_ = "Missing required GoToPose input target_x/y/yaw";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  const double target_x = res_x.value();
  const double target_y = res_y.value();
  const double target_yaw = res_yaw.value();
  if (std::isnan(target_x) || std::isnan(target_y) || std::isnan(target_yaw))
  {
    result_message_ = "GoToPose target contains NaN";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  timeout_sec_ = getInput<double>("timeout_sec").value_or(0.0);
  const auto frame_id = getInput<std::string>("frame_id").value_or("map");
  const auto server_name = getInput<std::string>("server_name").value_or("/go_to_pose");

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<ActionT>(node_, server_name);
  }
  if (!action_client_->action_server_is_ready())
  {
    result_message_ = server_name + " action server not available";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    RCLCPP_ERROR(node_->get_logger(), "[GoToPose] %s", result_message_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto goal = ActionT::Goal();
  goal.x = target_x;
  goal.y = target_y;
  goal.yaw = target_yaw;
  goal.frame_id = frame_id;
  goal.timeout_sec = static_cast<float>(timeout_sec_);

  auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();
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
        result_message_ = "Goal rejected by GoToPose server";
      }
    };
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_done_ = true;
      if (result.result)
      {
        result_message_ = result.result->message;
      }
      result_status_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                        result.result && result.result->success)
                         ? BT::NodeStatus::SUCCESS
                         : BT::NodeStatus::FAILURE;
    };

  start_time_ = std::chrono::steady_clock::now();
  config().blackboard->set("active_action", std::string{"GoToPose"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  action_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "[GoToPose] goal sent: x=%.3f y=%.3f yaw=%.3f timeout=%.1f",
              target_x, target_y, target_yaw, timeout_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!goal_response_received_)
  {
    return BT::NodeStatus::RUNNING;
  }
  if (!goal_accepted_)
  {
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (timeout_sec_ > 0.0 && elapsed > timeout_sec_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
    result_message_ = "GoToPose timed out";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_done_)
  {
    return BT::NodeStatus::RUNNING;
  }

  setOutput("message", result_message_);
  config().blackboard->set(
    "execution_state",
    result_status_ == BT::NodeStatus::SUCCESS ? std::string{"ACTION_SUCCESS"}
                                              : std::string{"ACTION_FAILED"});
  if (result_status_ == BT::NodeStatus::FAILURE)
  {
    config().blackboard->set("last_error", result_message_);
  }
  return result_status_;
}

void GoToPose::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (action_client_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[GoToPose] Halted");
  }
}

}  // namespace r2_bt
