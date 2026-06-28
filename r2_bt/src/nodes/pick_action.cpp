#include "r2_bt/nodes/actions/pick_action.hpp"

#include <cstdint>

namespace r2_bt
{

PickAction::PickAction(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList PickAction::providedPorts()
{
  return {
    BT::InputPort<int>("expected_count", 3, "Number of spear targets expected by pick_action"),
    BT::InputPort<double>("timeout_sec", 45.0, "Abort pick_action after this many seconds"),
    BT::InputPort<std::string>("server_name", "/pick_action", "ROS 2 pick action name"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus PickAction::onStart()
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

  if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
  {
    result_message_ = "Missing ros_node on blackboard";
    setOutput("message", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  const int expected_count = getInput<int>("expected_count").value_or(3);
  if (expected_count <= 0 || expected_count > 127)
  {
    result_message_ = "Invalid PickAction expected_count; expected 1..127";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  timeout_sec_ = getInput<double>("timeout_sec").value_or(45.0);
  const auto server_name = getInput<std::string>("server_name").value_or("/pick_action");

  if (!action_client_ || server_name_ != server_name)
  {
    server_name_ = server_name;
    rclcpp_action::Client<ActionT>::SharedPtr shared_client;
    std::string shared_client_name;
    if (config().blackboard->get("pick_action_client", shared_client) &&
        config().blackboard->get("pick_action_client_name", shared_client_name) &&
        shared_client && shared_client_name == server_name_)
    {
      action_client_ = shared_client;
      RCLCPP_DEBUG(node_->get_logger(),
                   "[PickAction] Reusing prewarmed action client: %s",
                   server_name_.c_str());
    }
    else
    {
      action_client_ = rclcpp_action::create_client<ActionT>(node_, server_name_);
      RCLCPP_WARN(node_->get_logger(),
                  "[PickAction] Created local action client for %s; "
                  "prewarmed client was missing or name-mismatched",
                  server_name_.c_str());
    }
  }

  if (!action_client_->action_server_is_ready())
  {
    result_message_ = "PickAction action server not available: " + server_name_;
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    RCLCPP_ERROR(node_->get_logger(), "[PickAction] %s", result_message_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto goal = ActionT::Goal();
  goal.expected_count = static_cast<int8_t>(expected_count);

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
        result_message_ = "Goal rejected by pick action server";
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
  config().blackboard->set("active_action", std::string{"PickAction"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  action_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(), "[PickAction] Goal sent: server=%s expected_count=%d timeout=%.1f",
              server_name_.c_str(), expected_count, timeout_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickAction::onRunning()
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
    result_message_ = "PickAction timed out";
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

void PickAction::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (action_client_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[PickAction] Halted");
  }
}

}  // namespace r2_bt
