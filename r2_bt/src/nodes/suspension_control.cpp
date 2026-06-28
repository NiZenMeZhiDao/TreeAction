#include "r2_bt/nodes/actions/suspension_control.hpp"

namespace r2_bt
{

SuspensionControl::SuspensionControl(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList SuspensionControl::providedPorts()
{
  return {
    BT::InputPort<int>("mode", 0, "0=AUTO, 1=CLIMB_UP, 2=CLIMB_DOWN, 3=DIRECT"),
    BT::InputPort<int>("direction", 0, "0=FORWARD, 1=LEFT, 2=RIGHT, 3=BACKWARD"),
    BT::InputPort<double>("height", 0.0, "Stair height; 0 means auto/default"),
    BT::InputPort<double>("timeout_sec", 30.0, "Abort action after this many seconds"),
    BT::InputPort<std::string>("server_name", "suspension_control", "ROS 2 action name"),
    BT::OutputPort<std::string>("message"),
    BT::OutputPort<int>("final_state"),
  };
}

BT::NodeStatus SuspensionControl::onStart()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_accepted_ = false;
    goal_done_ = false;
    result_status_ = BT::NodeStatus::FAILURE;
    result_message_.clear();
    final_state_ = 0;
    goal_handle_.reset();
  }

  if (!node_)
  {
    if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
    {
      setOutput("message", "Missing ros_node on blackboard");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto mode = getInput<int>("mode").value_or(0);
  auto direction = getInput<int>("direction").value_or(0);
  auto height = getInput<double>("height").value_or(0.0);
  auto timeout_sec = getInput<double>("timeout_sec").value_or(30.0);
  auto server_name = getInput<std::string>("server_name").value_or("suspension_control");

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<SuspensionAction>(node_, server_name);
  }

  if (!action_client_->action_server_is_ready())
  {
    result_message_ = "SuspensionControl action server not available";
    RCLCPP_ERROR(node_->get_logger(), "[SuspensionControl] %s", result_message_.c_str());
    setOutput("message", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  auto goal = SuspensionAction::Goal();
  goal.mode = static_cast<uint8_t>(mode);
  goal.direction = static_cast<uint8_t>(direction);
  goal.height = static_cast<float>(height);
  goal.timeout_sec = static_cast<float>(timeout_sec);

  auto send_goal_options = rclcpp_action::Client<SuspensionAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const std::shared_ptr<GoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle_ = goal_handle;
      goal_accepted_ = static_cast<bool>(goal_handle);
      if (!goal_handle)
      {
        goal_done_ = true;
        result_message_ = "Goal rejected by suspension action server";
        result_status_ = BT::NodeStatus::FAILURE;
      }
    };
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_done_ = true;
      if (result.result)
      {
        result_message_ = result.result->message;
        final_state_ = result.result->final_state;
      }
      result_status_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                        result.result && result.result->success)
                         ? BT::NodeStatus::SUCCESS
                         : BT::NodeStatus::FAILURE;
    };

  action_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "[SuspensionControl] Goal sent: mode=%d direction=%d height=%.1f timeout=%.1f",
              mode, direction, height, timeout_sec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SuspensionControl::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!goal_done_)
  {
    return BT::NodeStatus::RUNNING;
  }

  setOutput("message", result_message_);
  setOutput("final_state", final_state_);
  return result_status_;
}

void SuspensionControl::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[SuspensionControl] Halted, cancelling goal");
  }
  if (action_client_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_done_ = true;
  result_status_ = BT::NodeStatus::FAILURE;
}

}  // namespace r2_bt
