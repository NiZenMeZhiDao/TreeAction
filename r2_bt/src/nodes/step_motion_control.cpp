#include "r2_bt/nodes/actions/step_motion_control.hpp"

namespace r2_bt
{

StepMotionControl::StepMotionControl(const std::string& name,
                                     const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList StepMotionControl::providedPorts()
{
  return {
    BT::InputPort<bool>("enabled", true, "If false, skip the action and return SUCCESS"),
    BT::InputPort<int>("mode", 0, "0=AUTO, 1=CLIMB_UP, 2=CLIMB_DOWN, 3=DIRECT_HEIGHT"),
    BT::InputPort<int>("direction", 0, "0=FORWARD, 1=LEFT, 2=RIGHT"),
    BT::InputPort<double>("height", 0.0, "Step/direct height in mm"),
    BT::InputPort<double>("correction_x", 0.0, "Lateral correction reference X"),
    BT::InputPort<double>("correction_y", 0.0, "Lateral correction reference Y"),
    BT::InputPort<double>("correction_yaw_deg", 0.0, "Yaw correction reference in degrees"),
    BT::InputPort<double>("timeout_sec", 10.0, "Abort action after this many seconds"),
    BT::InputPort<std::string>("server_name", "step_motion_control", "ROS 2 action name"),
    BT::OutputPort<std::string>("message"),
    BT::OutputPort<int>("final_state"),
  };
}

BT::NodeStatus StepMotionControl::onStart()
{
  const bool enabled = getInput<bool>("enabled").value_or(true);
  if (!enabled)
  {
    setOutput("message", std::string{"Step skipped"});
    setOutput("final_state", 0);
    return BT::NodeStatus::SUCCESS;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
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
      setOutput("message", std::string{"Missing ros_node on blackboard"});
      return BT::NodeStatus::FAILURE;
    }
  }

  const int mode = getInput<int>("mode").value_or(0);
  const int direction = getInput<int>("direction").value_or(0);
  const double height = getInput<double>("height").value_or(0.0);
  const double correction_x = getInput<double>("correction_x").value_or(0.0);
  const double correction_y = getInput<double>("correction_y").value_or(0.0);
  const double correction_yaw_deg =
      getInput<double>("correction_yaw_deg").value_or(0.0);
  const double timeout_sec = getInput<double>("timeout_sec").value_or(10.0);
  const auto server_name =
      getInput<std::string>("server_name").value_or("step_motion_control");

  if (!action_client_)
  {
    rclcpp_action::Client<StepMotionAction>::SharedPtr shared_client;
    if (server_name == "step_motion_control" &&
        config().blackboard->get("step_motion_client", shared_client) && shared_client)
    {
      action_client_ = shared_client;
    }
    else
    {
      action_client_ = rclcpp_action::create_client<StepMotionAction>(node_, server_name);
    }
  }

  if (!action_client_->action_server_is_ready())
  {
    result_message_ = "/step_motion_control action server not available";
    RCLCPP_ERROR(node_->get_logger(), "[StepMotionControl] %s", result_message_.c_str());
    setOutput("message", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  auto goal = StepMotionAction::Goal();
  goal.mode = static_cast<uint8_t>(mode);
  goal.direction = static_cast<uint8_t>(direction);
  goal.height = static_cast<float>(height);
  goal.correction_x = correction_x;
  goal.correction_y = correction_y;
  goal.correction_yaw_deg = correction_yaw_deg;
  goal.timeout_sec = static_cast<float>(timeout_sec);

  auto opts = rclcpp_action::Client<StepMotionAction>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const std::shared_ptr<GoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        goal_done_ = true;
        result_message_ = "Goal rejected by step motion action server";
        result_status_ = BT::NodeStatus::FAILURE;
      }
    };
  opts.result_callback =
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

  config().blackboard->set("active_action", std::string{"StepMotionControl"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  action_client_->async_send_goal(goal, opts);

  RCLCPP_INFO(node_->get_logger(),
              "[StepMotionControl] goal sent: mode=%d direction=%d height=%.1f "
              "corr=(%.3f,%.3f,%.1fdeg) timeout=%.1f",
              mode, direction, height, correction_x, correction_y,
              correction_yaw_deg, timeout_sec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StepMotionControl::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!goal_done_)
  {
    return BT::NodeStatus::RUNNING;
  }

  setOutput("message", result_message_);
  setOutput("final_state", final_state_);
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

void StepMotionControl::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[StepMotionControl] Halted, cancelling goal");
  }
  if (action_client_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_done_ = true;
  result_status_ = BT::NodeStatus::FAILURE;
}

}  // namespace r2_bt
