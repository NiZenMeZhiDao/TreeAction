#include "r2_bt/nodes/actions/align.hpp"

#include <cmath>

namespace r2_bt
{

Align::Align(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList Align::providedPorts()
{
  return {
    BT::InputPort<double>("target_x", "Target X in map frame (m)"),
    BT::InputPort<double>("target_y", "Target Y in map frame (m)"),
    BT::InputPort<double>("target_yaw", "Target yaw angle in map frame (rad)"),
    BT::InputPort<double>("max_speed", "Max chassis speed (m/s), default 0.25"),
    BT::InputPort<double>("timeout_sec", 30.0, "Abort action after this many seconds"),
    BT::InputPort<std::string>("server_name", "align", "ROS 2 action server name"),
    BT::OutputPort<std::string>("error_msg", "Error description on failure"),
  };
}

BT::NodeStatus Align::onStart()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    goal_response_received_ = false;
    goal_accepted_ = false;
    goal_done_ = false;
    result_status_ = BT::NodeStatus::FAILURE;
    error_msg_.clear();
  }

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  if (!node_)
  {
    error_msg_ = "Missing ros_node on blackboard";
    return BT::NodeStatus::FAILURE;
  }

  auto res_x = getInput<double>("target_x");
  auto res_y = getInput<double>("target_y");
  auto res_yaw = getInput<double>("target_yaw");
  timeout_sec_ = getInput<double>("timeout_sec").value_or(30.0);

  if (!res_x || !res_y || !res_yaw)
  {
    error_msg_ = "Missing required input port (target_x/y/yaw)";
    RCLCPP_ERROR(node_->get_logger(), "[Align] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  double target_x = res_x.value();
  double target_y = res_y.value();
  double target_yaw_rad = res_yaw.value();

  if (std::isnan(target_x) || std::isnan(target_y) || std::isnan(target_yaw_rad))
  {
    error_msg_ = "Target pose contains NaN";
    RCLCPP_ERROR(node_->get_logger(), "[Align] %s", error_msg_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const std::string server_name = getInput<std::string>("server_name").value_or("align");

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<AlignAction>(node_, server_name);
  }

  if (!action_client_->action_server_is_ready())
  {
    error_msg_ = "Align action server '" + server_name + "' not available";
    RCLCPP_ERROR(node_->get_logger(), "[Align] %s", error_msg_.c_str());
    setOutput("error_msg", error_msg_);
    return BT::NodeStatus::FAILURE;
  }

  auto goal = AlignAction::Goal();
  goal.x = target_x;
  goal.y = target_y;
  goal.yaw_deg = target_yaw_rad * 180.0 / M_PI;

  auto send_goal_options = rclcpp_action::Client<AlignAction>::SendGoalOptions();
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
        error_msg_ = "Goal rejected by Align action server";
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

  start_time_ = std::chrono::steady_clock::now();
  config().blackboard->set("active_action", std::string{"Align"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  action_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "[Align] Goal sent: x=%.3f y=%.3f yaw_deg=%.1f",
              target_x, target_y, goal.yaw_deg);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Align::onRunning()
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
    error_msg_ = "Align timed out";
    setOutput("error_msg", error_msg_);
    config().blackboard->set("last_error", error_msg_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_done_)
  {
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

void Align::onHalted()
{
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[Align] Halted, cancelling goal");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (action_client_ && goal_handle_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_done_ = true;
  result_status_ = BT::NodeStatus::FAILURE;
}

}  // namespace r2_bt
