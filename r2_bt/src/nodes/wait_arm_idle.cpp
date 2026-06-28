#include "r2_bt/nodes/actions/wait_arm_idle.hpp"

namespace r2_bt
{

WaitArmIdle::WaitArmIdle(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitArmIdle::providedPorts()
{
  return {
    BT::InputPort<double>("timeout_sec", 30.0, "Max time to wait for arm background action"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus WaitArmIdle::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  config().blackboard->set("active_action", std::string{"WaitArmIdle"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});

  // 首次 tick 初始化订阅
  if (!node_)
  {
    if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("WaitArmIdle"), "Missing ros_node on blackboard");
      return BT::NodeStatus::FAILURE;
    }

    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/arm_runtime_state", rclcpp::QoS(10).reliable(),
        std::bind(&WaitArmIdle::state_callback, this, std::placeholders::_1));
  }

  state_received_ = false;
  last_state_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitArmIdle::onRunning()
{
  auto timeout = getInput<double>("timeout_sec").value_or(30.0);
  const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();

  if (timeout > 0.0 && elapsed > timeout)
  {
    const std::string msg = "Timed out waiting for arm background action";
    setOutput("message", msg);
    config().blackboard->set("last_error", msg);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    if (node_)
    {
      RCLCPP_WARN(node_->get_logger(), "[WaitArmIdle] %s", msg.c_str());
    }
    return BT::NodeStatus::FAILURE;
  }

  if (state_received_ && last_state_)
  {
    const std::string msg = "Arm background action completed";
    setOutput("message", msg);
    config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
    if (node_)
    {
      RCLCPP_INFO(node_->get_logger(), "[WaitArmIdle] %s", msg.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitArmIdle::onHalted()
{
}

void WaitArmIdle::state_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  state_received_ = true;
  last_state_ = msg->data;
}

}  // namespace r2_bt
