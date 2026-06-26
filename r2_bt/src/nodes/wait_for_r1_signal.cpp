#include "r2_bt/nodes/conditions/wait_for_r1_signal.hpp"

namespace r2_bt
{

WaitForR1Signal::WaitForR1Signal(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList WaitForR1Signal::providedPorts()
{
  return {
    BT::InputPort<std::string>("signal_topic", "/r1_ready",
                               "Topic name for R1 ready signal (std_msgs/Bool)"),
    BT::InputPort<double>("timeout_sec", 60.0,
                          "Max wait time before returning FAILURE (0 = no timeout)"),
  };
}

BT::NodeStatus WaitForR1Signal::tick()
{
  // 首次 tick 初始化
  if (!node_)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
    if (!node_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("WaitForR1Signal"), "Missing ros_node on blackboard");
      return BT::NodeStatus::FAILURE;
    }

    const auto topic = getInput<std::string>("signal_topic").value_or("/r1_ready");

    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        topic, rclcpp::QoS(10).reliable(),
        std::bind(&WaitForR1Signal::signal_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "[WaitForR1Signal] Waiting for R1 signal on topic: %s",
                topic.c_str());

    start_time_ = std::chrono::steady_clock::now();
  }

  // 检查超时
  const auto timeout_sec = getInput<double>("timeout_sec").value_or(60.0);
  if (timeout_sec > 0.0)
  {
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed > timeout_sec)
    {
      if (node_)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[WaitForR1Signal] Timeout after %.1f seconds, R1 signal not received",
                    timeout_sec);
      }
      return BT::NodeStatus::FAILURE;
    }
  }

  // 检查信号状态
  std::lock_guard<std::mutex> lock(mutex_);
  if (signal_received_ && last_signal_value_)
  {
    if (node_)
    {
      RCLCPP_INFO(node_->get_logger(), "[WaitForR1Signal] R1 ready signal received!");
    }
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForR1Signal::signal_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->data && !last_signal_value_)
  {
    signal_received_ = true;
  }
  last_signal_value_ = msg->data;
}

}  // namespace r2_bt
