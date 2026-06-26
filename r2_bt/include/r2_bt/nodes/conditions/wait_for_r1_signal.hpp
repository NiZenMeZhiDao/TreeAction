#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <mutex>
#include <string>

namespace r2_bt
{

/**
 * @brief 等待 R1 就位信号的 Condition 节点
 *
 * 订阅一个 IO 信号 topic（默认 /r1_ready），当收到 true 时返回 SUCCESS。
 * 用于 DOCK segment 中等待 R1 准备好接收矛头。
 */
class WaitForR1Signal : public BT::ConditionNode
{
public:
  WaitForR1Signal(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void signal_callback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::mutex mutex_;
  bool signal_received_ = false;
  bool last_signal_value_ = false;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace r2_bt
