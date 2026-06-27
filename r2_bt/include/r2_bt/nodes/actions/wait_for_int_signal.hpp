#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <mutex>
#include <set>
#include <string>

namespace r2_bt
{

class WaitForIntSignal : public BT::StatefulActionNode
{
public:
  WaitForIntSignal(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void signal_callback(const std_msgs::msg::Int32::SharedPtr msg);
  bool expected(int value) const;
  bool configure_subscription();
  std::set<int> parse_expected_values() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  std::string subscribed_topic_;
  std::set<int> expected_values_;
  std::chrono::steady_clock::time_point start_time_;
  double timeout_sec_ = 0.0;

  mutable std::mutex mutex_;
  uint64_t message_sequence_ = 0;
  uint64_t start_sequence_ = 0;
  uint64_t matched_sequence_ = 0;
  int last_value_ = 0;
  int matched_value_ = 0;
};

}  // namespace r2_bt
