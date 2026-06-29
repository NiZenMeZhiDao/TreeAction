#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <string>

namespace r2_bt
{

class PublishChassisHeight : public BT::StatefulActionNode
{
public:
  PublishChassisHeight(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using Msg = std_msgs::msg::Float32MultiArray;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Msg>::SharedPtr publisher_;
  std::string publisher_topic_;
  double settle_sec_ = 0.2;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace r2_bt
