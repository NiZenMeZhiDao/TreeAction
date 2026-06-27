#pragma once

#include <behaviortree_cpp/action_node.h>
#include <pick_action_interfaces/action/pick_sequence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <mutex>
#include <string>

namespace r2_bt
{

class PickAction : public BT::StatefulActionNode
{
public:
  PickAction(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using ActionT = pick_action_interfaces::action::PickSequence;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::shared_ptr<GoalHandle> goal_handle_;
  rclcpp::Node::SharedPtr node_;
  std::string server_name_;

  bool goal_response_received_ = false;
  bool goal_accepted_ = false;
  bool goal_done_ = false;
  BT::NodeStatus result_status_ = BT::NodeStatus::FAILURE;
  std::string result_message_;
  double timeout_sec_ = 45.0;
  std::chrono::steady_clock::time_point start_time_;
  std::mutex mutex_;
};

}  // namespace r2_bt
