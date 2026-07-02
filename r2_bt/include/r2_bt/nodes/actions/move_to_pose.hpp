#pragma once

#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <mutex>

namespace r2_bt
{

class MoveToPose : public BT::StatefulActionNode
{
public:
  MoveToPose(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveToPoseAction>;

  rclcpp_action::Client<MoveToPoseAction>::SharedPtr action_client_;
  std::shared_ptr<GoalHandle> goal_handle_;
  std::shared_ptr<rclcpp::Node> node_;

  bool goal_response_received_;
  bool goal_accepted_;
  bool goal_done_;
  bool feedback_received_;
  bool early_success_;
  BT::NodeStatus result_status_;
  std::string error_msg_;
  double timeout_sec_;
  double early_success_distance_;
  double early_success_yaw_tolerance_;
  double latest_distance_error_;
  double latest_yaw_error_;
  std::chrono::steady_clock::time_point start_time_;
  std::mutex mutex_;
};

}  // namespace r2_bt
