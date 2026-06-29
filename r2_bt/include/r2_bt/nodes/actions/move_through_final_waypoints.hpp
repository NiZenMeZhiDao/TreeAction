#pragma once

#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <cstddef>
#include <mutex>
#include <string>

#include "r2_bt/final_waypoint.hpp"

namespace r2_bt
{

class MoveThroughFinalWaypoints : public BT::StatefulActionNode
{
public:
  MoveThroughFinalWaypoints(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveToPoseAction>;
  using HeightMsg = std_msgs::msg::Float32MultiArray;

  enum class Stage
  {
    IDLE,
    SETTLING_HEIGHT,
    WAITING_GOAL_RESPONSE,
    WAITING_RESULT,
  };

  BT::NodeStatus begin_current_attempt();
  BT::NodeStatus send_current_goal();
  BT::NodeStatus finish_failed_attempt(const std::string& message);
  void publish_height(double height);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MoveToPoseAction>::SharedPtr action_client_;
  rclcpp::Publisher<HeightMsg>::SharedPtr height_publisher_;
  std::shared_ptr<GoalHandle> goal_handle_;

  FinalWaypointListPtr waypoints_;
  std::string height_topic_;
  Stage stage_ = Stage::IDLE;
  std::size_t waypoint_index_ = 0;
  int attempt_ = 0;
  int retry_attempts_ = 3;
  double settle_sec_ = 0.2;
  double height_wp1_ = 60.0;
  double height_wp3_ = 20.0;
  std::string error_msg_;
  BT::NodeStatus result_status_ = BT::NodeStatus::FAILURE;

  bool goal_response_received_ = false;
  bool goal_accepted_ = false;
  bool goal_done_ = false;
  std::chrono::steady_clock::time_point stage_start_time_;
  std::chrono::steady_clock::time_point goal_start_time_;
  std::mutex mutex_;
};

}  // namespace r2_bt
