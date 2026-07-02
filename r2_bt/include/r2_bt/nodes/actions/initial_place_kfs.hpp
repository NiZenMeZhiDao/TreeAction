#pragma once

#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <r2_interfaces/action/prepare_kfs.hpp>
#include <r2_interfaces/srv/tool_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <future>
#include <mutex>
#include <set>
#include <string>

namespace r2_bt
{

class InitialPlaceKFS : public BT::StatefulActionNode
{
public:
  InitialPlaceKFS(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using PrepareAction = r2_interfaces::action::PrepareKFS;
  using PrepareGoalHandle = rclcpp_action::ClientGoalHandle<PrepareAction>;
  using MoveAction = action_of_motion_interfaces::action::MoveToPose;
  using MoveGoalHandle = rclcpp_action::ClientGoalHandle<MoveAction>;
  using ToolService = r2_interfaces::srv::ToolAction;
  using VelocityMsg = std_msgs::msg::Float32MultiArray;

  enum class Stage
  {
    IDLE,
    WAIT_PREPARE_RESPONSE,
    WAIT_PREPARE_RESULT,
    WAIT_MOVE_RESPONSE,
    WAIT_MOVE_RESULT,
    FORWARD_APPROACH,
    WAIT_PLACE,
  };

  struct Target
  {
    int command = 5;
    std::string key = "2_mid";
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    int pid_profile = 1;
    double max_vel = 0.0;
    double max_wz = 0.0;
    double timeout_sec = 30.0;
  };

  bool configure();
  bool read_target(int command, Target& target, std::string& error) const;
  bool read_blackboard_double(const std::string& key, double& value) const;
  bool read_blackboard_int(const std::string& key, int& value) const;
  std::string target_key_for_command(int command) const;
  bool send_prepare();
  bool send_motion(int command);
  bool maybe_apply_override();
  BT::NodeStatus fail(const std::string& message);
  void start_forward_or_place();
  void start_place();
  BT::NodeStatus tick_forward();
  void publish_velocity(double vx, double vy, double wz);
  void deck_callback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PrepareAction>::SharedPtr prepare_client_;
  rclcpp_action::Client<MoveAction>::SharedPtr move_client_;
  rclcpp::Client<ToolService>::SharedPtr tool_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr deck_sub_;
  rclcpp::Publisher<VelocityMsg>::SharedPtr velocity_pub_;
  std::shared_ptr<PrepareGoalHandle> prepare_goal_handle_;
  std::shared_ptr<MoveGoalHandle> move_goal_handle_;
  rclcpp::Client<ToolService>::SharedFuture place_future_;

  Stage stage_ = Stage::IDLE;
  Target active_target_;
  std::string error_msg_;
  std::string deck_topic_ = "/aruco_comm/tx_id";
  std::string prepare_action_name_ = "/prepare_kfs";
  std::string tool_service_name_ = "/ares_tool_node/tool_action";
  std::string place_action_ = "arm_place_mid";
  std::string velocity_topic_ = "/t0x0111_pid";
  std::array<double, 4> place_args_{0.0, 0.0, 0.0, 0.0};
  std::set<int> override_commands_{4, 5, 6};
  int default_command_ = 5;
  int pending_command_ = 5;
  int motion_generation_ = 0;
  bool enabled_ = true;
  bool forward_enabled_ = false;
  double prepare_timeout_sec_ = 30.0;
  double forward_speed_mps_ = 0.15;
  double forward_duration_sec_ = 0.5;
  double forward_publish_rate_hz_ = 50.0;
  double place_timeout_sec_ = 30.0;
  std::chrono::steady_clock::time_point stage_start_time_;
  std::chrono::steady_clock::time_point forward_next_publish_;

  bool prepare_response_received_ = false;
  bool prepare_accepted_ = false;
  bool prepare_done_ = false;
  BT::NodeStatus prepare_status_ = BT::NodeStatus::FAILURE;
  std::string prepare_message_;
  bool move_response_received_ = false;
  bool move_accepted_ = false;
  bool move_done_ = false;
  BT::NodeStatus move_status_ = BT::NodeStatus::FAILURE;
  std::string move_message_;
  bool place_sent_ = false;
  std::mutex mutex_;
};

}  // namespace r2_bt
