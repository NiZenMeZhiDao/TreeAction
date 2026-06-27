#pragma once

#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <r2_interfaces/action/suspension_control.hpp>
#include <r2_interfaces/srv/tool_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <mutex>
#include <string>

namespace r2_bt
{

/**
 * @brief 梅林区 Fetch 动作节点（一体化）
 *
 * 执行模型:
 *   1. MoveToPose(grasp)：移动到 KFS 抓取位姿
 *   2. (仅 height_diff==400) 悬挂抬升至 200mm
 *   3. arm_grasp（内嵌 tool_node service 调用）：传入 args[0]=height_diff
 *   4. (仅 height_diff==400) 悬挂复位至 30mm
 *
 * tool_node 只做简单映射：正值→200，负值→-200。
 */
class MeilinFetch : public BT::StatefulActionNode
{
public:
  MeilinFetch(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus drive();

  // ---- MoveToPose Action Client ----
  enum class ActionState { IDLE, ACTIVE, DONE, FAILED };
  void sendMoveToPoseGoal();
  bool isAlignDone() const;
  bool isAlignFailed() const;
  void failAlignIfTimedOut();

  ActionState align_state_ = ActionState::IDLE;
  std::string align_message_;
  std::mutex align_mutex_;

  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPoseAction>;

  rclcpp_action::Client<MoveToPoseAction>::SharedPtr align_client_;
  std::shared_ptr<MoveToPoseGoalHandle> align_goal_handle_;
  std::chrono::steady_clock::time_point align_start_time_;

  // ---- Suspension Action Client（仅 height_diff==400）----
  using SuspensionAction = r2_interfaces::action::SuspensionControl;
  using SuspensionGoalHandle = rclcpp_action::ClientGoalHandle<SuspensionAction>;

  rclcpp_action::Client<SuspensionAction>::SharedPtr suspension_client_;
  std::shared_ptr<SuspensionGoalHandle> suspension_goal_handle_;
  bool suspension_goal_done_ = false;
  BT::NodeStatus suspension_result_ = BT::NodeStatus::FAILURE;
  std::string suspension_message_;
  std::mutex suspension_mutex_;
  bool suspension_needed_ = false;   // height_diff==400
  bool suspension_triggered_ = false; // sent lift goal
  bool suspension_resetting_ = false; // sent reset goal

  void sendSuspensionGoal(double target_height_mm);

  // ---- tool_node Service Client（arm_grasp）----
  using ToolActionSrv = r2_interfaces::srv::ToolAction;

  rclcpp::Client<ToolActionSrv>::SharedPtr tool_client_;
  bool tool_request_sent_ = false;
  bool tool_request_done_ = false;
  BT::NodeStatus tool_result_ = BT::NodeStatus::FAILURE;
  std::string tool_message_;
  std::chrono::steady_clock::time_point tool_start_time_;
  std::mutex tool_mutex_;
  double tool_timeout_sec_ = 30.0;

  void sendToolGrasp();
  void failToolIfTimedOut();

  // ---- 阶段 ----
  enum class Phase { START, WAIT_ALIGN, START_SUSPENSION, WAIT_SUSPENSION,
                     START_TOOL, WAIT_TOOL, RESET_SUSPENSION, WAIT_RESET, DONE };
  Phase phase_ = Phase::START;

  rclcpp::Node::SharedPtr node_;

  // planner 原始输入
  int kfs_row_ = 0;
  int kfs_col_ = 0;
  double height_diff_ = 0.0;
  double target_yaw_ = 0.0;

  // 内部计算
  double grasp_x_ = 0.0;
  double grasp_y_ = 0.0;

  static constexpr double kSuspension400LiftHeight = 200.0;
  static constexpr double kSuspensionNormalHeight = 30.0;
  static constexpr double kSuspensionTimeoutSec = 10.0;
};

}  // namespace r2_bt
