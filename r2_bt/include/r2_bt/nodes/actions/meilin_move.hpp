#pragma once

#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <r2_interfaces/action/suspension_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <mutex>
#include <string>

namespace r2_bt
{

/**
 * @brief 梅林区 Move 动作节点
 *
 * 执行模型:
 *   onStart: 完成全部计算，立即发出第一批 action（pre_align 和/或 climb 并行）
 *   onRunning: 检查当前 action 是否完成，完成则推进到下一批
 *   - MoveToPose 串行: pre_align → target_align
 *   - 悬挂并行: SuspensionControl 与 MoveToPose 同时进行
 *
 * 悬挂调用: 使用 rclcpp_action::Client<SuspensionControl> 异步发送 goal，
 *   参考 r2_hardware SuspensionActionServer（原 active_suspension_control 状态机）。
 */
class MeilinMove : public BT::StatefulActionNode
{
public:
  MeilinMove(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// 被 onStart / onRunning 调用，按当前阶段决定发 action 还是等待
  BT::NodeStatus drive();

  // ---- 微调 Action Client（Motion_control_accurate /move_to_pose）----
  enum class ActionState { IDLE, ACTIVE, DONE, FAILED };
  void sendMoveToPoseGoal(const std::string& name, double x, double y, double yaw_rad);
  bool isActionDone(const std::string& name) const;
  bool isActionActive(const std::string& name) const;
  bool isActionFailed(const std::string& name) const;
  void failActiveMoveToPoseIfTimedOut(const std::string& name);
  bool allActionsDone() const;
  void resetActions();

  ActionState pre_align_state_    = ActionState::IDLE;
  ActionState target_align_state_ = ActionState::IDLE;
  std::string align_message_;
  std::mutex align_mutex_;

  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPoseAction>;

  rclcpp_action::Client<MoveToPoseAction>::SharedPtr align_client_;
  std::shared_ptr<MoveToPoseGoalHandle> align_goal_handle_;
  std::chrono::steady_clock::time_point pre_align_start_time_;
  std::chrono::steady_clock::time_point target_align_start_time_;

  // ---- 悬挂 Action Client（真实异步调用）----
  using SuspensionAction = r2_interfaces::action::SuspensionControl;
  using SuspensionGoalHandle = rclcpp_action::ClientGoalHandle<SuspensionAction>;

  rclcpp_action::Client<SuspensionAction>::SharedPtr suspension_client_;
  std::shared_ptr<SuspensionGoalHandle> suspension_goal_handle_;
  bool suspension_goal_done_ = false;
  BT::NodeStatus suspension_result_ = BT::NodeStatus::FAILURE;
  std::string suspension_message_;
  std::mutex suspension_mutex_;

  /// 发送悬挂 goal（仅在 climb_needed_ 时调用）
  void sendSuspensionGoal();

  // ---- 阶段推进 ----
  enum class Phase { START, WAIT_PHASE1, WAIT_PHASE2, DONE };
  Phase phase_ = Phase::START;
  bool phase1_fired_ = false;
  bool phase2_fired_ = false;

  rclcpp::Node::SharedPtr node_;

  // planner 原始输入
  int move_row_ = 0;
  int move_col_ = 0;
  double target_yaw_ = 0.0;
  double target_height_mm_ = 0.0;

  // 内部计算
  bool pre_align_needed_ = false;
  double pre_align_x_ = 0.0;
  double pre_align_y_ = 0.0;
  double pre_align_yaw_ = 0.0;

  bool climb_needed_ = false;
  int climb_mode_ = 0;
  int climb_direction_ = 0;
  double climb_height_mm_ = 0.0;

  double target_x_ = 0.0;
  double target_y_ = 0.0;

  // 状态暂存
  int from_row_ = 0;
  int from_col_ = 0;
};

}  // namespace r2_bt
