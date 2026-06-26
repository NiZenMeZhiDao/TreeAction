#pragma once

#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <r2_interfaces/action/arm_action.hpp>
#include <r2_interfaces/action/suspension_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <mutex>
#include <string>

namespace r2_bt
{

/**
 * @brief 梅林区 Fetch 动作节点
 *
 * 执行模型:
 *   onStart: 完成全部计算，立即发 MoveToPose(grasp) + Suspension(如需，并行)
 *   onRunning: 检查完成 → 发 ArmAction(grasp) → 等待完成 → 更新状态
 *   - MoveToPose 与 ArmAction 串行（先到位再抓）
 *   - 悬挂并行: SuspensionControl 与 MoveToPose 同时进行
 *
 * 悬挂调用: 使用 rclcpp_action::Client<SuspensionControl> 异步发送 goal，
 *   参考 r2_hardware SuspensionActionServer（原 active_suspension_control 状态机）。
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

  // ---- 微调 + 机械臂 Action Client ----
  enum class ActionState { IDLE, ACTIVE, DONE, FAILED };
  void sendMoveToPoseGoal();
  void sendArmGoal();
  bool isActionDone(const std::string& name) const;
  bool isActionActive(const std::string& name) const;
  bool isActionFailed(const std::string& name) const;
  void failActiveActionIfTimedOut(const std::string& name);
  bool allActionsDone() const;
  void resetActions();

  ActionState align_state_ = ActionState::IDLE;
  ActionState arm_state_   = ActionState::IDLE;
  std::string align_message_;
  std::string arm_message_;
  std::mutex align_mutex_;
  std::mutex arm_mutex_;

  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPoseAction>;
  using ArmActionT = r2_interfaces::action::ArmAction;
  using ArmGoalHandle = rclcpp_action::ClientGoalHandle<ArmActionT>;

  rclcpp_action::Client<MoveToPoseAction>::SharedPtr align_client_;
  std::shared_ptr<MoveToPoseGoalHandle> align_goal_handle_;
  rclcpp_action::Client<ArmActionT>::SharedPtr arm_client_;
  std::shared_ptr<ArmGoalHandle> arm_goal_handle_;
  std::chrono::steady_clock::time_point align_start_time_;
  std::chrono::steady_clock::time_point arm_start_time_;

  // ---- 悬挂 Action Client（真实异步调用）----
  using SuspensionAction = r2_interfaces::action::SuspensionControl;
  using SuspensionGoalHandle = rclcpp_action::ClientGoalHandle<SuspensionAction>;

  rclcpp_action::Client<SuspensionAction>::SharedPtr suspension_client_;
  std::shared_ptr<SuspensionGoalHandle> suspension_goal_handle_;
  bool suspension_goal_done_ = false;
  BT::NodeStatus suspension_result_ = BT::NodeStatus::FAILURE;
  std::string suspension_message_;
  std::mutex suspension_mutex_;

  /// 发送悬挂 goal（仅在 suspension_mode_ != 0 时调用）
  void sendSuspensionGoal();

  // ---- 阶段 ----
  enum class Phase { START, WAIT_PHASE1, START_ARM, WAIT_ARM, DONE };
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
  double abs_height_diff_ = 0.0;
  int suspension_mode_ = 0;          // 0=无需, 3=MODE_DIRECT（Fetch 高度调整不走传感器状态机）
  int suspension_direction_ = 0;
  double suspension_target_height_ = 30.0;  // MODE_DIRECT 的绝对目标高度 (mm)
};

}  // namespace r2_bt
