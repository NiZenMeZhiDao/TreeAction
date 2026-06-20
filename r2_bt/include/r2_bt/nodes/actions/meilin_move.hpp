#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace r2_bt
{

/**
 * @brief 梅林区 Move 动作节点
 *
 * 执行模型:
 *   onStart: 完成全部计算，立即发出第一批 action（pre_align 和/或 climb 并行）
 *   onRunning: 检查当前 action 是否完成，完成则推进到下一批
 *   - Align 串行: pre_align → target_align
 *   - 悬挂并行: SuspensionControl 与 Align 同时进行
 *
 * 当前版本: 完整计算 + 日志，action 调用处标记 TODO
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

  // ---- 模拟 action 状态（真实实现会替换为 async goal handle）----
  enum class ActionState { IDLE, ACTIVE, DONE };
  void startAction(const std::string& name);
  bool isActionDone(const std::string& name) const;
  bool allActionsDone() const;
  void resetActions();

  ActionState pre_align_state_ = ActionState::IDLE;
  ActionState climb_state_     = ActionState::IDLE;
  ActionState target_align_state_ = ActionState::IDLE;

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
