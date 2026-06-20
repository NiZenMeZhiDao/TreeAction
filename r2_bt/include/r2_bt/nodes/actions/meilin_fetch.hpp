#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace r2_bt
{

/**
 * @brief 梅林区 Fetch 动作节点
 *
 * 执行模型:
 *   onStart: 完成全部计算，立即发 Align(grasp) + Suspension(如需，并行)
 *   onRunning: 检查完成 → 发 ArmAction(grasp) → 等待完成 → 更新状态
 *   - Align 与 ArmAction 串行（先到位再抓）
 *   - 悬挂并行: SuspensionControl 与 Align 同时进行
 *
 * 当前版本: 完整计算 + 日志，action 调用处标记 TODO
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

  // ---- 模拟 action 状态 ----
  enum class ActionState { IDLE, ACTIVE, DONE };
  void startAction(const std::string& name);
  bool isActionDone(const std::string& name) const;
  bool allActionsDone() const;
  void resetActions();

  ActionState align_state_       = ActionState::IDLE;
  ActionState suspension_state_  = ActionState::IDLE;
  ActionState arm_state_         = ActionState::IDLE;

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
  int suspension_mode_ = 0;
};

}  // namespace r2_bt
