#pragma once

#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <r2_interfaces/action/step_motion_control.hpp>
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
 *   - 新计划第一个 move 作为入口点，只调用 /move_to_pose。
 *   - 后续 move 若存在高度变化，先调用 /step_motion_control，再调用 /move_to_pose。
 *   - 同格转向和平高度移动只调用 /move_to_pose。
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
  enum class ActionState { IDLE, ACTIVE, DONE, FAILED };
  enum class Phase { START, WAIT_STEP, START_MOVE, WAIT_MOVE, DONE };

  BT::NodeStatus drive();
  void resetActions();

  void sendMoveToPoseGoal(double x, double y, double yaw_rad);
  void failMoveToPoseIfTimedOut();
  bool moveDone() const;
  bool moveFailed() const;

  void sendStepMotionGoal();
  bool stepDone() const;
  bool stepFailed() const;

  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPoseAction>;
  using StepMotionAction = r2_interfaces::action::StepMotionControl;
  using StepMotionGoalHandle = rclcpp_action::ClientGoalHandle<StepMotionAction>;

  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<MoveToPoseAction>::SharedPtr move_client_;
  std::shared_ptr<MoveToPoseGoalHandle> move_goal_handle_;
  ActionState move_state_ = ActionState::IDLE;
  std::string move_message_;
  std::mutex move_mutex_;
  std::chrono::steady_clock::time_point move_start_time_;

  rclcpp_action::Client<StepMotionAction>::SharedPtr step_client_;
  std::shared_ptr<StepMotionGoalHandle> step_goal_handle_;
  ActionState step_state_ = ActionState::IDLE;
  std::string step_message_;
  std::mutex step_mutex_;

  Phase phase_ = Phase::START;

  int move_row_ = 0;
  int move_col_ = 0;
  double target_yaw_ = 0.0;
  double target_height_mm_ = 0.0;

  int from_row_ = 0;
  int from_col_ = 0;
  double current_height_mm_ = 0.0;
  double current_yaw_ = 0.0;
  bool pose_is_center_ = true;
  bool pose_fresh_ = false;
  double pose_x_ = 0.0;
  double pose_y_ = 0.0;
  double pose_yaw_ = 0.0;

  bool entry_move_ = false;
  bool step_needed_ = false;
  uint8_t step_mode_ = 0;
  uint8_t step_direction_ = 0;
  double step_height_mm_ = 0.0;
  double step_correction_x_ = 0.0;
  double step_correction_y_ = 0.0;
  double step_correction_yaw_deg_ = 0.0;

  double target_x_ = 0.0;
  double target_y_ = 0.0;
};

}  // namespace r2_bt
