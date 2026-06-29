#pragma once

#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <r2_interfaces/action/step_motion_control.hpp>
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
 *   - omni: 一段 MoveToPose 到抓取位。
 *   - single_axis: 先回当前格中心并转向，再平移到抓取位。
 *   - 可选 StepMotionControl MODE_DIRECT_HEIGHT 调整高度。
 *   - 调用 tool_node arm_grasp。
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
  enum class ActionState { IDLE, ACTIVE, DONE, FAILED };
  enum class Phase {
    START,
    WAIT_CENTER_ALIGN,
    START_FETCH_TRANSLATE,
    WAIT_FETCH_TRANSLATE,
    START_HEIGHT,
    WAIT_HEIGHT,
    START_TOOL,
    WAIT_TOOL,
    DONE
  };

  BT::NodeStatus drive();

  void sendMoveToPoseGoal(double x, double y, double yaw_rad,
                          const std::string& label);
  bool isAlignDone() const;
  bool isAlignFailed() const;
  void failAlignIfTimedOut();

  void sendStepDirectHeightGoal(double target_height_mm);
  bool isHeightDone() const;
  bool isHeightFailed() const;

  void sendToolGrasp();
  void failToolIfTimedOut();

  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPoseAction>;
  using StepMotionAction = r2_interfaces::action::StepMotionControl;
  using StepMotionGoalHandle = rclcpp_action::ClientGoalHandle<StepMotionAction>;
  using ToolActionSrv = r2_interfaces::srv::ToolAction;

  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<MoveToPoseAction>::SharedPtr align_client_;
  std::shared_ptr<MoveToPoseGoalHandle> align_goal_handle_;
  ActionState align_state_ = ActionState::IDLE;
  std::string align_message_;
  std::string align_label_;
  std::mutex align_mutex_;
  std::chrono::steady_clock::time_point align_start_time_;

  rclcpp_action::Client<StepMotionAction>::SharedPtr height_client_;
  std::shared_ptr<StepMotionGoalHandle> height_goal_handle_;
  ActionState height_state_ = ActionState::IDLE;
  std::string height_message_;
  std::mutex height_mutex_;
  bool height_triggered_ = false;

  rclcpp::Client<ToolActionSrv>::SharedPtr tool_client_;
  bool tool_request_sent_ = false;
  bool tool_request_done_ = false;
  BT::NodeStatus tool_result_ = BT::NodeStatus::FAILURE;
  std::string tool_message_;
  std::chrono::steady_clock::time_point tool_start_time_;
  std::mutex tool_mutex_;
  double tool_timeout_sec_ = 30.0;

  Phase phase_ = Phase::START;

  int kfs_row_ = 0;
  int kfs_col_ = 0;
  double height_diff_ = 0.0;
  double target_yaw_ = 0.0;

  int current_row_ = 0;
  int current_col_ = 0;
  double center_x_ = 0.0;
  double center_y_ = 0.0;
  double grasp_x_ = 0.0;
  double grasp_y_ = 0.0;
  bool single_axis_mode_ = true;
  bool height_needed_ = false;

  static constexpr double kFetchLiftHeight = 200.0;
  static constexpr double kHeightTimeoutSec = 10.0;
};

}  // namespace r2_bt
