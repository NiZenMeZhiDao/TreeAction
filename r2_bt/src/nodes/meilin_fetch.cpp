#include "r2_bt/nodes/actions/meilin_fetch.hpp"

#include <cmath>
#include <string>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

// ===========================================================================
// 微调 + 机械臂 Action Client
// ===========================================================================

void MeilinFetch::sendMoveToPoseGoal()
{
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    align_state_ = ActionState::ACTIVE;
    align_message_.clear();
    align_goal_handle_.reset();
    align_start_time_ = std::chrono::steady_clock::now();
  }

  auto fail_goal = [this](const std::string& message) {
    std::lock_guard<std::mutex> lock(align_mutex_);
    align_state_ = ActionState::FAILED;
    align_message_ = message;
  };

  if (!node_)
  {
    fail_goal("Missing ros_node for Motion_control_accurate client");
    return;
  }

  if (!align_client_)
  {
    rclcpp_action::Client<MoveToPoseAction>::SharedPtr shared_client;
    if (config().blackboard->get("move_to_pose_client", shared_client) && shared_client)
    {
      align_client_ = shared_client;
    }
    else
    {
      align_client_ =
          rclcpp_action::create_client<MoveToPoseAction>(node_, "move_to_pose");
    }
  }

  if (!align_client_->action_server_is_ready())
  {
    fail_goal("Motion_control_accurate /move_to_pose action server not available");
    RCLCPP_ERROR(node_->get_logger(), "[MeilinFetch] %s", align_message_.c_str());
    return;
  }

  auto goal = MoveToPoseAction::Goal();
  goal.x = grasp_x_;
  goal.y = grasp_y_;
  goal.yaw_deg = target_yaw_ * 180.0 / M_PI;
  goal.pid_profile = MoveToPoseAction::Goal::PID_PROFILE_SLOW;

  auto send_goal_options = rclcpp_action::Client<MoveToPoseAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const std::shared_ptr<MoveToPoseGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(align_mutex_);
      if (align_state_ != ActionState::ACTIVE)
      {
        return;
      }
      align_goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        align_state_ = ActionState::FAILED;
        align_message_ = "Goal rejected by Motion_control_accurate /move_to_pose";
      }
    };
  send_goal_options.result_callback =
    [this](const MoveToPoseGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(align_mutex_);
      if (align_state_ != ActionState::ACTIVE)
      {
        return;
      }
      if (result.result)
      {
        align_message_ = result.result->message;
      }
      align_state_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                      result.result && result.result->success)
                         ? ActionState::DONE
                         : ActionState::FAILED;
    };

  align_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "[MeilinFetch] >>> /move_to_pose goal(grasp): x=%.3f y=%.3f yaw=%.1f° pid_profile=%u",
              goal.x, goal.y, goal.yaw_deg, goal.pid_profile);
}

void MeilinFetch::sendArmGoal()
{
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    arm_state_ = ActionState::ACTIVE;
    arm_message_.clear();
    arm_goal_handle_.reset();
    arm_start_time_ = std::chrono::steady_clock::now();
  }

  auto fail_goal = [this](const std::string& message) {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    arm_state_ = ActionState::FAILED;
    arm_message_ = message;
  };

  if (!node_)
  {
    fail_goal("Missing ros_node for ArmAction client");
    return;
  }

  if (!arm_client_)
  {
    arm_client_ =
        rclcpp_action::create_client<ArmActionT>(node_, "arm_action");
  }

  if (!arm_client_->action_server_is_ready())
  {
    fail_goal("ArmAction action server not available");
    RCLCPP_ERROR(node_->get_logger(), "[MeilinFetch] %s", arm_message_.c_str());
    return;
  }

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double timeout_sec = cfg ? cfg->arm_timeout_sec : 30.0;

  auto goal = ArmActionT::Goal();
  goal.command = ArmActionT::Goal::CMD_GRASP;
  goal.wait_result = true;
  goal.timeout_sec = static_cast<float>(timeout_sec);

  auto send_goal_options = rclcpp_action::Client<ArmActionT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const std::shared_ptr<ArmGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(arm_mutex_);
      if (arm_state_ != ActionState::ACTIVE)
      {
        return;
      }
      arm_goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        arm_state_ = ActionState::FAILED;
        arm_message_ = "Goal rejected by arm action server";
      }
    };
  send_goal_options.result_callback =
    [this](const ArmGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(arm_mutex_);
      if (arm_state_ != ActionState::ACTIVE)
      {
        return;
      }
      if (result.result)
      {
        arm_message_ = result.result->message;
      }
      arm_state_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                    result.result && result.result->success)
                       ? ActionState::DONE
                       : ActionState::FAILED;
    };

  arm_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "[MeilinFetch] >>> /arm_action goal: CMD_GRASP timeout=%.1fs",
              timeout_sec);
}

bool MeilinFetch::isActionDone(const std::string& name) const
{
  if (name == "align") return align_state_ == ActionState::DONE;
  if (name == "arm")   return arm_state_   == ActionState::DONE;
  return true;
}

bool MeilinFetch::isActionActive(const std::string& name) const
{
  if (name == "align") return align_state_ == ActionState::ACTIVE;
  if (name == "arm")   return arm_state_   == ActionState::ACTIVE;
  return false;
}

bool MeilinFetch::isActionFailed(const std::string& name) const
{
  if (name == "align") return align_state_ == ActionState::FAILED;
  if (name == "arm")   return arm_state_   == ActionState::FAILED;
  return false;
}

void MeilinFetch::failActiveActionIfTimedOut(const std::string& name)
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double timeout_sec =
      name == "align"
          ? (cfg ? cfg->align_timeout_sec : 30.0)
          : (cfg ? cfg->arm_timeout_sec : 30.0);
  if (timeout_sec <= 0.0)
  {
    return;
  }

  const auto start_time =
      (name == "align") ? align_start_time_ : arm_start_time_;
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();

  if (elapsed <= timeout_sec)
  {
    return;
  }

  if (name == "align")
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    if (align_state_ != ActionState::ACTIVE)
    {
      return;
    }
    if (align_client_ && align_goal_handle_)
    {
      align_client_->async_cancel_goal(align_goal_handle_);
    }
    align_state_ = ActionState::FAILED;
    align_message_ = "Motion_control_accurate /move_to_pose timed out";
  }
  else
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    if (arm_state_ != ActionState::ACTIVE)
    {
      return;
    }
    if (arm_client_ && arm_goal_handle_)
    {
      arm_client_->async_cancel_goal(arm_goal_handle_);
    }
    arm_state_ = ActionState::FAILED;
    arm_message_ = "ArmAction timed out";
  }
}

bool MeilinFetch::allActionsDone() const
{
  if (align_state_ == ActionState::ACTIVE) return false;
  if (arm_state_   == ActionState::ACTIVE) return false;
  // 检查悬挂 action client 是否已完成
  if (suspension_mode_ != 0 && !suspension_goal_done_) return false;
  return true;
}

void MeilinFetch::resetActions()
{
  align_state_ = ActionState::IDLE;
  arm_state_   = ActionState::IDLE;
  align_message_.clear();
  arm_message_.clear();
  align_goal_handle_.reset();
  arm_goal_handle_.reset();
  // 悬挂状态在 onStart 中初始化
}

// ===========================================================================
// 悬挂 Action Client（真实异步调用，参考 suspension_control.cpp 和
// active_suspension_control 原始状态机）
// ===========================================================================

void MeilinFetch::sendSuspensionGoal()
{
  {
    std::lock_guard<std::mutex> lock(suspension_mutex_);
    suspension_goal_done_ = false;
    suspension_result_    = BT::NodeStatus::FAILURE;
    suspension_message_.clear();
    suspension_goal_handle_.reset();
  }

  if (!node_)
  {
    suspension_goal_done_ = true;
    suspension_result_    = BT::NodeStatus::FAILURE;
    suspension_message_   = "Missing ros_node for SuspensionControl client";
    RCLCPP_ERROR(rclcpp::get_logger("MeilinFetch"), "%s", suspension_message_.c_str());
    return;
  }

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double timeout_sec = cfg ? cfg->suspension_timeout_sec : 10.0;

  // 首次调用时创建 action client
  if (!suspension_client_)
  {
    suspension_client_ =
        rclcpp_action::create_client<SuspensionAction>(node_, "suspension_control");
  }

  if (!suspension_client_->action_server_is_ready())
  {
    suspension_goal_done_ = true;
    suspension_result_    = BT::NodeStatus::FAILURE;
    suspension_message_   = "SuspensionControl action server not available";
    RCLCPP_ERROR(node_->get_logger(), "[MeilinFetch] %s", suspension_message_.c_str());
    return;
  }

  auto goal = SuspensionAction::Goal();
  goal.mode        = static_cast<uint8_t>(suspension_mode_);
  goal.direction   = static_cast<uint8_t>(suspension_direction_);
  // MODE_DIRECT 时 height 是四轮绝对目标高度（正常高度 + 高度差）
  goal.height      = static_cast<float>(suspension_target_height_);
  goal.timeout_sec = static_cast<float>(timeout_sec);

  auto send_goal_options = rclcpp_action::Client<SuspensionAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const std::shared_ptr<SuspensionGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(suspension_mutex_);
      suspension_goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        suspension_goal_done_ = true;
        suspension_result_    = BT::NodeStatus::FAILURE;
        suspension_message_   = "Goal rejected by suspension action server";
      }
    };
  send_goal_options.result_callback =
    [this](const SuspensionGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(suspension_mutex_);
      suspension_goal_done_ = true;
      if (result.result)
      {
        suspension_message_ = result.result->message;
      }
      suspension_result_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                            result.result && result.result->success)
                               ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
    };

  suspension_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "[MeilinFetch] >>> Suspension goal sent: mode=%d dir=%d height=%.1f timeout=%.1f",
              goal.mode, goal.direction, goal.height, goal.timeout_sec);
}

// ===========================================================================
// BT Node 接口
// ===========================================================================

MeilinFetch::MeilinFetch(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList MeilinFetch::providedPorts()
{
  return {
    BT::InputPort<int>("kfs_row"),
    BT::InputPort<int>("kfs_col"),
    BT::InputPort<double>("height_diff"),
    BT::InputPort<double>("target_yaw"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus MeilinFetch::onStart()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  if (!node_)
  {
    setOutput("message", std::string{"Missing ros_node on blackboard"});
    return BT::NodeStatus::FAILURE;
  }

  // =========================================================================
  // 1. 读取输入
  // =========================================================================
  const auto kfs_row = getInput<int>("kfs_row");
  const auto kfs_col = getInput<int>("kfs_col");
  const auto target_yaw = getInput<double>("target_yaw");
  if (!kfs_row || !kfs_col || !target_yaw)
  {
    const std::string err = "Fetch missing kfs_row/kfs_col/target_yaw";
    setOutput("message", err);
    config().blackboard->set("last_error", err);
    return BT::NodeStatus::FAILURE;
  }
  kfs_row_ = kfs_row.value();
  kfs_col_ = kfs_col.value();
  target_yaw_ = target_yaw.value();
  height_diff_ = getInput<double>("height_diff").value_or(0.0);

  // =========================================================================
  // 2. 读取配置和当前状态
  // =========================================================================
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  if (!cfg)
  {
    setOutput("message", std::string{"Missing meilin_config on blackboard"});
    return BT::NodeStatus::FAILURE;
  }

  const int current_row = config().blackboard->get<int>("meilin_current_row");
  const int current_col = config().blackboard->get<int>("meilin_current_col");

  RCLCPP_INFO(node_->get_logger(),
              "[Fetch] from (%d,%d) → KFS(%d,%d)  h_diff=%.0fmm  yaw=%.2f",
              current_row, current_col, kfs_row_, kfs_col_, height_diff_, target_yaw_);

  // =========================================================================
  // 3. 计算抓取位姿
  // =========================================================================
  {
    double actual_yaw = 0.0;
    meilin_calculate_grasp_position(kfs_row_, kfs_col_, target_yaw_, *cfg,
                                    current_row, current_col,
                                    grasp_x_, grasp_y_, actual_yaw);
    target_yaw_ = actual_yaw;
  }
  RCLCPP_INFO(node_->get_logger(),
              "[Fetch] grasp pose: (%.3f,%.3f) yaw=%.1f°  (grid/2+%.2fm from KFS)",
              grasp_x_, grasp_y_, target_yaw_ * 180.0 / M_PI, cfg->grasp_distance);

  // =========================================================================
  // 4. 悬挂高度调整（MODE_DIRECT 快速模式）
  //    Fetch 的高度调整不是真正的上下台阶，不需要走传感器引导的状态机。
  //    使用 MODE_DIRECT 直接设置四轮目标高度，完成后不自动恢复到 H_INIT，
  //    而是通过 meilin_suspension_offset 记录偏移量，留给后续 Move 节点处理，
  //    避免不必要的"降回 H_INIT → 再升高"绕路，节约时间。
  // =========================================================================
  abs_height_diff_ = std::abs(height_diff_);
  if (abs_height_diff_ > cfg->height_tolerance)
  {
    suspension_mode_ = 3;  // MODE_DIRECT: 直接设置四轮统一高度
    suspension_direction_ = 0;  // MODE_DIRECT 不使用 direction

    // MODE_DIRECT 的 height 是四轮绝对目标高度 = 正常行驶高度 + 高度差
    suspension_target_height_ = cfg->suspension_normal_height + height_diff_;

    // 仍计算方向用于日志
    double unused_yaw = 0.0;
    std::string dir_name;
    int unused_dir = 0;
    meilin_direction_yaw(current_row, current_col, kfs_row_, kfs_col_,
                         unused_yaw, unused_dir, dir_name, target_yaw_);

    RCLCPP_INFO(node_->get_logger(),
                "[Fetch] suspension: MODE_DIRECT height=%.0fmm (normal %.0f + diff %.0f)  approach=%s",
                suspension_target_height_, cfg->suspension_normal_height, height_diff_,
                dir_name.c_str());
  }
  else
  {
    suspension_mode_ = 0;
    suspension_direction_ = 0;
    suspension_target_height_ = cfg->suspension_normal_height;
    RCLCPP_INFO(node_->get_logger(), "[Fetch] no suspension needed");
  }

  // =========================================================================
  // 5. onStart 直接发出第一批 action: MoveToPose(grasp) + Suspension(并行)
  // =========================================================================
  resetActions();
  {
    std::lock_guard<std::mutex> lock(suspension_mutex_);
    suspension_goal_done_ = false;
    suspension_result_    = BT::NodeStatus::FAILURE;
    suspension_message_.clear();
    suspension_goal_handle_.reset();
  }
  phase_ = Phase::START;

  setOutput("message", std::string{});
  config().blackboard->set("active_action", std::string{"Fetch"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});

  return drive();
}

BT::NodeStatus MeilinFetch::onRunning()
{
  return drive();
}

void MeilinFetch::onHalted()
{
  if (node_)
    RCLCPP_WARN(node_->get_logger(), "[Fetch] Halted");
  // 取消进行中的悬挂 goal
  {
    std::lock_guard<std::mutex> lock(suspension_mutex_);
    if (suspension_client_ && suspension_goal_handle_)
    {
      suspension_client_->async_cancel_goal(suspension_goal_handle_);
      RCLCPP_INFO(node_->get_logger(), "[Fetch] Suspension goal cancelled");
    }
    suspension_goal_done_ = true;
    suspension_result_    = BT::NodeStatus::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    if (align_client_ && align_goal_handle_)
    {
      align_client_->async_cancel_goal(align_goal_handle_);
      if (node_) RCLCPP_INFO(node_->get_logger(), "[Fetch] /move_to_pose goal cancelled");
    }
    align_state_ = ActionState::FAILED;
  }
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    if (arm_client_ && arm_goal_handle_)
    {
      arm_client_->async_cancel_goal(arm_goal_handle_);
      if (node_) RCLCPP_INFO(node_->get_logger(), "[Fetch] /arm_action goal cancelled");
    }
    arm_state_ = ActionState::FAILED;
  }
}

// ===========================================================================
// 核心状态机
// ===========================================================================

BT::NodeStatus MeilinFetch::drive()
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");

  switch (phase_)
  {
    // =======================================================================
    // Phase::START — 并行发出 MoveToPose(grasp) + Suspension
    // =======================================================================
    case Phase::START:
    {
      phase_ = Phase::WAIT_PHASE1;

      {
        const double t = cfg ? cfg->align_timeout_sec : 30.0;
        RCLCPP_INFO(node_->get_logger(),
                    "[Fetch] >>> START MoveToPose(grasp): (%.3f,%.3f) yaw=%.1f° timeout=%.1fs",
                    grasp_x_, grasp_y_, target_yaw_ * 180.0 / M_PI, t);
        sendMoveToPoseGoal();
      }

      if (suspension_mode_ != 0)
      {
        sendSuspensionGoal();
      }

      return BT::NodeStatus::RUNNING;
    }

    // =======================================================================
    // Phase::WAIT_PHASE1 — 等 MoveToPose 和 Suspension 都完成
    // =======================================================================
    case Phase::WAIT_PHASE1:
    {
      failActiveActionIfTimedOut("align");
      if (!allActionsDone())
        return BT::NodeStatus::RUNNING;

      if (isActionFailed("align"))
      {
        setOutput("message", align_message_);
        config().blackboard->set("last_error", align_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(),
                     "[Fetch] align failed: %s", align_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // 检查悬挂结果
      if (suspension_mode_ != 0)
      {
        std::lock_guard<std::mutex> lock(suspension_mutex_);
        if (suspension_result_ != BT::NodeStatus::SUCCESS)
        {
          setOutput("message", suspension_message_);
          config().blackboard->set("last_error", suspension_message_);
          config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
          RCLCPP_ERROR(node_->get_logger(),
                       "[Fetch] Suspension failed: %s", suspension_message_.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }

      // 两者都完成，进入 Arm 阶段
      phase_ = Phase::START_ARM;
      // fall through
    }
    [[fallthrough]];

    // =======================================================================
    // Phase::START_ARM — 串行: MoveToPose 完成后才发 ArmAction（不能边动边抓）
    // =======================================================================
    case Phase::START_ARM:
    {
      phase_ = Phase::WAIT_ARM;

      {
        const double t = cfg ? cfg->arm_timeout_sec : 30.0;
        RCLCPP_INFO(node_->get_logger(),
                    "[Fetch] >>> START ArmAction(GRASP) timeout=%.1fs", t);
        sendArmGoal();
      }

      return BT::NodeStatus::RUNNING;
    }

    // =======================================================================
    // Phase::WAIT_ARM — 等 ArmAction 完成
    // =======================================================================
    case Phase::WAIT_ARM:
    {
      failActiveActionIfTimedOut("arm");
      if (isActionActive("arm"))
        return BT::NodeStatus::RUNNING;

      if (isActionFailed("arm"))
      {
        setOutput("message", arm_message_);
        config().blackboard->set("last_error", arm_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(),
                     "[Fetch] arm failed: %s", arm_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      phase_ = Phase::DONE;
      // fall through
    }
    [[fallthrough]];

    // =======================================================================
    // Phase::DONE
    // =======================================================================
    case Phase::DONE:
    {
      config().blackboard->set("meilin_current_yaw", target_yaw_);
      config().blackboard->set("meilin_pose_is_cell_center", false);
      // 记录悬挂偏移量: 如果此次 fetch 调整了高度，偏移量保留给后续 Move
      // 避免 Move 再降回 H_INIT → 重新升高（绕路）
      if (suspension_mode_ != 0)
      {
        config().blackboard->set("meilin_suspension_offset", height_diff_);
        RCLCPP_INFO(node_->get_logger(),
                    "[Fetch] suspension offset kept: %.0fmm (will be consumed by next Move)",
                    height_diff_);
      }
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      setOutput("message", std::string{"Fetch done"});

      RCLCPP_INFO(node_->get_logger(),
                  "[Fetch] DONE → blackboard: yaw=%.2f  pose_is_center=false",
                  target_yaw_);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace r2_bt
