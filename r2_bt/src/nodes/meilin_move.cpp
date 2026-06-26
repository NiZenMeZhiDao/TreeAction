#include "r2_bt/nodes/actions/meilin_move.hpp"

#include <cmath>
#include <string>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

// ===========================================================================
// 微调 Action Client（Motion_control_accurate /move_to_pose）
// ===========================================================================

void MeilinMove::sendMoveToPoseGoal(const std::string& name,
                               double x,
                               double y,
                               double yaw_rad)
{
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    if (name == "pre_align")
    {
      pre_align_state_ = ActionState::ACTIVE;
      pre_align_start_time_ = std::chrono::steady_clock::now();
    }
    else
    {
      target_align_state_ = ActionState::ACTIVE;
      target_align_start_time_ = std::chrono::steady_clock::now();
    }
    align_message_.clear();
    align_goal_handle_.reset();
  }

  auto fail_goal = [this, &name](const std::string& message) {
    std::lock_guard<std::mutex> lock(align_mutex_);
    if (name == "pre_align") pre_align_state_ = ActionState::FAILED;
    else target_align_state_ = ActionState::FAILED;
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
    RCLCPP_ERROR(node_->get_logger(), "[MeilinMove] %s", align_message_.c_str());
    return;
  }

  auto goal = MoveToPoseAction::Goal();
  goal.x = x;
  goal.y = y;
  goal.yaw_deg = yaw_rad * 180.0 / M_PI;
  goal.pid_profile = MoveToPoseAction::Goal::PID_PROFILE_SLOW;

  auto send_goal_options = rclcpp_action::Client<MoveToPoseAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, name](const std::shared_ptr<MoveToPoseGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(align_mutex_);
      auto& state = (name == "pre_align") ? pre_align_state_ : target_align_state_;
      if (state != ActionState::ACTIVE)
      {
        return;
      }
      align_goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        state = ActionState::FAILED;
        align_message_ = "Goal rejected by Motion_control_accurate /move_to_pose";
      }
    };
  send_goal_options.result_callback =
    [this, name](const MoveToPoseGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(align_mutex_);
      auto& state = (name == "pre_align") ? pre_align_state_ : target_align_state_;
      if (state != ActionState::ACTIVE)
      {
        return;
      }
      if (result.result)
      {
        align_message_ = result.result->message;
      }
      state = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
               result.result && result.result->success)
                  ? ActionState::DONE
                  : ActionState::FAILED;
    };

  align_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "[MeilinMove] >>> /move_to_pose goal(%s): x=%.3f y=%.3f yaw=%.1f° pid_profile=%u",
              name.c_str(), goal.x, goal.y, goal.yaw_deg, goal.pid_profile);
}

bool MeilinMove::isActionDone(const std::string& name) const
{
  if (name == "pre_align")    return pre_align_state_ == ActionState::DONE;
  if (name == "target_align") return target_align_state_ == ActionState::DONE;
  return true;
}

bool MeilinMove::isActionActive(const std::string& name) const
{
  if (name == "pre_align")    return pre_align_state_ == ActionState::ACTIVE;
  if (name == "target_align") return target_align_state_ == ActionState::ACTIVE;
  return false;
}

bool MeilinMove::isActionFailed(const std::string& name) const
{
  if (name == "pre_align")    return pre_align_state_ == ActionState::FAILED;
  if (name == "target_align") return target_align_state_ == ActionState::FAILED;
  return false;
}

void MeilinMove::failActiveMoveToPoseIfTimedOut(const std::string& name)
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double timeout_sec = cfg ? cfg->align_timeout_sec : 30.0;
  if (timeout_sec <= 0.0)
  {
    return;
  }

  const auto start_time =
      (name == "pre_align") ? pre_align_start_time_ : target_align_start_time_;
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();

  if (elapsed <= timeout_sec)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(align_mutex_);
  auto& state = (name == "pre_align") ? pre_align_state_ : target_align_state_;
  if (state != ActionState::ACTIVE)
  {
    return;
  }
  if (align_client_ && align_goal_handle_)
  {
    align_client_->async_cancel_goal(align_goal_handle_);
  }
  state = ActionState::FAILED;
  align_message_ = "Motion_control_accurate /move_to_pose timed out";
}

bool MeilinMove::allActionsDone() const
{
  // 检查所有已触发的 action 是否都已完成
  if (pre_align_state_    == ActionState::ACTIVE) return false;
  if (target_align_state_ == ActionState::ACTIVE) return false;
  // 检查悬挂 action client 是否已完成
  if (climb_needed_ && !suspension_goal_done_)   return false;
  return true;
}

void MeilinMove::resetActions()
{
  pre_align_state_    = ActionState::IDLE;
  target_align_state_ = ActionState::IDLE;
  align_message_.clear();
  align_goal_handle_.reset();
  // 悬挂状态不需要在这里 reset（在 onStart 中统一初始化）
}

// ===========================================================================
// 悬挂 Action Client（真实异步调用，参考 suspension_control.cpp 和
// active_suspension_control 原始状态机）
// ===========================================================================

void MeilinMove::sendSuspensionGoal()
{
  {
    std::lock_guard<std::mutex> lock(suspension_mutex_);
    suspension_goal_done_ = false;
    suspension_result_   = BT::NodeStatus::FAILURE;
    suspension_message_.clear();
    suspension_goal_handle_.reset();
  }

  if (!node_)
  {
    suspension_goal_done_ = true;
    suspension_result_    = BT::NodeStatus::FAILURE;
    suspension_message_   = "Missing ros_node for SuspensionControl client";
    RCLCPP_ERROR(rclcpp::get_logger("MeilinMove"), "%s", suspension_message_.c_str());
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
    RCLCPP_ERROR(node_->get_logger(), "[MeilinMove] %s", suspension_message_.c_str());
    return;
  }

  auto goal = SuspensionAction::Goal();
  goal.mode       = static_cast<uint8_t>(climb_mode_);
  goal.direction  = static_cast<uint8_t>(climb_direction_);
  goal.height     = static_cast<float>(climb_height_mm_);
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
              "[MeilinMove] >>> Suspension goal sent: mode=%d dir=%d height=%.1f timeout=%.1f",
              goal.mode, goal.direction, goal.height, goal.timeout_sec);
}

// ===========================================================================
// BT Node 接口
// ===========================================================================

MeilinMove::MeilinMove(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList MeilinMove::providedPorts()
{
  return {
    BT::InputPort<int>("move_row"),
    BT::InputPort<int>("move_col"),
    BT::InputPort<double>("target_height_mm"),
    BT::InputPort<double>("target_yaw"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus MeilinMove::onStart()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
  if (!node_)
  {
    setOutput("message", std::string{"Missing ros_node on blackboard"});
    return BT::NodeStatus::FAILURE;
  }

  // =========================================================================
  // 1. 读取 planner 原始输入
  // =========================================================================
  const auto move_row = getInput<int>("move_row");
  const auto move_col = getInput<int>("move_col");
  const auto target_yaw = getInput<double>("target_yaw");
  if (!move_row || !move_col || !target_yaw)
  {
    const std::string err = "Move missing move_row/move_col/target_yaw";
    setOutput("message", err);
    config().blackboard->set("last_error", err);
    return BT::NodeStatus::FAILURE;
  }
  move_row_ = move_row.value();
  move_col_ = move_col.value();
  target_yaw_ = target_yaw.value();
  target_height_mm_ = getInput<double>("target_height_mm").value_or(0.0);

  // =========================================================================
  // 2. 读取配置
  // =========================================================================
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  if (!cfg)
  {
    setOutput("message", std::string{"Missing meilin_config on blackboard"});
    return BT::NodeStatus::FAILURE;
  }

  // =========================================================================
  // 3. 读取当前状态
  //    优先使用 /transformed/pose 缓存的 map 系 base_link 位姿；
  //    定位未接入或超时时，降级使用 blackboard 中的模拟状态。
  // =========================================================================
  from_row_    = config().blackboard->get<int>("meilin_current_row");
  from_col_    = config().blackboard->get<int>("meilin_current_col");
  double current_height =
      config().blackboard->get<double>("meilin_current_height");
  double current_yaw =
      config().blackboard->get<double>("meilin_current_yaw");
  bool pose_is_center =
      config().blackboard->get<bool>("meilin_pose_is_cell_center");
  // 读取悬挂偏移量（由上一个 Fetch 的 MODE_DIRECT 留存）
  const double suspension_offset =
      config().blackboard->get<double>("meilin_suspension_offset");

  const bool pose_received = config().blackboard->get<bool>("meilin_pose_received");
  if (pose_received)
  {
    const double last_update_sec =
        config().blackboard->get<double>("meilin_pose_last_update_sec");
    const double pose_age_sec = node_->now().seconds() - last_update_sec;
    const bool pose_fresh =
        cfg->pose_timeout_sec <= 0.0 || pose_age_sec <= cfg->pose_timeout_sec;

    if (pose_fresh)
    {
      const double pose_x = config().blackboard->get<double>("meilin_pose_x");
      const double pose_y = config().blackboard->get<double>("meilin_pose_y");
      const double pose_yaw = config().blackboard->get<double>("meilin_pose_yaw");

      int pose_row = 0;
      int pose_col = 0;
      if (!meilin_world_to_grid(pose_x, pose_y, *cfg, pose_row, pose_col))
      {
        const std::string err = "Current /transformed/pose is outside Meilin grid";
        setOutput("message", err);
        config().blackboard->set("last_error", err);
        RCLCPP_ERROR(node_->get_logger(),
                     "[Move] %s: x=%.3f y=%.3f", err.c_str(), pose_x, pose_y);
        return BT::NodeStatus::FAILURE;
      }

      const auto [center_x, center_y] = meilin_grid_to_world(pose_row, pose_col, *cfg);
      const double center_distance = std::hypot(pose_x - center_x, pose_y - center_y);
      from_row_ = pose_row;
      from_col_ = pose_col;
      current_height = meilin_height_at(from_row_, from_col_, cfg->side);
      current_yaw = pose_yaw;
      pose_is_center = center_distance <= cfg->cell_center_tolerance;

      config().blackboard->set("meilin_current_row", from_row_);
      config().blackboard->set("meilin_current_col", from_col_);
      config().blackboard->set("meilin_current_height", current_height);
      config().blackboard->set("meilin_current_yaw", current_yaw);
      config().blackboard->set("meilin_pose_is_cell_center", pose_is_center);

      RCLCPP_INFO(node_->get_logger(),
                  "[Move] localization: /transformed/pose x=%.3f y=%.3f yaw=%.2f "
                  "→ grid=(%d,%d), center_dist=%.3f",
                  pose_x, pose_y, current_yaw, from_row_, from_col_, center_distance);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[Move] /transformed/pose stale (age=%.2fs > %.2fs), using blackboard state",
                  pose_age_sec, cfg->pose_timeout_sec);
    }
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(),
                "[Move] /transformed/pose not received yet, using blackboard state");
  }

  RCLCPP_INFO(node_->get_logger(),
              "[Move] from (%d,%d) h=%.0f → to (%d,%d) h=%.0f  yaw=%.2f  center=%s  suspension_offset=%.0f",
              from_row_, from_col_, current_height,
              move_row_, move_col_, target_height_mm_, target_yaw_,
              pose_is_center ? "yes" : "no",
              suspension_offset);

  // =========================================================================
  // 4. grid → world
  // =========================================================================
  {
    const auto [wx, wy] = meilin_grid_to_world(move_row_, move_col_, *cfg);
    target_x_ = wx;
    target_y_ = wy;
  }

  // =========================================================================
  // 5. 爬台判断 → onStart 时直接发
  //    若存在悬挂偏移（上次 Fetch 存留），计入实际高度差中，
  //    避免不必要的"降回 H_INIT → 再升高"绕路。
  //    CLIMB_UP/DOWN 的 height 参数是台阶高度，状态机会从当前轮高出发，
  //    不会重复运动已在目标高度的轮子。
  // =========================================================================
  const double height_diff = target_height_mm_ - current_height;
  climb_needed_ = std::abs(height_diff) > cfg->height_tolerance;
  if (climb_needed_)
  {
    climb_mode_      = (height_diff > 0.0) ? 1 : 2;
    climb_height_mm_ = std::abs(height_diff);
    double unused_yaw = 0.0;
    std::string dir_name;
    meilin_direction_yaw(from_row_, from_col_, move_row_, move_col_,
                         unused_yaw, climb_direction_, dir_name);
    if (std::abs(suspension_offset) > 0.0)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "[Move] climb: mode=%s dir=%s %.0fmm (offset=%.0fmm from prev Fetch, "
                  "state machine starts from current wheel height)",
                  climb_mode_ == 1 ? "UP" : "DOWN",
                  dir_name.c_str(), climb_height_mm_, suspension_offset);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(),
                  "[Move] climb: mode=%s dir=%s %.0fmm",
                  climb_mode_ == 1 ? "UP" : "DOWN",
                  dir_name.c_str(), climb_height_mm_);
    }
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(),
                "[Move] no climb (diff=%.0f ≤ tol=%.0f)", height_diff, cfg->height_tolerance);
    if (std::abs(suspension_offset) > cfg->height_tolerance)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[Move] suspension offset %.0fmm present but no climb needed — "
                  "offset will be reset, suspension returns to normal height",
                  suspension_offset);
    }
  }

  // =========================================================================
  // 6. pre_align 判断
  //    TODO: 优化 — 如果回中会绕路（当前偏移方向与目标方向一致），可跳过
  // =========================================================================
  pre_align_needed_ = !pose_is_center;
  if (pre_align_needed_)
  {
    const auto [cx, cy] = meilin_grid_to_world(from_row_, from_col_, *cfg);
    pre_align_x_   = cx;
    pre_align_y_   = cy;
    pre_align_yaw_ = current_yaw;
    RCLCPP_INFO(node_->get_logger(),
                "[Move] pre_align: back to (%.3f,%.3f) yaw=%.2f", cx, cy, current_yaw);
  }

  // =========================================================================
  // 7. onStart 直接发出第一批 action（pre_align 与 climb 并行）
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
  phase1_fired_ = false;
  phase2_fired_ = false;

  setOutput("message", std::string{});
  config().blackboard->set("active_action", std::string{"Move"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});

  return drive();
}

BT::NodeStatus MeilinMove::onRunning()
{
  return drive();
}

void MeilinMove::onHalted()
{
  if (node_)
    RCLCPP_WARN(node_->get_logger(), "[Move] Halted");
  // 取消进行中的悬挂 goal
  {
    std::lock_guard<std::mutex> lock(suspension_mutex_);
    if (suspension_client_ && suspension_goal_handle_)
    {
      suspension_client_->async_cancel_goal(suspension_goal_handle_);
      RCLCPP_INFO(node_->get_logger(), "[Move] Suspension goal cancelled");
    }
    suspension_goal_done_ = true;
    suspension_result_    = BT::NodeStatus::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    if (align_client_ && align_goal_handle_)
    {
      align_client_->async_cancel_goal(align_goal_handle_);
      RCLCPP_INFO(node_->get_logger(), "[Move] /move_to_pose goal cancelled");
    }
    pre_align_state_ = ActionState::FAILED;
    target_align_state_ = ActionState::FAILED;
  }
}

// ===========================================================================
// 核心状态机
// ===========================================================================

BT::NodeStatus MeilinMove::drive()
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");

  switch (phase_)
  {
    // =======================================================================
    // Phase::START — 并行发出 pre_align + climb(悬挂)
    // =======================================================================
    case Phase::START:
    {
      phase_ = Phase::WAIT_PHASE1;

      if (pre_align_needed_)
      {
        RCLCPP_INFO(node_->get_logger(),
                    "[Move] >>> START MoveToPose(pre_align): (%.3f,%.3f) yaw=%.1f°",
                    pre_align_x_, pre_align_y_, pre_align_yaw_ * 180.0 / M_PI);
        sendMoveToPoseGoal("pre_align", pre_align_x_, pre_align_y_, pre_align_yaw_);
      }

      if (climb_needed_)
      {
        sendSuspensionGoal();
      }

      if (!pre_align_needed_ && !climb_needed_)
      {
        // 什么都不需要，直接跳到 Phase2
        phase_ = Phase::WAIT_PHASE2;
        // fall through to WAIT_PHASE2
      }
      else
      {
        phase1_fired_ = true;
        return BT::NodeStatus::RUNNING;
      }
    }
    [[fallthrough]];

    // =======================================================================
    // Phase::WAIT_PHASE1 — 等 pre_align 完成（climb 可能还在跑）
    // =======================================================================
    case Phase::WAIT_PHASE1:
    {
      // 检查 pre_align 是否完成（climb 不阻塞，继续并行）
      if (pre_align_needed_)
      {
        failActiveMoveToPoseIfTimedOut("pre_align");
        if (isActionActive("pre_align"))
        {
          return BT::NodeStatus::RUNNING;
        }
        if (isActionFailed("pre_align"))
        {
          setOutput("message", align_message_);
          config().blackboard->set("last_error", align_message_);
          config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
          RCLCPP_ERROR(node_->get_logger(),
                       "[Move] pre_align failed: %s", align_message_.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }

      // 检查悬挂 goal 是否失败（提前退出）
      {
        std::lock_guard<std::mutex> lock(suspension_mutex_);
        if (climb_needed_ && suspension_goal_done_ &&
            suspension_result_ == BT::NodeStatus::FAILURE)
        {
          setOutput("message", suspension_message_);
          config().blackboard->set("last_error", suspension_message_);
          config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
          RCLCPP_ERROR(node_->get_logger(),
                       "[Move] Suspension failed early: %s", suspension_message_.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }

      // pre_align 完成（或不需要），进入 Phase2: 发 target_align
      // climb 如果还在进行中会继续并行
      phase_ = Phase::WAIT_PHASE2;

      {
        const double t = cfg ? cfg->align_timeout_sec : 30.0;
        RCLCPP_INFO(node_->get_logger(),
                    "[Move] >>> START MoveToPose(target): (%.3f,%.3f) yaw=%.1f° timeout=%.1fs",
                    target_x_, target_y_, target_yaw_ * 180.0 / M_PI, t);
        sendMoveToPoseGoal("target_align", target_x_, target_y_, target_yaw_);
      }
      phase2_fired_ = true;
      return BT::NodeStatus::RUNNING;
    }

    // =======================================================================
    // Phase::WAIT_PHASE2 — 等 target_align 和 climb 都完成
    // =======================================================================
    case Phase::WAIT_PHASE2:
    {
      failActiveMoveToPoseIfTimedOut("target_align");
      if (!allActionsDone())
        return BT::NodeStatus::RUNNING;

      if (isActionFailed("target_align"))
      {
        setOutput("message", align_message_);
        config().blackboard->set("last_error", align_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(),
                     "[Move] target_align failed: %s", align_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // 检查悬挂结果
      if (climb_needed_)
      {
        std::lock_guard<std::mutex> lock(suspension_mutex_);
        if (suspension_result_ != BT::NodeStatus::SUCCESS)
        {
          setOutput("message", suspension_message_);
          config().blackboard->set("last_error", suspension_message_);
          config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
          RCLCPP_ERROR(node_->get_logger(),
                       "[Move] Suspension failed: %s", suspension_message_.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }

      phase_ = Phase::DONE;
      // fall through
    }
    [[fallthrough]];

    // =======================================================================
    // Phase::DONE — 更新 blackboard 状态
    // =======================================================================
    case Phase::DONE:
    {
      config().blackboard->set("meilin_current_row", move_row_);
      config().blackboard->set("meilin_current_col", move_col_);
      config().blackboard->set("meilin_current_height", target_height_mm_);
      config().blackboard->set("meilin_current_yaw", target_yaw_);
      config().blackboard->set("meilin_pose_is_cell_center", true);
      // 爬台后悬挂已恢复至正常行驶高度（CLIMB state machine 的 RECOVER 步骤）
      config().blackboard->set("meilin_suspension_offset", 0.0);
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      setOutput("message", std::string{"Move done"});

      RCLCPP_INFO(node_->get_logger(),
                  "[Move] DONE → blackboard: row=%d col=%d h=%.0f yaw=%.2f",
                  move_row_, move_col_, target_height_mm_, target_yaw_);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace r2_bt
