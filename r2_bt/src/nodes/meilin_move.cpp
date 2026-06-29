#include "r2_bt/nodes/actions/meilin_move.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

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

void MeilinMove::resetActions()
{
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    move_state_ = ActionState::IDLE;
    move_message_.clear();
    move_goal_handle_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(step_mutex_);
    step_state_ = ActionState::IDLE;
    step_message_.clear();
    step_goal_handle_.reset();
  }
}

void MeilinMove::sendMoveToPoseGoal(double x, double y, double yaw_rad)
{
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    move_state_ = ActionState::ACTIVE;
    move_message_.clear();
    move_goal_handle_.reset();
    move_start_time_ = std::chrono::steady_clock::now();
  }

  auto fail_goal = [this](const std::string& message) {
    std::lock_guard<std::mutex> lock(move_mutex_);
    move_state_ = ActionState::FAILED;
    move_message_ = message;
  };

  if (!node_)
  {
    fail_goal("Missing ros_node for /move_to_pose client");
    return;
  }

  if (!move_client_)
  {
    rclcpp_action::Client<MoveToPoseAction>::SharedPtr shared_client;
    if (config().blackboard->get("move_to_pose_client", shared_client) && shared_client)
    {
      move_client_ = shared_client;
    }
    else
    {
      move_client_ =
          rclcpp_action::create_client<MoveToPoseAction>(node_, "move_to_pose");
    }
  }

  if (!move_client_->action_server_is_ready())
  {
    fail_goal("/move_to_pose action server not available");
    RCLCPP_ERROR(node_->get_logger(), "[Move] %s", move_message_.c_str());
    return;
  }

  auto goal = MoveToPoseAction::Goal();
  goal.x = x;
  goal.y = y;
  goal.yaw_deg = yaw_rad * 180.0 / M_PI;
  goal.pid_profile = MoveToPoseAction::Goal::PID_PROFILE_SLOW;

  auto opts = rclcpp_action::Client<MoveToPoseAction>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const std::shared_ptr<MoveToPoseGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(move_mutex_);
      if (move_state_ != ActionState::ACTIVE)
      {
        return;
      }
      move_goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        move_state_ = ActionState::FAILED;
        move_message_ = "Goal rejected by /move_to_pose";
      }
    };
  opts.result_callback =
    [this](const MoveToPoseGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(move_mutex_);
      if (move_state_ != ActionState::ACTIVE)
      {
        return;
      }
      if (result.result)
      {
        move_message_ = result.result->message;
      }
      move_state_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                     result.result && result.result->success)
                        ? ActionState::DONE
                        : ActionState::FAILED;
    };
  opts.feedback_callback =
    [this](std::shared_ptr<MoveToPoseGoalHandle>,
           const std::shared_ptr<const MoveToPoseAction::Feedback> feedback) {
      if (!feedback || !node_)
      {
        return;
      }
      std::ostringstream debug;
      debug << std::fixed << std::setprecision(3)
            << "phase=" << feedback->phase
            << " cur=(" << feedback->current_x << "," << feedback->current_y << ")"
            << " target=(" << feedback->target_x << "," << feedback->target_y << ")"
            << " yaw_err=" << std::setprecision(1) << feedback->yaw_error_deg
            << "deg dist_err=" << std::setprecision(3) << feedback->distance_error
            << " cmd=(" << feedback->cmd_vx << ","
            << feedback->cmd_vy << ","
            << feedback->cmd_wz << ")";
      const std::string debug_msg = debug.str();
      config().blackboard->set("move_feedback_debug", debug_msg);
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[MeilinMove feedback] %s",
        debug_msg.c_str());
    };

  move_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(node_->get_logger(),
              "[Move] >>> /move_to_pose: x=%.3f y=%.3f yaw=%.1f deg",
              goal.x, goal.y, goal.yaw_deg);
}

void MeilinMove::failMoveToPoseIfTimedOut()
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double timeout_sec = cfg ? cfg->align_timeout_sec : 30.0;
  if (timeout_sec <= 0.0)
  {
    return;
  }

  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - move_start_time_).count();
  if (elapsed <= timeout_sec)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(move_mutex_);
  if (move_state_ != ActionState::ACTIVE)
  {
    return;
  }
  if (move_client_ && move_goal_handle_)
  {
    move_client_->async_cancel_goal(move_goal_handle_);
  }
  move_state_ = ActionState::FAILED;
  move_message_ = "/move_to_pose timed out";
}

bool MeilinMove::moveDone() const
{
  return move_state_ == ActionState::DONE;
}

bool MeilinMove::moveFailed() const
{
  return move_state_ == ActionState::FAILED;
}

void MeilinMove::sendStepMotionGoal()
{
  {
    std::lock_guard<std::mutex> lock(step_mutex_);
    step_state_ = ActionState::ACTIVE;
    step_message_.clear();
    step_goal_handle_.reset();
  }

  auto fail_goal = [this](const std::string& message) {
    std::lock_guard<std::mutex> lock(step_mutex_);
    step_state_ = ActionState::FAILED;
    step_message_ = message;
  };

  if (!node_)
  {
    fail_goal("Missing ros_node for /step_motion_control client");
    return;
  }

  if (!step_client_)
  {
    rclcpp_action::Client<StepMotionAction>::SharedPtr shared_client;
    if (config().blackboard->get("step_motion_client", shared_client) && shared_client)
    {
      step_client_ = shared_client;
    }
    else
    {
      step_client_ =
          rclcpp_action::create_client<StepMotionAction>(node_, "step_motion_control");
    }
  }

  if (!step_client_->action_server_is_ready())
  {
    fail_goal("/step_motion_control action server not available");
    RCLCPP_ERROR(node_->get_logger(), "[Move] %s", step_message_.c_str());
    return;
  }

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double timeout_sec = cfg ? cfg->suspension_timeout_sec : 10.0;

  auto goal = StepMotionAction::Goal();
  goal.mode = step_mode_;
  goal.direction = step_direction_;
  goal.height = static_cast<float>(step_height_mm_);
  goal.correction_x = step_correction_x_;
  goal.correction_y = step_correction_y_;
  goal.correction_yaw_deg = step_correction_yaw_deg_;
  goal.timeout_sec = static_cast<float>(timeout_sec);

  auto opts = rclcpp_action::Client<StepMotionAction>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const std::shared_ptr<StepMotionGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(step_mutex_);
      if (step_state_ != ActionState::ACTIVE)
      {
        return;
      }
      step_goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        step_state_ = ActionState::FAILED;
        step_message_ = "Goal rejected by /step_motion_control";
      }
    };
  opts.result_callback =
    [this](const StepMotionGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(step_mutex_);
      if (step_state_ != ActionState::ACTIVE)
      {
        return;
      }
      if (result.result)
      {
        step_message_ = result.result->message;
      }
      step_state_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                     result.result && result.result->success)
                        ? ActionState::DONE
                        : ActionState::FAILED;
    };

  step_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(node_->get_logger(),
              "[Move] >>> /step_motion_control: mode=%u dir=%u height=%.0f "
              "corr=(%.3f,%.3f,%.1fdeg) timeout=%.1f",
              goal.mode, goal.direction, goal.height,
              goal.correction_x, goal.correction_y, goal.correction_yaw_deg,
              goal.timeout_sec);
}

bool MeilinMove::stepDone() const
{
  return step_state_ == ActionState::DONE;
}

bool MeilinMove::stepFailed() const
{
  return step_state_ == ActionState::FAILED;
}

BT::NodeStatus MeilinMove::onStart()
{
  if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
  {
    setOutput("message", std::string{"Missing ros_node on blackboard"});
    return BT::NodeStatus::FAILURE;
  }

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

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  if (!cfg)
  {
    const std::string err = "Missing meilin_config on blackboard";
    setOutput("message", err);
    config().blackboard->set("last_error", err);
    return BT::NodeStatus::FAILURE;
  }

  move_row_ = move_row.value();
  move_col_ = move_col.value();
  target_yaw_ = target_yaw.value();
  target_height_mm_ = getInput<double>("target_height_mm").value_or(
      meilin_height_at(move_row_, move_col_, cfg->side));
  {
    const auto [wx, wy] = meilin_grid_to_world(move_row_, move_col_, *cfg);
    target_x_ = wx;
    target_y_ = wy;
  }

  from_row_ = config().blackboard->get<int>("meilin_current_row");
  from_col_ = config().blackboard->get<int>("meilin_current_col");
  current_height_mm_ = config().blackboard->get<double>("meilin_current_height");
  current_yaw_ = config().blackboard->get<double>("meilin_current_yaw");
  pose_is_center_ = config().blackboard->get<bool>("meilin_pose_is_cell_center");
  entry_move_ = config().blackboard->get<bool>("meilin_entry_move_pending");

  pose_fresh_ = false;
  const bool pose_received = config().blackboard->get<bool>("meilin_pose_received");
  if (pose_received)
  {
    const double last_update_sec =
        config().blackboard->get<double>("meilin_pose_last_update_sec");
    const double pose_age_sec = node_->now().seconds() - last_update_sec;
    pose_fresh_ = cfg->pose_timeout_sec <= 0.0 || pose_age_sec <= cfg->pose_timeout_sec;
    if (pose_fresh_)
    {
      pose_x_ = config().blackboard->get<double>("meilin_pose_x");
      pose_y_ = config().blackboard->get<double>("meilin_pose_y");
      pose_yaw_ = config().blackboard->get<double>("meilin_pose_yaw");
      current_yaw_ = pose_yaw_;

      int pose_row = 0;
      int pose_col = 0;
      if (meilin_world_to_grid(pose_x_, pose_y_, *cfg, pose_row, pose_col))
      {
        const auto [center_x, center_y] = meilin_grid_to_world(pose_row, pose_col, *cfg);
        const double center_dist = std::hypot(pose_x_ - center_x, pose_y_ - center_y);
        if (!entry_move_)
        {
          from_row_ = pose_row;
          from_col_ = pose_col;
          current_height_mm_ = meilin_height_at(from_row_, from_col_, cfg->side);
          pose_is_center_ = center_dist <= cfg->cell_center_tolerance;
          config().blackboard->set("meilin_current_row", from_row_);
          config().blackboard->set("meilin_current_col", from_col_);
          config().blackboard->set("meilin_current_height", current_height_mm_);
          config().blackboard->set("meilin_pose_is_cell_center", pose_is_center_);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "[Move] localization: x=%.3f y=%.3f yaw=%.2f grid=(%d,%d) center_dist=%.3f",
                    pose_x_, pose_y_, current_yaw_, pose_row, pose_col, center_dist);
      }
      else if (!entry_move_)
      {
        const std::string err = "Current relocation pose is outside Meilin grid";
        setOutput("message", err);
        config().blackboard->set("last_error", err);
        RCLCPP_ERROR(node_->get_logger(), "[Move] %s: x=%.3f y=%.3f",
                     err.c_str(), pose_x_, pose_y_);
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  config().blackboard->set("meilin_current_yaw", current_yaw_);

  const double height_diff = target_height_mm_ - current_height_mm_;
  const bool same_grid = from_row_ == move_row_ && from_col_ == move_col_;
  step_needed_ = !entry_move_ && !same_grid &&
                 std::abs(height_diff) > cfg->height_tolerance;

  if (step_needed_)
  {
    if (!meilin_adjacent(from_row_, from_col_, move_row_, move_col_))
    {
      const std::string err = "Move with height change requires adjacent target grid";
      setOutput("message", err);
      config().blackboard->set("last_error", err);
      return BT::NodeStatus::FAILURE;
    }

    double move_yaw = 0.0;
    std::string dir_name;
    int direction = 0;
    if (!meilin_direction_yaw(from_row_, from_col_, move_row_, move_col_,
                              move_yaw, direction, dir_name, current_yaw_))
    {
      const std::string err = "Move step direction is invalid";
      setOutput("message", err);
      config().blackboard->set("last_error", err);
      return BT::NodeStatus::FAILURE;
    }
    if (dir_name == "backward")
    {
      const std::string err = "StepMotionControl does not support backward direction";
      setOutput("message", err);
      config().blackboard->set("last_error", err);
      return BT::NodeStatus::FAILURE;
    }

    step_mode_ = static_cast<uint8_t>(
        height_diff > 0.0
            ? StepMotionAction::Goal::MODE_CLIMB_UP
            : StepMotionAction::Goal::MODE_CLIMB_DOWN);
    step_direction_ = static_cast<uint8_t>(direction);
    step_height_mm_ = std::abs(height_diff);

    const auto [cx, cy] = meilin_grid_to_world(from_row_, from_col_, *cfg);
    step_correction_x_ = cx;
    step_correction_y_ = cy;
    step_correction_yaw_deg_ = meilin_snap_cardinal_yaw(current_yaw_) * 180.0 / M_PI;

    RCLCPP_INFO(node_->get_logger(),
                "[Move] step: %s dir=%s height=%.0f correction(anchor+yaw)=(%.3f,%.3f,%.1fdeg)",
                step_mode_ == StepMotionAction::Goal::MODE_CLIMB_UP ? "UP" : "DOWN",
                dir_name.c_str(), step_height_mm_,
                step_correction_x_, step_correction_y_, step_correction_yaw_deg_);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(),
                "[Move] no step: entry=%s same_grid=%s diff=%.0f tol=%.0f",
                entry_move_ ? "yes" : "no",
                same_grid ? "yes" : "no",
                height_diff, cfg->height_tolerance);
  }

  resetActions();
  phase_ = Phase::START;
  setOutput("message", std::string{});
  config().blackboard->set("active_action", std::string{"Move"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  config().blackboard->set("move_feedback_debug", std::string{});

  return drive();
}

BT::NodeStatus MeilinMove::onRunning()
{
  return drive();
}

void MeilinMove::onHalted()
{
  if (node_)
  {
    RCLCPP_WARN(node_->get_logger(), "[Move] Halted");
  }
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    if (move_client_ && move_goal_handle_)
    {
      move_client_->async_cancel_goal(move_goal_handle_);
    }
    move_state_ = ActionState::FAILED;
  }
  {
    std::lock_guard<std::mutex> lock(step_mutex_);
    if (step_client_ && step_goal_handle_)
    {
      step_client_->async_cancel_goal(step_goal_handle_);
    }
    step_state_ = ActionState::FAILED;
  }
}

BT::NodeStatus MeilinMove::drive()
{
  switch (phase_)
  {
    case Phase::START:
    {
      if (step_needed_)
      {
        sendStepMotionGoal();
        phase_ = Phase::WAIT_STEP;
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::START_MOVE;
    }
    [[fallthrough]];

    case Phase::START_MOVE:
    {
      sendMoveToPoseGoal(target_x_, target_y_, target_yaw_);
      phase_ = Phase::WAIT_MOVE;
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_STEP:
    {
      if (stepFailed())
      {
        setOutput("message", step_message_);
        config().blackboard->set("last_error", step_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(), "[Move] step failed: %s", step_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (!stepDone())
      {
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::START_MOVE;
      return drive();
    }

    case Phase::WAIT_MOVE:
    {
      failMoveToPoseIfTimedOut();
      if (moveFailed())
      {
        setOutput("message", move_message_);
        config().blackboard->set("last_error", move_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(), "[Move] move_to_pose failed: %s",
                     move_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (!moveDone())
      {
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::DONE;
    }
    [[fallthrough]];

    case Phase::DONE:
    {
      config().blackboard->set("meilin_current_row", move_row_);
      config().blackboard->set("meilin_current_col", move_col_);
      config().blackboard->set("meilin_current_height", target_height_mm_);
      config().blackboard->set("meilin_current_yaw", target_yaw_);
      config().blackboard->set("meilin_pose_is_cell_center", true);
      config().blackboard->set("meilin_suspension_offset", 0.0);
      if (entry_move_)
      {
        config().blackboard->set("meilin_entry_move_pending", false);
      }
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      setOutput("message", std::string{"Move done"});
      RCLCPP_INFO(node_->get_logger(),
                  "[Move] DONE: row=%d col=%d h=%.0f yaw=%.2f entry=%s",
                  move_row_, move_col_, target_height_mm_, target_yaw_,
                  entry_move_ ? "yes" : "no");
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace r2_bt
