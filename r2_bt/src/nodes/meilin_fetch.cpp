#include "r2_bt/nodes/actions/meilin_fetch.hpp"

#include <cmath>
#include <string>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

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

void MeilinFetch::sendMoveToPoseGoal(double x, double y, double yaw_rad,
                                     const std::string& label)
{
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    align_state_ = ActionState::ACTIVE;
    align_message_.clear();
    align_label_ = label;
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
    fail_goal("Missing ros_node for /move_to_pose client");
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
      align_client_ = rclcpp_action::create_client<MoveToPoseAction>(
          node_, "move_to_pose");
    }
  }

  if (!align_client_->action_server_is_ready())
  {
    fail_goal("/move_to_pose action server not available");
    RCLCPP_ERROR(node_->get_logger(), "[Fetch] %s", align_message_.c_str());
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
      std::lock_guard<std::mutex> lock(align_mutex_);
      if (align_state_ != ActionState::ACTIVE)
      {
        return;
      }
      align_goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        align_state_ = ActionState::FAILED;
        align_message_ = "MoveToPose goal rejected";
      }
    };
  opts.result_callback =
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

  align_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(node_->get_logger(),
              "[Fetch] >>> MoveToPose(%s): x=%.3f y=%.3f yaw=%.1f deg",
              label.c_str(), goal.x, goal.y, goal.yaw_deg);
}

bool MeilinFetch::isAlignDone() const
{
  return align_state_ == ActionState::DONE;
}

bool MeilinFetch::isAlignFailed() const
{
  return align_state_ == ActionState::FAILED;
}

void MeilinFetch::failAlignIfTimedOut()
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double timeout_sec = cfg ? cfg->align_timeout_sec : 30.0;
  if (timeout_sec <= 0.0)
  {
    return;
  }
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - align_start_time_).count();
  if (elapsed <= timeout_sec)
  {
    return;
  }

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
  align_message_ = "MoveToPose timed out";
}

void MeilinFetch::sendStepDirectHeightGoal(double target_height_mm)
{
  {
    std::lock_guard<std::mutex> lock(height_mutex_);
    height_state_ = ActionState::ACTIVE;
    height_message_.clear();
    height_goal_handle_.reset();
  }

  auto fail_goal = [this](const std::string& message) {
    std::lock_guard<std::mutex> lock(height_mutex_);
    height_state_ = ActionState::FAILED;
    height_message_ = message;
  };

  if (!node_)
  {
    fail_goal("Missing ros_node for /step_motion_control client");
    return;
  }

  if (!height_client_)
  {
    rclcpp_action::Client<StepMotionAction>::SharedPtr shared_client;
    if (config().blackboard->get("step_motion_client", shared_client) && shared_client)
    {
      height_client_ = shared_client;
    }
    else
    {
      height_client_ = rclcpp_action::create_client<StepMotionAction>(
          node_, "step_motion_control");
    }
  }

  if (!height_client_->action_server_is_ready())
  {
    fail_goal("/step_motion_control action server not available");
    RCLCPP_ERROR(node_->get_logger(), "[Fetch] %s", height_message_.c_str());
    return;
  }

  auto goal = StepMotionAction::Goal();
  goal.mode = StepMotionAction::Goal::MODE_DIRECT_HEIGHT;
  goal.direction = StepMotionAction::Goal::DIR_FORWARD;
  goal.height = static_cast<float>(target_height_mm);
  goal.correction_x = 0.0;
  goal.correction_y = 0.0;
  goal.correction_yaw_deg = 0.0;
  goal.timeout_sec = static_cast<float>(kHeightTimeoutSec);

  auto opts = rclcpp_action::Client<StepMotionAction>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const std::shared_ptr<StepMotionGoalHandle>& goal_handle) {
      std::lock_guard<std::mutex> lock(height_mutex_);
      if (height_state_ != ActionState::ACTIVE)
      {
        return;
      }
      height_goal_handle_ = goal_handle;
      if (!goal_handle)
      {
        height_state_ = ActionState::FAILED;
        height_message_ = "StepMotion direct-height goal rejected";
      }
    };
  opts.result_callback =
    [this](const StepMotionGoalHandle::WrappedResult& result) {
      std::lock_guard<std::mutex> lock(height_mutex_);
      if (height_state_ != ActionState::ACTIVE)
      {
        return;
      }
      if (result.result)
      {
        height_message_ = result.result->message;
      }
      height_state_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                       result.result && result.result->success)
                          ? ActionState::DONE
                          : ActionState::FAILED;
    };

  height_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(node_->get_logger(),
              "[Fetch] >>> StepMotion MODE_DIRECT_HEIGHT height=%.0f",
              goal.height);
}

bool MeilinFetch::isHeightDone() const
{
  return height_state_ == ActionState::DONE;
}

bool MeilinFetch::isHeightFailed() const
{
  return height_state_ == ActionState::FAILED;
}

void MeilinFetch::sendToolGrasp()
{
  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_sent_ = false;
    tool_request_done_ = false;
    tool_result_ = BT::NodeStatus::FAILURE;
    tool_message_.clear();
    tool_start_time_ = std::chrono::steady_clock::now();
  }

  if (!node_)
  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_done_ = true;
    tool_message_ = "Missing ros_node";
    return;
  }

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  tool_timeout_sec_ = cfg ? cfg->arm_timeout_sec : 30.0;

  if (!tool_client_)
  {
    tool_client_ = node_->create_client<ToolActionSrv>("/ares_tool_node/tool_action");
  }

  if (!tool_client_->service_is_ready())
  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_done_ = true;
    tool_message_ = "tool_node service not available";
    RCLCPP_ERROR(node_->get_logger(), "[Fetch] %s", tool_message_.c_str());
    return;
  }

  auto request = std::make_shared<ToolActionSrv::Request>();
  request->action = "arm_grasp";
  request->args[0] = static_cast<float>(height_diff_);

  tool_client_->async_send_request(request,
    [this](rclcpp::Client<ToolActionSrv>::SharedFuture future) {
      const auto response = future.get();
      std::lock_guard<std::mutex> lock(tool_mutex_);
      tool_request_done_ = true;
      tool_message_ = response->message;
      tool_result_ = (response->success && response->ret == 0)
                         ? BT::NodeStatus::SUCCESS
                         : BT::NodeStatus::FAILURE;
    });

  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_sent_ = true;
  }
  RCLCPP_INFO(node_->get_logger(),
              "[Fetch] >>> tool_node arm_grasp: args[0]=%.0f", height_diff_);
}

void MeilinFetch::failToolIfTimedOut()
{
  if (tool_timeout_sec_ <= 0.0)
  {
    return;
  }
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - tool_start_time_).count();
  if (elapsed <= tool_timeout_sec_)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(tool_mutex_);
  if (tool_request_done_)
  {
    return;
  }
  tool_request_done_ = true;
  tool_result_ = BT::NodeStatus::FAILURE;
  tool_message_ = "tool_node arm_grasp timed out";
}

BT::NodeStatus MeilinFetch::onStart()
{
  if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
  {
    setOutput("message", std::string{"Missing ros_node"});
    return BT::NodeStatus::FAILURE;
  }

  const auto kfs_row = getInput<int>("kfs_row");
  const auto kfs_col = getInput<int>("kfs_col");
  const auto target_yaw = getInput<double>("target_yaw");
  if (!kfs_row || !kfs_col || !target_yaw)
  {
    setOutput("message", std::string{"Fetch missing kfs_row/kfs_col/target_yaw"});
    config().blackboard->set("last_error", std::string{"Fetch missing inputs"});
    return BT::NodeStatus::FAILURE;
  }

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  if (!cfg)
  {
    setOutput("message", std::string{"Missing meilin_config"});
    return BT::NodeStatus::FAILURE;
  }

  kfs_row_ = kfs_row.value();
  kfs_col_ = kfs_col.value();
  target_yaw_ = target_yaw.value();
  height_diff_ = getInput<double>("height_diff").value_or(0.0);
  current_row_ = config().blackboard->get<int>("meilin_current_row");
  current_col_ = config().blackboard->get<int>("meilin_current_col");
  {
    const auto [cx, cy] = meilin_grid_to_world(current_row_, current_col_, *cfg);
    center_x_ = cx;
    center_y_ = cy;
  }

  double actual_yaw = 0.0;
  meilin_calculate_grasp_position(kfs_row_, kfs_col_, target_yaw_, *cfg,
                                  current_row_, current_col_,
                                  grasp_x_, grasp_y_, actual_yaw);
  target_yaw_ = actual_yaw;
  single_axis_mode_ = cfg->motion_mode != "omni";
  height_needed_ = std::abs(height_diff_ - 400.0) < 1.0;
  height_triggered_ = false;

  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    align_state_ = ActionState::IDLE;
    align_message_.clear();
    align_goal_handle_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(height_mutex_);
    height_state_ = ActionState::IDLE;
    height_message_.clear();
    height_goal_handle_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_sent_ = false;
    tool_request_done_ = false;
    tool_result_ = BT::NodeStatus::FAILURE;
    tool_message_.clear();
  }

  phase_ = Phase::START;
  setOutput("message", std::string{});
  config().blackboard->set("active_action", std::string{"Fetch"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});

  RCLCPP_INFO(node_->get_logger(),
              "[Fetch] (%d,%d) -> KFS(%d,%d) grasp=(%.3f,%.3f) yaw=%.1fdeg mode=%s h_diff=%.0f",
              current_row_, current_col_, kfs_row_, kfs_col_,
              grasp_x_, grasp_y_, target_yaw_ * 180.0 / M_PI,
              single_axis_mode_ ? "single_axis" : "omni",
              height_diff_);

  return drive();
}

BT::NodeStatus MeilinFetch::onRunning()
{
  return drive();
}

void MeilinFetch::onHalted()
{
  if (node_)
  {
    RCLCPP_WARN(node_->get_logger(), "[Fetch] Halted");
  }
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    if (align_client_ && align_goal_handle_)
    {
      align_client_->async_cancel_goal(align_goal_handle_);
    }
    align_state_ = ActionState::FAILED;
  }
  {
    std::lock_guard<std::mutex> lock(height_mutex_);
    if (height_client_ && height_goal_handle_)
    {
      height_client_->async_cancel_goal(height_goal_handle_);
    }
    height_state_ = ActionState::FAILED;
  }
  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_done_ = true;
  }
}

BT::NodeStatus MeilinFetch::drive()
{
  switch (phase_)
  {
    case Phase::START:
    {
      if (single_axis_mode_)
      {
        phase_ = Phase::WAIT_CENTER_ALIGN;
        sendMoveToPoseGoal(center_x_, center_y_, target_yaw_, "center_align");
      }
      else
      {
        phase_ = Phase::WAIT_FETCH_TRANSLATE;
        sendMoveToPoseGoal(grasp_x_, grasp_y_, target_yaw_, "fetch_pose");
      }
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_CENTER_ALIGN:
    {
      failAlignIfTimedOut();
      if (isAlignFailed())
      {
        setOutput("message", align_message_);
        config().blackboard->set("last_error", align_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(), "[Fetch] center_align failed: %s",
                     align_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (!isAlignDone())
      {
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::START_FETCH_TRANSLATE;
    }
    [[fallthrough]];

    case Phase::START_FETCH_TRANSLATE:
    {
      phase_ = Phase::WAIT_FETCH_TRANSLATE;
      sendMoveToPoseGoal(grasp_x_, grasp_y_, target_yaw_, "fetch_translate");
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_FETCH_TRANSLATE:
    {
      failAlignIfTimedOut();
      if (isAlignFailed())
      {
        setOutput("message", align_message_);
        config().blackboard->set("last_error", align_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(), "[Fetch] fetch motion failed: %s",
                     align_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (!isAlignDone())
      {
        return BT::NodeStatus::RUNNING;
      }
      phase_ = height_needed_ ? Phase::START_HEIGHT : Phase::START_TOOL;
    }
    [[fallthrough]];

    case Phase::START_HEIGHT:
    {
      if (!height_needed_)
      {
        phase_ = Phase::START_TOOL;
        return drive();
      }
      height_triggered_ = true;
      phase_ = Phase::WAIT_HEIGHT;
      sendStepDirectHeightGoal(kFetchLiftHeight);
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_HEIGHT:
    {
      if (isHeightFailed())
      {
        setOutput("message", height_message_);
        config().blackboard->set("last_error", height_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(), "[Fetch] height adjustment failed: %s",
                     height_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (!isHeightDone())
      {
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::START_TOOL;
    }
    [[fallthrough]];

    case Phase::START_TOOL:
    {
      phase_ = Phase::WAIT_TOOL;
      sendToolGrasp();
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_TOOL:
    {
      failToolIfTimedOut();
      bool done = false;
      {
        std::lock_guard<std::mutex> lock(tool_mutex_);
        done = tool_request_done_;
      }
      if (!done)
      {
        return BT::NodeStatus::RUNNING;
      }
      {
        std::lock_guard<std::mutex> lock(tool_mutex_);
        if (tool_result_ != BT::NodeStatus::SUCCESS)
        {
          setOutput("message", tool_message_);
          config().blackboard->set("last_error", tool_message_);
          config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
          RCLCPP_ERROR(node_->get_logger(), "[Fetch] arm_grasp failed: %s",
                       tool_message_.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }
      phase_ = Phase::DONE;
    }
    [[fallthrough]];

    case Phase::DONE:
    {
      config().blackboard->set("meilin_current_yaw", target_yaw_);
      config().blackboard->set("meilin_pose_is_cell_center", false);
      if (height_triggered_)
      {
        const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
        const double normal_height = cfg ? cfg->suspension_normal_height : 30.0;
        config().blackboard->set(
            "meilin_suspension_offset", kFetchLiftHeight - normal_height);
      }
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      setOutput("message", std::string{"Fetch done"});
      RCLCPP_INFO(node_->get_logger(), "[Fetch] DONE");
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace r2_bt
