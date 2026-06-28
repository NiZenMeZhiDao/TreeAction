#include "r2_bt/nodes/actions/meilin_fetch.hpp"

#include <cmath>
#include <string>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

// ===========================================================================
// MoveToPose Action Client
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

  if (!node_) { fail_goal("Missing ros_node for MoveToPose client"); return; }

  if (!align_client_)
  {
    rclcpp_action::Client<MoveToPoseAction>::SharedPtr shared_client;
    if (config().blackboard->get("move_to_pose_client", shared_client) && shared_client)
      align_client_ = shared_client;
    else
      align_client_ = rclcpp_action::create_client<MoveToPoseAction>(node_, "move_to_pose");
  }

  if (!align_client_->action_server_is_ready())
  {
    fail_goal("/move_to_pose action server not available");
    RCLCPP_ERROR(node_->get_logger(), "[MeilinFetch] %s", align_message_.c_str());
    return;
  }

  auto goal = MoveToPoseAction::Goal();
  goal.x = grasp_x_;
  goal.y = grasp_y_;
  goal.yaw_deg = target_yaw_ * 180.0 / M_PI;
  goal.pid_profile = MoveToPoseAction::Goal::PID_PROFILE_SLOW;

  auto opts = rclcpp_action::Client<MoveToPoseAction>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const std::shared_ptr<MoveToPoseGoalHandle>& gh) {
      std::lock_guard<std::mutex> lock(align_mutex_);
      if (align_state_ != ActionState::ACTIVE) return;
      align_goal_handle_ = gh;
      if (!gh) { align_state_ = ActionState::FAILED; align_message_ = "MoveToPose goal rejected"; }
    };
  opts.result_callback =
    [this](const MoveToPoseGoalHandle::WrappedResult& r) {
      std::lock_guard<std::mutex> lock(align_mutex_);
      if (align_state_ != ActionState::ACTIVE) return;
      if (r.result) align_message_ = r.result->message;
      align_state_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED &&
                      r.result && r.result->success)
                         ? ActionState::DONE : ActionState::FAILED;
    };

  align_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(node_->get_logger(), "[Fetch] >>> MoveToPose(grasp): (%.3f,%.3f) yaw=%.1f°",
              goal.x, goal.y, goal.yaw_deg);
}

bool MeilinFetch::isAlignDone() const  { return align_state_ == ActionState::DONE; }
bool MeilinFetch::isAlignFailed() const { return align_state_ == ActionState::FAILED; }

void MeilinFetch::failAlignIfTimedOut()
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double t = cfg ? cfg->align_timeout_sec : 30.0;
  if (t <= 0.0) return;
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - align_start_time_).count() <= t) return;

  std::lock_guard<std::mutex> lock(align_mutex_);
  if (align_state_ != ActionState::ACTIVE) return;
  if (align_client_ && align_goal_handle_) align_client_->async_cancel_goal(align_goal_handle_);
  align_state_ = ActionState::FAILED;
  align_message_ = "MoveToPose timed out";
}

// ===========================================================================
// Suspension Action Client（仅 height_diff==400）
// ===========================================================================

void MeilinFetch::sendSuspensionGoal(double target_height_mm)
{
  {
    std::lock_guard<std::mutex> lock(suspension_mutex_);
    suspension_goal_done_ = false;
    suspension_result_ = BT::NodeStatus::FAILURE;
    suspension_message_.clear();
    suspension_goal_handle_.reset();
  }

  if (!node_) { suspension_goal_done_ = true; suspension_message_ = "Missing ros_node"; return; }

  if (!suspension_client_)
    suspension_client_ = rclcpp_action::create_client<SuspensionAction>(node_, "suspension_control");

  if (!suspension_client_->action_server_is_ready())
  {
    suspension_goal_done_ = true;
    suspension_message_ = "SuspensionControl not available";
    RCLCPP_ERROR(node_->get_logger(), "[Fetch] %s", suspension_message_.c_str());
    return;
  }

  auto goal = SuspensionAction::Goal();
  goal.mode = SuspensionAction::Goal::MODE_DIRECT;
  goal.direction = SuspensionAction::Goal::DIR_FORWARD;
  goal.height = static_cast<float>(target_height_mm);
  goal.timeout_sec = static_cast<float>(kSuspensionTimeoutSec);

  auto opts = rclcpp_action::Client<SuspensionAction>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const std::shared_ptr<SuspensionGoalHandle>& gh) {
      std::lock_guard<std::mutex> lock(suspension_mutex_);
      suspension_goal_handle_ = gh;
      if (!gh) { suspension_goal_done_ = true; suspension_result_ = BT::NodeStatus::FAILURE;
                 suspension_message_ = "Suspension goal rejected"; }
    };
  opts.result_callback =
    [this](const SuspensionGoalHandle::WrappedResult& r) {
      std::lock_guard<std::mutex> lock(suspension_mutex_);
      suspension_goal_done_ = true;
      if (r.result) suspension_message_ = r.result->message;
      suspension_result_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED &&
                            r.result && r.result->success)
                               ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };

  suspension_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(node_->get_logger(), "[Fetch] >>> Suspension: MODE_DIRECT height=%.0f", goal.height);
}

// ===========================================================================
// tool_node Service Client（arm_grasp）
// ===========================================================================

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

  if (!node_) { tool_request_done_ = true; tool_message_ = "Missing ros_node"; return; }

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  tool_timeout_sec_ = cfg ? cfg->arm_timeout_sec : 30.0;

  if (!tool_client_)
    tool_client_ = node_->create_client<ToolActionSrv>("/ares_tool_node/tool_action");

  if (!tool_client_->service_is_ready())
  {
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
      const auto r = future.get();
      std::lock_guard<std::mutex> lock(tool_mutex_);
      tool_request_done_ = true;
      tool_message_ = r->message;
      tool_result_ = (r->success && r->ret == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

  { std::lock_guard<std::mutex> lock(tool_mutex_); tool_request_sent_ = true; }
  RCLCPP_INFO(node_->get_logger(), "[Fetch] >>> tool_node arm_grasp: args[0]=%.0f", height_diff_);
}

void MeilinFetch::failToolIfTimedOut()
{
  if (tool_timeout_sec_ <= 0.0) return;
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - tool_start_time_).count() <= tool_timeout_sec_) return;
  std::lock_guard<std::mutex> lock(tool_mutex_);
  if (tool_request_done_) return;
  tool_request_done_ = true;
  tool_result_ = BT::NodeStatus::FAILURE;
  tool_message_ = "tool_node arm_grasp timed out";
}

// ===========================================================================
// BT Node 接口
// ===========================================================================

MeilinFetch::MeilinFetch(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

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
  if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
  {
    setOutput("message", std::string{"Missing ros_node"});
    return BT::NodeStatus::FAILURE;
  }

  // 1. 读取输入
  const auto kfs_row = getInput<int>("kfs_row");
  const auto kfs_col = getInput<int>("kfs_col");
  const auto target_yaw = getInput<double>("target_yaw");
  if (!kfs_row || !kfs_col || !target_yaw)
  {
    setOutput("message", std::string{"Fetch missing kfs_row/kfs_col/target_yaw"});
    config().blackboard->set("last_error", std::string{"Fetch missing inputs"});
    return BT::NodeStatus::FAILURE;
  }
  kfs_row_ = kfs_row.value();
  kfs_col_ = kfs_col.value();
  target_yaw_ = target_yaw.value();
  height_diff_ = getInput<double>("height_diff").value_or(0.0);

  // 2. 配置和当前状态
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  if (!cfg) { setOutput("message", std::string{"Missing meilin_config"}); return BT::NodeStatus::FAILURE; }

  const int current_row = config().blackboard->get<int>("meilin_current_row");
  const int current_col = config().blackboard->get<int>("meilin_current_col");
  RCLCPP_INFO(node_->get_logger(), "[Fetch] (%d,%d) → KFS(%d,%d) h_diff=%.0f yaw=%.2f",
              current_row, current_col, kfs_row_, kfs_col_, height_diff_, target_yaw_);

  // 3. 计算抓取位姿
  {
    double actual_yaw = 0.0;
    meilin_calculate_grasp_position(kfs_row_, kfs_col_, target_yaw_, *cfg,
                                    current_row, current_col, grasp_x_, grasp_y_, actual_yaw);
    target_yaw_ = actual_yaw;
  }
  RCLCPP_INFO(node_->get_logger(), "[Fetch] grasp pose: (%.3f,%.3f) yaw=%.1f°",
              grasp_x_, grasp_y_, target_yaw_ * 180.0 / M_PI);

  // 4. 判断是否需要悬挂（仅 height_diff==400）
  suspension_needed_ = (std::abs(height_diff_ - 400.0) < 1.0);
  suspension_triggered_ = false;
  suspension_resetting_ = false;

  // 5. 发出 MoveToPose
  align_state_ = ActionState::IDLE;
  align_message_.clear();
  align_goal_handle_.reset();
  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_sent_ = false;
    tool_request_done_ = false;
  }
  phase_ = Phase::START;
  setOutput("message", std::string{});
  config().blackboard->set("active_action", std::string{"Fetch"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  return drive();
}

BT::NodeStatus MeilinFetch::onRunning() { return drive(); }

void MeilinFetch::onHalted()
{
  if (node_) RCLCPP_WARN(node_->get_logger(), "[Fetch] Halted");
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    if (align_client_ && align_goal_handle_) { align_client_->async_cancel_goal(align_goal_handle_); }
    align_state_ = ActionState::FAILED;
  }
  {
    std::lock_guard<std::mutex> lock(suspension_mutex_);
    if (suspension_client_ && suspension_goal_handle_) { suspension_client_->async_cancel_goal(suspension_goal_handle_); }
    suspension_goal_done_ = true;
  }
  { std::lock_guard<std::mutex> lock(tool_mutex_); tool_request_done_ = true; }
}

// ===========================================================================
// 核心状态机
// ===========================================================================

BT::NodeStatus MeilinFetch::drive()
{
  switch (phase_)
  {
    // ===================================================================
    case Phase::START:
    {
      phase_ = Phase::WAIT_ALIGN;
      RCLCPP_INFO(node_->get_logger(), "[Fetch] >>> MoveToPose");
      sendMoveToPoseGoal();
      return BT::NodeStatus::RUNNING;
    }

    // ===================================================================
    case Phase::WAIT_ALIGN:
    {
      failAlignIfTimedOut();
      if (align_state_ == ActionState::ACTIVE) return BT::NodeStatus::RUNNING;
      if (isAlignFailed())
      {
        setOutput("message", align_message_);
        config().blackboard->set("last_error", align_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(), "[Fetch] align failed: %s", align_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // MoveToPose 完成 → 如需悬挂则进入悬挂阶段，否则直接 arm_grasp
      if (suspension_needed_)
        phase_ = Phase::START_SUSPENSION;
      else
        phase_ = Phase::START_TOOL;
      // fall through
    }
    [[fallthrough]];

    // ===================================================================
    case Phase::START_SUSPENSION:
    {
      phase_ = Phase::WAIT_SUSPENSION;
      suspension_triggered_ = true;
      RCLCPP_INFO(node_->get_logger(), "[Fetch] >>> Suspension lift to %.0fmm", kSuspension400LiftHeight);
      sendSuspensionGoal(kSuspension400LiftHeight);
      return BT::NodeStatus::RUNNING;
    }

    // ===================================================================
    case Phase::WAIT_SUSPENSION:
    {
      bool done = false;
      { std::lock_guard<std::mutex> lock(suspension_mutex_); done = suspension_goal_done_; }
      if (!done) return BT::NodeStatus::RUNNING;

      {
        std::lock_guard<std::mutex> lock(suspension_mutex_);
        if (suspension_result_ != BT::NodeStatus::SUCCESS)
        {
          setOutput("message", suspension_message_);
          config().blackboard->set("last_error", suspension_message_);
          config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
          RCLCPP_ERROR(node_->get_logger(), "[Fetch] suspension lift failed: %s", suspension_message_.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }

      phase_ = Phase::START_TOOL;
      // fall through
    }
    [[fallthrough]];

    // ===================================================================
    case Phase::START_TOOL:
    {
      phase_ = Phase::WAIT_TOOL;
      RCLCPP_INFO(node_->get_logger(), "[Fetch] >>> tool_node arm_grasp");
      sendToolGrasp();
      return BT::NodeStatus::RUNNING;
    }

    // ===================================================================
    case Phase::WAIT_TOOL:
    {
      failToolIfTimedOut();
      bool done = false;
      { std::lock_guard<std::mutex> lock(tool_mutex_); done = tool_request_done_; }
      if (!done) return BT::NodeStatus::RUNNING;

      {
        std::lock_guard<std::mutex> lock(tool_mutex_);
        if (tool_result_ != BT::NodeStatus::SUCCESS)
        {
          setOutput("message", tool_message_);
          config().blackboard->set("last_error", tool_message_);
          config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
          RCLCPP_ERROR(node_->get_logger(), "[Fetch] arm_grasp failed: %s", tool_message_.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }

      // arm_grasp 完成 → 如需复位悬挂则进入复位阶段
      if (suspension_triggered_)
        phase_ = Phase::RESET_SUSPENSION;
      else
        phase_ = Phase::DONE;
      // fall through
    }
    [[fallthrough]];

    // ===================================================================
    case Phase::RESET_SUSPENSION:
    {
      phase_ = Phase::WAIT_RESET;
      suspension_resetting_ = true;
      RCLCPP_INFO(node_->get_logger(), "[Fetch] >>> Suspension reset to %.0fmm", kSuspensionNormalHeight);
      sendSuspensionGoal(kSuspensionNormalHeight);
      return BT::NodeStatus::RUNNING;
    }

    // ===================================================================
    case Phase::WAIT_RESET:
    {
      bool done = false;
      { std::lock_guard<std::mutex> lock(suspension_mutex_); done = suspension_goal_done_; }
      if (!done) return BT::NodeStatus::RUNNING;

      {
        std::lock_guard<std::mutex> lock(suspension_mutex_);
        if (suspension_result_ != BT::NodeStatus::SUCCESS)
        {
          // 复位失败不致命，记录日志继续
          RCLCPP_ERROR(node_->get_logger(), "[Fetch] suspension reset failed (non-fatal): %s",
                       suspension_message_.c_str());
        }
      }

      phase_ = Phase::DONE;
      // fall through
    }
    [[fallthrough]];

    // ===================================================================
    case Phase::DONE:
    {
      config().blackboard->set("meilin_current_yaw", target_yaw_);
      config().blackboard->set("meilin_pose_is_cell_center", false);
      config().blackboard->set("meilin_suspension_offset", 0.0);
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      setOutput("message", std::string{"Fetch done"});
      RCLCPP_INFO(node_->get_logger(), "[Fetch] DONE");
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace r2_bt
