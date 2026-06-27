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
      if (align_state_ != ActionState::ACTIVE) return;
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
      if (align_state_ != ActionState::ACTIVE) return;
      if (result.result) align_message_ = result.result->message;
      align_state_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                      result.result && result.result->success)
                         ? ActionState::DONE
                         : ActionState::FAILED;
    };

  align_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(node_->get_logger(),
              "[MeilinFetch] >>> /move_to_pose goal(grasp): x=%.3f y=%.3f yaw=%.1f°",
              goal.x, goal.y, goal.yaw_deg);
}

bool MeilinFetch::isAlignDone() const  { return align_state_ == ActionState::DONE; }
bool MeilinFetch::isAlignFailed() const { return align_state_ == ActionState::FAILED; }

void MeilinFetch::failAlignIfTimedOut()
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  const double timeout_sec = cfg ? cfg->align_timeout_sec : 30.0;
  if (timeout_sec <= 0.0) return;

  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - align_start_time_).count();
  if (elapsed <= timeout_sec) return;

  std::lock_guard<std::mutex> lock(align_mutex_);
  if (align_state_ != ActionState::ACTIVE) return;
  if (align_client_ && align_goal_handle_)
    align_client_->async_cancel_goal(align_goal_handle_);
  align_state_ = ActionState::FAILED;
  align_message_ = "MoveToPose timed out";
}

// ===========================================================================
// tool_node Service Client（内嵌 arm_grasp）
// 传入 height_diff 作为 args[0]，tool_node 负责映射和悬挂（仅 400 时）
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

  if (!node_)
  {
    tool_request_done_ = true;
    tool_result_ = BT::NodeStatus::FAILURE;
    tool_message_ = "Missing ros_node for tool_node client";
    RCLCPP_ERROR(rclcpp::get_logger("MeilinFetch"), "%s", tool_message_.c_str());
    return;
  }

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  tool_timeout_sec_ = cfg ? cfg->arm_timeout_sec : 30.0;

  if (!tool_client_)
    tool_client_ = node_->create_client<ToolActionSrv>("/ares_tool_node/tool_action");

  if (!tool_client_->service_is_ready())
  {
    tool_request_done_ = true;
    tool_result_ = BT::NodeStatus::FAILURE;
    tool_message_ = "tool_node service not available";
    RCLCPP_ERROR(node_->get_logger(), "[MeilinFetch] %s", tool_message_.c_str());
    return;
  }

  auto request = std::make_shared<ToolActionSrv::Request>();
  request->action = "arm_grasp";
  request->args[0] = static_cast<float>(height_diff_);

  tool_client_->async_send_request(
      request,
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
              "[MeilinFetch] >>> tool_node arm_grasp: args[0]=%.0f timeout=%.1fs",
              height_diff_, tool_timeout_sec_);
}

void MeilinFetch::failToolIfTimedOut()
{
  if (tool_timeout_sec_ <= 0.0) return;

  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - tool_start_time_).count();
  if (elapsed <= tool_timeout_sec_) return;

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

  // 1. 读取输入
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

  // 2. 读取配置和当前状态
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

  // 3. 计算抓取位姿
  {
    double actual_yaw = 0.0;
    meilin_calculate_grasp_position(kfs_row_, kfs_col_, target_yaw_, *cfg,
                                    current_row, current_col,
                                    grasp_x_, grasp_y_, actual_yaw);
    target_yaw_ = actual_yaw;
  }
  RCLCPP_INFO(node_->get_logger(),
              "[Fetch] grasp pose: (%.3f,%.3f) yaw=%.1f°",
              grasp_x_, grasp_y_, target_yaw_ * 180.0 / M_PI);

  // 4. 准备 tool_node client（arm_grasp 稍后在 move 到位后调用）
  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_sent_ = false;
    tool_request_done_ = false;
    tool_result_ = BT::NodeStatus::FAILURE;
    tool_message_.clear();
  }

  // 5. onStart: 发出 MoveToPose(grasp)
  align_state_ = ActionState::IDLE;
  align_message_.clear();
  align_goal_handle_.reset();
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
    std::lock_guard<std::mutex> lock(tool_mutex_);
    tool_request_done_ = true;
    tool_result_ = BT::NodeStatus::FAILURE;
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
    // Phase::START — 发出 MoveToPose(grasp)
    // =======================================================================
    case Phase::START:
    {
      phase_ = Phase::WAIT_ALIGN;
      RCLCPP_INFO(node_->get_logger(),
                  "[Fetch] >>> START MoveToPose(grasp): (%.3f,%.3f) yaw=%.1f°",
                  grasp_x_, grasp_y_, target_yaw_ * 180.0 / M_PI);
      sendMoveToPoseGoal();
      return BT::NodeStatus::RUNNING;
    }

    // =======================================================================
    // Phase::WAIT_ALIGN — 等 MoveToPose 完成
    // =======================================================================
    case Phase::WAIT_ALIGN:
    {
      failAlignIfTimedOut();
      if (align_state_ == ActionState::ACTIVE)
        return BT::NodeStatus::RUNNING;

      if (isAlignFailed())
      {
        setOutput("message", align_message_);
        config().blackboard->set("last_error", align_message_);
        config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
        RCLCPP_ERROR(node_->get_logger(), "[Fetch] align failed: %s", align_message_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      phase_ = Phase::START_TOOL;
      // fall through
    }
    [[fallthrough]];

    // =======================================================================
    // Phase::START_TOOL — 发出 tool_node arm_grasp（内嵌 service 调用）
    // =======================================================================
    case Phase::START_TOOL:
    {
      phase_ = Phase::WAIT_TOOL;
      RCLCPP_INFO(node_->get_logger(),
                  "[Fetch] >>> START tool_node arm_grasp: height_diff=%.0f", height_diff_);
      sendToolGrasp();
      return BT::NodeStatus::RUNNING;
    }

    // =======================================================================
    // Phase::WAIT_TOOL — 等 tool_node arm_grasp 完成
    // =======================================================================
    case Phase::WAIT_TOOL:
    {
      failToolIfTimedOut();

      bool done = false;
      {
        std::lock_guard<std::mutex> lock(tool_mutex_);
        done = tool_request_done_;
      }
      if (!done)
        return BT::NodeStatus::RUNNING;

      {
        std::lock_guard<std::mutex> lock(tool_mutex_);
        if (tool_result_ != BT::NodeStatus::SUCCESS)
        {
          setOutput("message", tool_message_);
          config().blackboard->set("last_error", tool_message_);
          config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
          RCLCPP_ERROR(node_->get_logger(), "[Fetch] tool_node arm_grasp failed: %s",
                       tool_message_.c_str());
          return BT::NodeStatus::FAILURE;
        }
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
      config().blackboard->set("meilin_suspension_offset", 0.0);
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      setOutput("message", std::string{"Fetch done"});

      RCLCPP_INFO(node_->get_logger(),
                  "[Fetch] DONE → yaw=%.2f  pose_is_center=false",
                  target_yaw_);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace r2_bt
