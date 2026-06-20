#include "r2_bt/nodes/actions/meilin_fetch.hpp"

#include <cmath>
#include <string>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

// ===========================================================================
// 模拟 action 辅助
// ===========================================================================

void MeilinFetch::startAction(const std::string& name)
{
  if (name == "align")          align_state_      = ActionState::ACTIVE;
  else if (name == "suspension") suspension_state_ = ActionState::ACTIVE;
  else if (name == "arm")       arm_state_        = ActionState::ACTIVE;
}

bool MeilinFetch::isActionDone(const std::string& name) const
{
  if (name == "align")      return align_state_      == ActionState::DONE;
  if (name == "suspension") return suspension_state_ == ActionState::DONE;
  if (name == "arm")        return arm_state_        == ActionState::DONE;
  return true;
}

bool MeilinFetch::allActionsDone() const
{
  if (align_state_      == ActionState::ACTIVE) return false;
  if (suspension_state_ == ActionState::ACTIVE) return false;
  if (arm_state_        == ActionState::ACTIVE) return false;
  return true;
}

void MeilinFetch::resetActions()
{
  align_state_      = ActionState::IDLE;
  suspension_state_ = ActionState::IDLE;
  arm_state_        = ActionState::IDLE;
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
  // 4. 悬挂模式推导
  // =========================================================================
  abs_height_diff_ = std::abs(height_diff_);
  if (abs_height_diff_ > cfg->height_tolerance)
  {
    suspension_mode_ = (height_diff_ > 0.0) ? 1 : 2;
    RCLCPP_INFO(node_->get_logger(),
                "[Fetch] suspension: mode=%s %.0fmm",
                suspension_mode_ == 1 ? "UP" : "DOWN", abs_height_diff_);
  }
  else
  {
    suspension_mode_ = 0;
    RCLCPP_INFO(node_->get_logger(), "[Fetch] no suspension needed");
  }

  // =========================================================================
  // 5. onStart 直接发出第一批 action: Align(grasp) + Suspension(并行)
  // =========================================================================
  resetActions();
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
  // TODO: 真实实现时 cancel 所有进行中的 action goal
}

// ===========================================================================
// 核心状态机
// ===========================================================================

BT::NodeStatus MeilinFetch::drive()
{
  // ---- 模拟: 将所有 ACTIVE 标记推进为 DONE ----
  if (align_state_      == ActionState::ACTIVE) align_state_      = ActionState::DONE;
  if (suspension_state_ == ActionState::ACTIVE) suspension_state_ = ActionState::DONE;
  if (arm_state_        == ActionState::ACTIVE) arm_state_        = ActionState::DONE;

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");

  switch (phase_)
  {
    // =======================================================================
    // Phase::START — 并行发出 Align(grasp) + Suspension
    // =======================================================================
    case Phase::START:
    {
      phase_ = Phase::WAIT_PHASE1;

      {
        const double t = cfg ? cfg->align_timeout_sec : 30.0;
        RCLCPP_INFO(node_->get_logger(),
                    "[Fetch] >>> START Align(grasp): (%.3f,%.3f) yaw=%.1f° timeout=%.1fs",
                    grasp_x_, grasp_y_, target_yaw_ * 180.0 / M_PI, t);
        // TODO: align_client_->async_send_goal(grasp_x_, grasp_y_, target_yaw_)
        startAction("align");
      }

      if (suspension_mode_ != 0)
      {
        const double t = cfg ? cfg->suspension_timeout_sec : 10.0;
        RCLCPP_INFO(node_->get_logger(),
                    "[Fetch] >>> START Suspension(mode=%d,h=%.0fmm,timeout=%.1fs) — parallel",
                    suspension_mode_, abs_height_diff_, t);
        // TODO: suspension_client_->async_send_goal(suspension_mode_, abs_height_diff_)
        startAction("suspension");
      }

      return BT::NodeStatus::RUNNING;
    }

    // =======================================================================
    // Phase::WAIT_PHASE1 — 等 Align 和 Suspension 都完成
    // =======================================================================
    case Phase::WAIT_PHASE1:
    {
      if (!allActionsDone())
        return BT::NodeStatus::RUNNING;

      // 两者都完成，进入 Arm 阶段
      phase_ = Phase::START_ARM;
      // fall through
    }
    [[fallthrough]];

    // =======================================================================
    // Phase::START_ARM — 串行: Align 完成后才发 ArmAction（不能边动边抓）
    // =======================================================================
    case Phase::START_ARM:
    {
      phase_ = Phase::WAIT_ARM;

      {
        const double t = cfg ? cfg->arm_timeout_sec : 30.0;
        RCLCPP_INFO(node_->get_logger(),
                    "[Fetch] >>> START ArmAction(GRASP) timeout=%.1fs", t);
        // TODO: arm_client_->async_send_goal(CMD_GRASP, wait_result=true)
        startAction("arm");
      }

      return BT::NodeStatus::RUNNING;
    }

    // =======================================================================
    // Phase::WAIT_ARM — 等 ArmAction 完成
    // =======================================================================
    case Phase::WAIT_ARM:
    {
      if (!isActionDone("arm"))
        return BT::NodeStatus::RUNNING;

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
