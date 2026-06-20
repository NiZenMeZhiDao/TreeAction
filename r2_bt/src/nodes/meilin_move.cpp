#include "r2_bt/nodes/actions/meilin_move.hpp"

#include <cmath>
#include <string>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

// ===========================================================================
// 模拟 action 辅助（真实实现会替换为 async action client）
// ===========================================================================

void MeilinMove::startAction(const std::string& name)
{
  // 模拟: 标记为 ACTIVE，下一 tick 自动变为 DONE
  if (name == "pre_align")
    pre_align_state_ = ActionState::ACTIVE;
  else if (name == "climb")
    climb_state_ = ActionState::ACTIVE;
  else if (name == "target_align")
    target_align_state_ = ActionState::ACTIVE;
}

bool MeilinMove::isActionDone(const std::string& name) const
{
  if (name == "pre_align")    return pre_align_state_ == ActionState::DONE;
  if (name == "climb")        return climb_state_ == ActionState::DONE;
  if (name == "target_align") return target_align_state_ == ActionState::DONE;
  return true;
}

bool MeilinMove::allActionsDone() const
{
  // 检查所有已触发的 action 是否都已完成
  if (pre_align_state_ == ActionState::ACTIVE)    return false;
  if (climb_state_ == ActionState::ACTIVE)        return false;
  if (target_align_state_ == ActionState::ACTIVE) return false;
  return true;
}

void MeilinMove::resetActions()
{
  pre_align_state_    = ActionState::IDLE;
  climb_state_        = ActionState::IDLE;
  target_align_state_ = ActionState::IDLE;
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
  //    TODO: 后续改为直接从 odometry topic 读取定位坐标
  // =========================================================================
  from_row_    = config().blackboard->get<int>("meilin_current_row");
  from_col_    = config().blackboard->get<int>("meilin_current_col");
  const double current_height =
      config().blackboard->get<double>("meilin_current_height");
  const double current_yaw =
      config().blackboard->get<double>("meilin_current_yaw");
  const bool pose_is_center =
      config().blackboard->get<bool>("meilin_pose_is_cell_center");

  RCLCPP_INFO(node_->get_logger(),
              "[Move] from (%d,%d) h=%.0f → to (%d,%d) h=%.0f  yaw=%.2f  center=%s",
              from_row_, from_col_, current_height,
              move_row_, move_col_, target_height_mm_, target_yaw_,
              pose_is_center ? "yes" : "no");

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
  // =========================================================================
  const double height_diff = target_height_mm_ - current_height;
  climb_needed_ = std::abs(height_diff) > cfg->height_tolerance;
  if (climb_needed_)
  {
    climb_mode_     = (height_diff > 0.0) ? 1 : 2;
    climb_height_mm_ = std::abs(height_diff);
    double unused_yaw = 0.0;
    std::string dir_name;
    meilin_direction_yaw(from_row_, from_col_, move_row_, move_col_,
                         unused_yaw, climb_direction_, dir_name);
    RCLCPP_INFO(node_->get_logger(),
                "[Move] climb: mode=%s dir=%s %.0fmm",
                climb_mode_ == 1 ? "UP" : "DOWN",
                dir_name.c_str(), climb_height_mm_);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(),
                "[Move] no climb (diff=%.0f ≤ tol=%.0f)", height_diff, cfg->height_tolerance);
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
  // TODO: 真实实现时 cancel 所有进行中的 action goal
}

// ===========================================================================
// 核心状态机
// ===========================================================================

BT::NodeStatus MeilinMove::drive()
{
  // ---- 模拟: 将所有 ACTIVE 标记推进为 DONE ----
  if (pre_align_state_    == ActionState::ACTIVE) pre_align_state_    = ActionState::DONE;
  if (climb_state_        == ActionState::ACTIVE) climb_state_        = ActionState::DONE;
  if (target_align_state_ == ActionState::ACTIVE) target_align_state_ = ActionState::DONE;

  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");

  switch (phase_)
  {
    // =======================================================================
    // Phase::START — 并行发出 pre_align + climb
    // =======================================================================
    case Phase::START:
    {
      phase_ = Phase::WAIT_PHASE1;

      if (pre_align_needed_)
      {
        RCLCPP_INFO(node_->get_logger(),
                    "[Move] >>> START Align(pre_align): (%.3f,%.3f) yaw=%.1f°",
                    pre_align_x_, pre_align_y_, pre_align_yaw_ * 180.0 / M_PI);
        // TODO: align_client_->async_send_goal(pre_align_x_, pre_align_y_, pre_align_yaw_)
        startAction("pre_align");
      }

      if (climb_needed_)
      {
        const double t = cfg ? cfg->suspension_timeout_sec : 10.0;
        RCLCPP_INFO(node_->get_logger(),
                    "[Move] >>> START Suspension(mode=%d,dir=%d,h=%.0fmm,timeout=%.1fs)",
                    climb_mode_, climb_direction_, climb_height_mm_, t);
        // TODO: suspension_client_->async_send_goal(climb_mode_, climb_direction_, climb_height_mm_)
        startAction("climb");
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
      if (pre_align_needed_ && !isActionDone("pre_align"))
        return BT::NodeStatus::RUNNING;

      // pre_align 完成（或不需要），进入 Phase2: 发 target_align
      // climb 如果还在进行中会继续并行
      phase_ = Phase::WAIT_PHASE2;

      {
        const double t = cfg ? cfg->align_timeout_sec : 30.0;
        RCLCPP_INFO(node_->get_logger(),
                    "[Move] >>> START Align(target): (%.3f,%.3f) yaw=%.1f° timeout=%.1fs",
                    target_x_, target_y_, target_yaw_ * 180.0 / M_PI, t);
        // TODO: align_client_->async_send_goal(target_x_, target_y_, target_yaw_)
        startAction("target_align");
      }
      phase2_fired_ = true;
      return BT::NodeStatus::RUNNING;
    }

    // =======================================================================
    // Phase::WAIT_PHASE2 — 等 target_align 和 climb 都完成
    // =======================================================================
    case Phase::WAIT_PHASE2:
    {
      if (!allActionsDone())
        return BT::NodeStatus::RUNNING;

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
