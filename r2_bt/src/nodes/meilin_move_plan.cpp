#include "r2_bt/nodes/actions/meilin_move_plan.hpp"

#include <r2_interfaces/action/step_motion_control.hpp>

#include <cmath>
#include <string>

#include "r2_bt/segment.hpp"

namespace r2_bt
{

MeilinPlanMove::MeilinPlanMove(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList MeilinPlanMove::providedPorts()
{
  return {
    BT::InputPort<int>("move_row"),
    BT::InputPort<int>("move_col"),
    BT::InputPort<double>("target_height_mm"),
    BT::InputPort<double>("target_yaw"),
    BT::OutputPort<double>("target_x"),
    BT::OutputPort<double>("target_y"),
    BT::OutputPort<double>("move_target_yaw"),
    BT::OutputPort<double>("align_timeout_sec"),
    BT::OutputPort<bool>("entry_move"),
    BT::OutputPort<bool>("step_needed"),
    BT::OutputPort<bool>("center_motion_needed"),
    BT::OutputPort<int>("step_mode"),
    BT::OutputPort<int>("step_direction"),
    BT::OutputPort<double>("step_height"),
    BT::OutputPort<double>("step_correction_x"),
    BT::OutputPort<double>("step_correction_y"),
    BT::OutputPort<double>("step_correction_yaw_deg"),
    BT::OutputPort<double>("step_timeout_sec"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus MeilinPlanMove::tick()
{
  if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
  {
    setOutput("message", std::string{"Missing ros_node on blackboard"});
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

  const auto move_row = getInput<int>("move_row");
  const auto move_col = getInput<int>("move_col");
  const auto target_yaw = getInput<double>("target_yaw");
  if (!move_row || !move_col || !target_yaw)
  {
    const std::string err = "MeilinPlanMove missing move_row/move_col/target_yaw";
    setOutput("message", err);
    config().blackboard->set("last_error", err);
    return BT::NodeStatus::FAILURE;
  }

  const int to_row = move_row.value();
  const int to_col = move_col.value();
  const double yaw = target_yaw.value();
  const double target_height = getInput<double>("target_height_mm").value_or(
      meilin_height_at(to_row, to_col, cfg->side));
  const auto [target_x, target_y] = meilin_grid_to_world(to_row, to_col, *cfg);

  int from_row = config().blackboard->get<int>("meilin_current_row");
  int from_col = config().blackboard->get<int>("meilin_current_col");
  double current_height = config().blackboard->get<double>("meilin_current_height");
  double current_yaw = config().blackboard->get<double>("meilin_current_yaw");
  bool pose_is_center = config().blackboard->get<bool>("meilin_pose_is_cell_center");
  const bool entry_move = config().blackboard->get<bool>("meilin_entry_move_pending");

  bool pose_fresh = false;
  double pose_x = 0.0;
  double pose_y = 0.0;
  const bool pose_received = config().blackboard->get<bool>("meilin_pose_received");
  if (pose_received)
  {
    const double last_update_sec =
        config().blackboard->get<double>("meilin_pose_last_update_sec");
    const double pose_age_sec = node_->now().seconds() - last_update_sec;
    pose_fresh = cfg->pose_timeout_sec <= 0.0 || pose_age_sec <= cfg->pose_timeout_sec;
    if (pose_fresh)
    {
      pose_x = config().blackboard->get<double>("meilin_pose_x");
      pose_y = config().blackboard->get<double>("meilin_pose_y");
      current_yaw = config().blackboard->get<double>("meilin_pose_yaw");

      int pose_row = 0;
      int pose_col = 0;
      if (meilin_world_to_grid(pose_x, pose_y, *cfg, pose_row, pose_col))
      {
        const auto [center_x, center_y] = meilin_grid_to_world(pose_row, pose_col, *cfg);
        const double center_dist = std::hypot(pose_x - center_x, pose_y - center_y);
        if (!entry_move)
        {
          from_row = pose_row;
          from_col = pose_col;
          current_height = meilin_height_at(from_row, from_col, cfg->side);
          pose_is_center = center_dist <= cfg->cell_center_tolerance;
          config().blackboard->set("meilin_current_row", from_row);
          config().blackboard->set("meilin_current_col", from_col);
          config().blackboard->set("meilin_current_height", current_height);
          config().blackboard->set("meilin_pose_is_cell_center", pose_is_center);
        }
      }
      else if (!entry_move)
      {
        const std::string err = "Current relocation pose is outside Meilin grid";
        setOutput("message", err);
        config().blackboard->set("last_error", err);
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  config().blackboard->set("meilin_current_yaw", current_yaw);

  const double height_diff = target_height - current_height;
  const bool same_grid = from_row == to_row && from_col == to_col;
  const bool step_needed =
      !entry_move && !same_grid && std::abs(height_diff) > cfg->height_tolerance;
  const bool center_motion_needed = !step_needed;

  int step_mode = 0;
  int step_direction = 0;
  double step_height = 0.0;
  double correction_x = 0.0;
  double correction_y = 0.0;
  const double reference_yaw = meilin_snap_cardinal_yaw(yaw);
  double correction_yaw_deg = reference_yaw * 180.0 / M_PI;

  if (step_needed)
  {
    if (!meilin_adjacent(from_row, from_col, to_row, to_col))
    {
      const std::string err = "Move with height change requires adjacent target grid";
      setOutput("message", err);
      config().blackboard->set("last_error", err);
      return BT::NodeStatus::FAILURE;
    }

    double move_yaw = 0.0;
    std::string dir_name;
    if (!meilin_direction_yaw(from_row, from_col, to_row, to_col,
                              move_yaw, step_direction, dir_name,
                              reference_yaw))
    {
      const std::string err =
          "Move step direction is invalid: from=(" +
          std::to_string(from_row) + "," + std::to_string(from_col) +
          ") to=(" + std::to_string(to_row) + "," +
          std::to_string(to_col) + ") target_yaw=" +
          std::to_string(yaw) + " reference_yaw=" +
          std::to_string(reference_yaw) + " pose_yaw=" +
          std::to_string(current_yaw);
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

    step_mode = height_diff > 0.0
                    ? r2_interfaces::action::StepMotionControl::Goal::MODE_CLIMB_UP
                    : r2_interfaces::action::StepMotionControl::Goal::MODE_CLIMB_DOWN;
    step_height = std::abs(height_diff);

    const auto [cx, cy] = meilin_grid_to_world(from_row, from_col, *cfg);
    correction_x = cx;
    correction_y = cy;
  }

  setOutput("target_x", target_x);
  setOutput("target_y", target_y);
  setOutput("move_target_yaw", yaw);
  setOutput("align_timeout_sec", cfg->align_timeout_sec);
  setOutput("entry_move", entry_move);
  setOutput("step_needed", step_needed);
  setOutput("center_motion_needed", center_motion_needed);
  setOutput("step_mode", step_mode);
  setOutput("step_direction", step_direction);
  setOutput("step_height", step_height);
  setOutput("step_correction_x", correction_x);
  setOutput("step_correction_y", correction_y);
  setOutput("step_correction_yaw_deg", correction_yaw_deg);
  setOutput("step_timeout_sec", cfg->suspension_timeout_sec);
  setOutput("message", std::string{});

  config().blackboard->set("meilin_planned_move_row", to_row);
  config().blackboard->set("meilin_planned_move_col", to_col);
  config().blackboard->set("meilin_planned_target_height", target_height);
  config().blackboard->set("meilin_planned_target_yaw", yaw);
  config().blackboard->set("meilin_planned_entry_move", entry_move);
  config().blackboard->set("active_action", std::string{"MeilinPlanMove"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});

  RCLCPP_INFO(node_->get_logger(),
              "[MeilinPlanMove] from=(%d,%d h=%.0f) to=(%d,%d h=%.0f) "
              "entry=%s step=%s center_motion=%s target_yaw=%.3f "
              "reference_yaw=%.3f pose_yaw=%.3f",
              from_row, from_col, current_height, to_row, to_col, target_height,
              entry_move ? "yes" : "no", step_needed ? "yes" : "no",
              center_motion_needed ? "yes" : "no", yaw, reference_yaw,
              current_yaw);

  return BT::NodeStatus::SUCCESS;
}

MeilinCommitMove::MeilinCommitMove(const std::string& name,
                                   const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList MeilinCommitMove::providedPorts()
{
  return {
    BT::InputPort<int>("move_row"),
    BT::InputPort<int>("move_col"),
    BT::InputPort<double>("target_height_mm"),
    BT::InputPort<double>("target_yaw"),
    BT::InputPort<bool>("entry_move"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus MeilinCommitMove::tick()
{
  const auto cfg = config().blackboard->get<MeilinConfigPtr>("meilin_config");
  if (!cfg)
  {
    const std::string err = "Missing meilin_config on blackboard";
    setOutput("message", err);
    config().blackboard->set("last_error", err);
    return BT::NodeStatus::FAILURE;
  }

  const int row = getInput<int>("move_row").value_or(
      config().blackboard->get<int>("meilin_planned_move_row"));
  const int col = getInput<int>("move_col").value_or(
      config().blackboard->get<int>("meilin_planned_move_col"));
  const double height = getInput<double>("target_height_mm").value_or(
      meilin_height_at(row, col, cfg->side));
  const double yaw = getInput<double>("target_yaw").value_or(
      config().blackboard->get<double>("meilin_planned_target_yaw"));
  const bool entry_move = getInput<bool>("entry_move").value_or(
      config().blackboard->get<bool>("meilin_planned_entry_move"));

  config().blackboard->set("meilin_current_row", row);
  config().blackboard->set("meilin_current_col", col);
  config().blackboard->set("meilin_current_height", height);
  config().blackboard->set("meilin_current_yaw", yaw);
  config().blackboard->set("meilin_pose_is_cell_center", true);
  config().blackboard->set("meilin_suspension_offset", 0.0);
  if (entry_move)
  {
    config().blackboard->set("meilin_entry_move_pending", false);
  }

  setOutput("message", std::string{"Move done"});
  config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
  return BT::NodeStatus::SUCCESS;
}

}  // namespace r2_bt
