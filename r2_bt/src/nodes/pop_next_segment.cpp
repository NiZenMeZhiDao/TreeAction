#include "r2_bt/nodes/actions/pop_next_segment.hpp"

#include <cmath>
#include <mutex>

namespace r2_bt
{

PopNextSegment::PopNextSegment(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList PopNextSegment::providedPorts()
{
  return {
    BT::InputPort<SegmentQueuePtr>("queue", "Segment queue blackboard pointer"),
    BT::OutputPort<int>("current_segment_index", "Index of the popped segment"),
    BT::OutputPort<std::string>("segment_type", "Type of the popped segment (MOVE2, CLIMB, GRASP, ...)"),
    BT::OutputPort<std::string>("segment_debug_name", "Human-readable debug label for this segment"),
    BT::OutputPort<double>("target_x", "Move target X in map frame (m)"),
    BT::OutputPort<double>("target_y", "Move target Y in map frame (m)"),
    BT::OutputPort<double>("target_yaw", "Move target yaw in map frame (rad)"),
    BT::OutputPort<double>("timeout_sec", "Time limit for this segment (s)"),
    BT::OutputPort<int>("climb_mode", "Climb mode: 0=AUTO, 1=UP, 2=DOWN, 3=RECOVER"),
    BT::OutputPort<int>("climb_direction", "Climb direction: 0=FORWARD, 1=LEFT, 2=RIGHT, 3=BACKWARD"),
    BT::OutputPort<double>("climb_height", "Stair climb height (mm); 0=auto/default"),
    BT::OutputPort<uint8_t>("arm_command", "Arm action command (uint8)"),
    BT::OutputPort<bool>("wait_result", "Whether to block until arm action completes"),
    BT::OutputPort<uint8_t>("spear_command", "Spear action command (uint8)"),
    BT::OutputPort<double>("height_diff", "Height diff for GRASP suspension (mm). Absolute value."),
    BT::OutputPort<int>("grasp_suspension_mode", "Suspension mode for GRASP: 0=no-op, 1=UP, 2=DOWN"),
  };
}

BT::NodeStatus PopNextSegment::onStart()
{
  return tryPop();
}

BT::NodeStatus PopNextSegment::onRunning()
{
  return tryPop();
}

void PopNextSegment::onHalted()
{
}

BT::NodeStatus PopNextSegment::tryPop()
{
  SegmentQueuePtr queue;
  if (!getInput("queue", queue) || !queue)
  {
    config().blackboard->set("execution_state", std::string{"WAITING_PLAN"});
    config().blackboard->set("last_error", std::string{"Missing segment_queue"});
    return BT::NodeStatus::RUNNING;
  }

  std::unique_lock<std::mutex> lock(queue->mtx);
  if (queue->items.empty())
  {
    const auto state =
      config().blackboard->get<std::string>("execution_state");
    if (state != "WAITING_PLAN")
    {
      config().blackboard->set("execution_state", std::string{"MISSION_SUCCESS"});
    }
    return BT::NodeStatus::RUNNING;
  }

  const Segment segment = queue->items.front();
  queue->items.pop_front();
  lock.unlock();

  setOutput("current_segment_index", segment.index);
  setOutput("segment_type", segment.segment_type);
  setOutput("segment_debug_name", segment.debug_name);
  setOutput("target_x", segment.target_x);
  setOutput("target_y", segment.target_y);
  setOutput("target_yaw", segment.target_yaw);
  setOutput("timeout_sec", segment.timeout_sec);
  setOutput("climb_mode", segment.climb_mode);
  setOutput("climb_direction", segment.climb_direction);
  setOutput("climb_height", segment.climb_height);
  setOutput("arm_command", segment.arm_command);
  setOutput("wait_result", segment.wait_result);
  setOutput("spear_command", segment.spear_command);

  // GRASP 悬挂调整参数
  double abs_height_diff = std::abs(segment.height_diff);
  int grasp_suspension_mode = 0;
  if (segment.height_diff > 0.0) {
    grasp_suspension_mode = 1;  // UP
  } else if (segment.height_diff < 0.0) {
    grasp_suspension_mode = 2;  // DOWN
  }
  setOutput("height_diff", abs_height_diff);
  setOutput("grasp_suspension_mode", grasp_suspension_mode);

  config().blackboard->set("current_segment_index", segment.index);
  config().blackboard->set("segment_type", segment.segment_type);
  config().blackboard->set("segment_debug_name", segment.debug_name);
  config().blackboard->set("active_action", std::string{});
  config().blackboard->set("retry_count", 0);
  config().blackboard->set("last_error", std::string{});
  config().blackboard->set("execution_state", std::string{"SEGMENT_LOADED"});

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r2_bt
