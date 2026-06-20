#include "r2_bt/nodes/actions/pop_next_meilin_segment.hpp"

#include <mutex>

namespace r2_bt
{

PopNextMeilinSegment::PopNextMeilinSegment(const std::string& name,
                                           const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList PopNextMeilinSegment::providedPorts()
{
  return {
    BT::InputPort<SegmentQueuePtr>("queue", "Meilin segment queue"),
    // 核心输出 — 只暴露 planner 原始值
    BT::OutputPort<std::string>("segment_type"),
    BT::OutputPort<int>("move_row"),
    BT::OutputPort<int>("move_col"),
    BT::OutputPort<double>("target_height_mm"),
    BT::OutputPort<int>("kfs_row"),
    BT::OutputPort<int>("kfs_col"),
    BT::OutputPort<double>("height_diff"),
    BT::OutputPort<double>("target_yaw"),
  };
}

BT::NodeStatus PopNextMeilinSegment::onStart()
{
  return tryPop();
}

BT::NodeStatus PopNextMeilinSegment::onRunning()
{
  return tryPop();
}

void PopNextMeilinSegment::onHalted()
{
}

BT::NodeStatus PopNextMeilinSegment::tryPop()
{
  SegmentQueuePtr queue;
  if (!getInput("queue", queue) || !queue)
  {
    config().blackboard->set("execution_state", std::string{"WAITING_MF_ACTION_SEQ"});
    config().blackboard->set("last_error", std::string{"Missing meilin segment_queue"});
    return BT::NodeStatus::RUNNING;
  }

  std::unique_lock<std::mutex> lock(queue->mtx);
  if (queue->items.empty())
  {
    config().blackboard->set("execution_state", std::string{"WAITING_MF_ACTION_SEQ"});
    return BT::NodeStatus::RUNNING;
  }

  const Segment segment = queue->items.front();
  queue->items.pop_front();
  lock.unlock();

  setOutput("segment_type", segment.segment_type);
  setOutput("move_row", segment.move_row);
  setOutput("move_col", segment.move_col);
  setOutput("target_height_mm", segment.target_height_mm);
  setOutput("kfs_row", segment.kfs_row);
  setOutput("kfs_col", segment.kfs_col);
  setOutput("height_diff", segment.height_diff);
  setOutput("target_yaw", segment.target_yaw);

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
