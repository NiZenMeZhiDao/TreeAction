#pragma once

#include <behaviortree_cpp/decorator_node.h>

namespace r2_bt
{

/**
 * @brief 循环执行子节点，直到 segment_type 匹配停止条件。
 *
 * 用于梅林区等变长 segment 队列的动态循环:
 *   - 子节点每轮负责弹出一个 segment 并执行对应动作
 *   - 子节点返回 SUCCESS 后，若 segment_type == stop_type 则停止循环
 *   - 否则 reset 子节点并返回 RUNNING，下一个 tick 继续下一轮
 *   - 子节点返回 FAILURE 则立即停止并传播失败
 *   - 子节点返回 RUNNING 则等待
 *
 * 用法:
 *   <ForEachSegment stop_type="EXIT">
 *     <Sequence>
 *       <PopNextSegment .../>
 *       <SwitchSegmentType .../>
 *     </Sequence>
 *   </ForEachSegment>
 */
class ForEachSegment : public BT::DecoratorNode
{
public:
  ForEachSegment(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool first_tick_ = true;
};

}  // namespace r2_bt
