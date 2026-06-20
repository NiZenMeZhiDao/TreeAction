#pragma once

#include <behaviortree_cpp/control_node.h>

namespace r2_bt
{

/**
 * @brief 根据 blackboard 中的 segment_type 动态调度到匹配的子节点。
 *
 * 用法:
 *   <SwitchSegmentType segment_type="{segment_type}">
 *     <Sequence name="MOVE2"> ... </Sequence>
 *     <Sequence name="GRASP"> ... </Sequence>
 *     <Sequence name="default"> ... </Sequence>   <!-- 可选的兜底分支 -->
 *   </SwitchSegmentType>
 *
 * 每个子节点通过 XML `name` 属性声明其匹配的 segment_type 字符串。
 * Name 为 "default" 的子节点会在无匹配时被 tick（兜底分支）。
 *
 * 执行逻辑:
 *   1. 读取 segment_type 端口
 *   2. 如果当前有 RUNNING 的子节点，继续 tick 它
 *   3. 否则遍历子节点，找到 name == segment_type 的第一个子节点并 tick
 *   4. 无匹配时 tick name=="default" 的子节点
 *   5. 完全无匹配返回 FAILURE
 */
class SwitchSegmentType : public BT::ControlNode
{
public:
  SwitchSegmentType(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  size_t current_child_idx_ = 0;
};

}  // namespace r2_bt
