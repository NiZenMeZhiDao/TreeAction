#include "r2_bt/nodes/actions/place_object_placeholder.hpp"

#include <rclcpp/rclcpp.hpp>

namespace r2_bt
{

PlaceObjectPlaceholder::PlaceObjectPlaceholder(const std::string& name,
                                               const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList PlaceObjectPlaceholder::providedPorts()
{
  return {
    BT::InputPort<int>("layer", "Final area layer"),
    BT::InputPort<std::string>("column", "left/mid/right"),
    BT::InputPort<int>("place_index", 1, "Placement index within this command"),
    BT::OutputPort<std::string>("message", "Status message"),
  };
}

BT::NodeStatus PlaceObjectPlaceholder::tick()
{
  const int layer = getInput<int>("layer").value_or(0);
  const auto column = getInput<std::string>("column").value_or("unknown");
  const int place_index = getInput<int>("place_index").value_or(1);
  const std::string message =
      "Placeholder place: layer=" + std::to_string(layer) +
      " column=" + column +
      " index=" + std::to_string(place_index);

  setOutput("message", message);
  config().blackboard->set("active_action", std::string{"PlaceObjectPlaceholder"});
  config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});

  rclcpp::Node::SharedPtr node;
  if (config().blackboard->rootBlackboard()->get("ros_node", node) && node)
  {
    RCLCPP_INFO(node->get_logger(), "[PlaceObjectPlaceholder] %s", message.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace r2_bt
