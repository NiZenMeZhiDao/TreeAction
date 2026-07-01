#include "r2_bt/nodes/actions/select_final_target.hpp"

#include <sstream>

namespace r2_bt
{

SelectFinalTarget::SelectFinalTarget(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList SelectFinalTarget::providedPorts()
{
  return {
    BT::InputPort<int>("command", "Deck command value 1..6"),
    BT::OutputPort<int>("layer", "Final area layer, 2 or 3"),
    BT::OutputPort<std::string>("column", "left/mid/right"),
    BT::OutputPort<std::string>("flow", "LAYER2 or LAYER3"),
    BT::OutputPort<double>("target_x", "MoveToPose target x"),
    BT::OutputPort<double>("target_y", "MoveToPose target y"),
    BT::OutputPort<double>("target_yaw", "MoveToPose target yaw"),
    BT::OutputPort<int>("pid_profile", "MoveToPose PID profile"),
    BT::OutputPort<double>("max_vel", "MoveToPose max linear speed override"),
    BT::OutputPort<double>("max_wz", "MoveToPose max yaw angular speed override"),
    BT::OutputPort<double>("timeout_sec", "MoveToPose timeout"),
    BT::OutputPort<std::string>("message", "Status or error message"),
  };
}

BT::NodeStatus SelectFinalTarget::tick()
{
  const auto command = getInput<int>("command");
  if (!command)
  {
    setOutput("message", "Missing final command");
    return BT::NodeStatus::FAILURE;
  }

  int layer = 0;
  std::string column;
  switch (command.value())
  {
    case 1: layer = 3; column = "left"; break;
    case 2: layer = 3; column = "mid"; break;
    case 3: layer = 3; column = "right"; break;
    case 4: layer = 2; column = "left"; break;
    case 5: layer = 2; column = "mid"; break;
    case 6: layer = 2; column = "right"; break;
    default:
      setOutput("message", "Unsupported final command");
      return BT::NodeStatus::FAILURE;
  }

  const std::string prefix =
      "final_target_" + std::to_string(layer) + "_" + column + "_";
  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;
  double max_vel = 0.0;
  double max_wz = 0.0;
  double timeout_sec = 0.0;
  int pid_profile = 0;
  std::string error;

  if (!read_target_value(prefix + "target_x", target_x, error) ||
      !read_target_value(prefix + "target_y", target_y, error) ||
      !read_target_value(prefix + "target_yaw", target_yaw, error) ||
      !read_target_value(prefix + "pid_profile", pid_profile, error) ||
      !read_target_value(prefix + "timeout_sec", timeout_sec, error))
  {
    setOutput("message", error);
    config().blackboard->set("last_error", error);
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get(prefix + "max_vel", max_vel))
  {
    max_vel = 0.0;
  }
  if (!config().blackboard->get(prefix + "max_wz", max_wz))
  {
    max_wz = 0.0;
  }

  setOutput("layer", layer);
  setOutput("column", column);
  setOutput("flow", layer == 2 ? std::string{"LAYER2"} : std::string{"LAYER3"});
  setOutput("target_x", target_x);
  setOutput("target_y", target_y);
  setOutput("target_yaw", target_yaw);
  setOutput("pid_profile", pid_profile);
  setOutput("max_vel", max_vel);
  setOutput("max_wz", max_wz);
  setOutput("timeout_sec", timeout_sec);

  std::ostringstream oss;
  oss << "Selected final target command=" << command.value()
      << " layer=" << layer << " column=" << column;
  setOutput("message", oss.str());
  config().blackboard->set("active_action", std::string{"SelectFinalTarget"});
  config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
  return BT::NodeStatus::SUCCESS;
}

template <typename T>
bool SelectFinalTarget::read_target_value(const std::string& key,
                                          T& value,
                                          std::string& error) const
{
  if (config().blackboard->get(key, value))
  {
    return true;
  }
  error = "Missing final target config: " + key;
  return false;
}

template bool SelectFinalTarget::read_target_value<double>(
    const std::string&, double&, std::string&) const;
template bool SelectFinalTarget::read_target_value<int>(
    const std::string&, int&, std::string&) const;

}  // namespace r2_bt
