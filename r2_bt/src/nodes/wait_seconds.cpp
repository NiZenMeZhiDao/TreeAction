#include "r2_bt/nodes/actions/wait_seconds.hpp"

#include <rclcpp/rclcpp.hpp>

namespace r2_bt
{

WaitSeconds::WaitSeconds(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitSeconds::providedPorts()
{
  return {
    BT::InputPort<double>("seconds", 0.0, "Seconds to wait"),
  };
}

BT::NodeStatus WaitSeconds::onStart()
{
  seconds_ = getInput<double>("seconds").value_or(0.0);
  start_time_ = std::chrono::steady_clock::now();
  config().blackboard->set("active_action", std::string{"WaitSeconds"});
  if (seconds_ <= 0.0)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitSeconds::onRunning()
{
  const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
  return elapsed >= seconds_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void WaitSeconds::onHalted()
{
}

}  // namespace r2_bt
