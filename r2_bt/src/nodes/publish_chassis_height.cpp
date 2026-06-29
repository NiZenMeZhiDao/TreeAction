#include "r2_bt/nodes/actions/publish_chassis_height.hpp"

#include <array>
#include <cmath>

namespace r2_bt
{

PublishChassisHeight::PublishChassisHeight(const std::string& name,
                                           const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList PublishChassisHeight::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", "t0x0112_final",
                               "Float32MultiArray chassis height topic"),
    BT::InputPort<double>("height", "Uniform wheel height in mm"),
    BT::InputPort<double>("settle_sec", 0.2,
                          "Seconds to wait after publishing"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus PublishChassisHeight::onStart()
{
  if (!node_)
  {
    if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
    {
      setOutput("message", "Missing ros_node on blackboard");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto height = getInput<double>("height");
  if (!height || !std::isfinite(height.value()))
  {
    setOutput("message", "PublishChassisHeight missing finite height");
    return BT::NodeStatus::FAILURE;
  }

  const auto topic = getInput<std::string>("topic").value_or("t0x0112_final");
  settle_sec_ = getInput<double>("settle_sec").value_or(0.2);
  if (settle_sec_ < 0.0)
  {
    settle_sec_ = 0.0;
  }

  if (!publisher_ || publisher_topic_ != topic)
  {
    rclcpp::Publisher<Msg>::SharedPtr shared_publisher;
    std::string shared_topic;
    if (config().blackboard->get("final_chassis_height_publisher", shared_publisher) &&
        config().blackboard->get("final_chassis_height_topic", shared_topic) &&
        shared_publisher && shared_topic == topic)
    {
      publisher_ = shared_publisher;
      publisher_topic_ = shared_topic;
    }
    else
    {
      publisher_ = node_->create_publisher<Msg>(topic, 10);
      publisher_topic_ = topic;
    }
  }

  Msg msg;
  msg.data.assign(4, static_cast<float>(height.value()));
  publisher_->publish(msg);

  start_time_ = std::chrono::steady_clock::now();
  const auto text = "Published chassis height " + std::to_string(height.value()) +
                    "mm to " + publisher_topic_;
  setOutput("message", text);
  config().blackboard->set("active_action", std::string{"PublishChassisHeight"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  RCLCPP_INFO(node_->get_logger(),
              "[PublishChassisHeight] topic=%s height=%.1f settle=%.2f",
              publisher_topic_.c_str(), height.value(), settle_sec_);

  return settle_sec_ > 0.0 ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PublishChassisHeight::onRunning()
{
  const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed >= settle_sec_)
  {
    config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void PublishChassisHeight::onHalted()
{
}

}  // namespace r2_bt
