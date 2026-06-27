#include "r2_bt/nodes/actions/wait_for_int_signal.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <sstream>

namespace r2_bt
{

WaitForIntSignal::WaitForIntSignal(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitForIntSignal::providedPorts()
{
  return {
    BT::InputPort<std::string>("signal_topic", "/aruco_comm/tx_id",
                               "Topic name for std_msgs/Int32 deck command"),
    BT::InputPort<int>("expected_value", -1,
                       "Single expected value. Ignored when expected_values is set"),
    BT::InputPort<std::string>("expected_values", "",
                               "Comma separated expected values, e.g. 1,2,3"),
    BT::InputPort<double>("timeout_sec", 0.0,
                          "Max wait time before failure. 0 = no timeout"),
    BT::OutputPort<int>("received_value", "Matched Int32 value"),
    BT::OutputPort<std::string>("message", "Status or error message"),
  };
}

BT::NodeStatus WaitForIntSignal::onStart()
{
  if (!node_)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
    if (!node_)
    {
      setOutput("message", "Missing ros_node on blackboard");
      return BT::NodeStatus::FAILURE;
    }
  }

  expected_values_ = parse_expected_values();
  if (expected_values_.empty())
  {
    setOutput("message", "WaitForIntSignal has no expected value");
    return BT::NodeStatus::FAILURE;
  }

  if (!configure_subscription())
  {
    return BT::NodeStatus::FAILURE;
  }

  timeout_sec_ = getInput<double>("timeout_sec").value_or(0.0);
  start_time_ = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    start_sequence_ = message_sequence_;
    matched_sequence_ = 0;
    matched_value_ = 0;
  }

  config().blackboard->set("active_action", std::string{"WaitForIntSignal"});
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});
  RCLCPP_INFO(node_->get_logger(), "[WaitForIntSignal] waiting on %s",
              subscribed_topic_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForIntSignal::onRunning()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (matched_sequence_ > start_sequence_)
    {
      setOutput("received_value", matched_value_);
      setOutput("message", "Signal received");
      config().blackboard->set("execution_state", std::string{"ACTION_SUCCESS"});
      RCLCPP_INFO(node_->get_logger(), "[WaitForIntSignal] received %d", matched_value_);
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (timeout_sec_ > 0.0)
  {
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed > timeout_sec_)
    {
      std::ostringstream oss;
      oss << "WaitForIntSignal timed out after " << timeout_sec_ << "s";
      const auto message = oss.str();
      setOutput("message", message);
      config().blackboard->set("last_error", message);
      config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
      RCLCPP_WARN(node_->get_logger(), "[WaitForIntSignal] %s", message.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForIntSignal::onHalted()
{
}

void WaitForIntSignal::signal_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++message_sequence_;
  last_value_ = msg->data;
  if (expected(last_value_))
  {
    matched_sequence_ = message_sequence_;
    matched_value_ = last_value_;
  }
}

bool WaitForIntSignal::expected(int value) const
{
  return expected_values_.find(value) != expected_values_.end();
}

bool WaitForIntSignal::configure_subscription()
{
  const auto topic = getInput<std::string>("signal_topic").value_or("/aruco_comm/tx_id");
  if (sub_ && topic == subscribed_topic_)
  {
    return true;
  }

  subscribed_topic_ = topic;
  sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      subscribed_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&WaitForIntSignal::signal_callback, this, std::placeholders::_1));
  return static_cast<bool>(sub_);
}

std::set<int> WaitForIntSignal::parse_expected_values() const
{
  std::set<int> values;
  const auto csv = getInput<std::string>("expected_values").value_or("");
  if (!csv.empty())
  {
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ','))
    {
      item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
      if (!item.empty())
      {
        try
        {
          values.insert(std::stoi(item));
        }
        catch (const std::exception&)
        {
          values.clear();
          return values;
        }
      }
    }
    return values;
  }

  const int single_value = getInput<int>("expected_value").value_or(-1);
  if (single_value >= 0)
  {
    values.insert(single_value);
  }
  return values;
}

}  // namespace r2_bt
