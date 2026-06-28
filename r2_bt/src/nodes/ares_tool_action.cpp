#include "r2_bt/nodes/actions/ares_tool_action.hpp"

namespace r2_bt
{

AresToolAction::AresToolAction(const std::string& name,
                               const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
{
}

BT::PortsList AresToolAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action", "grasp", "ARES tool action name"),
    BT::InputPort<std::string>("service_name", "/ares_tool_node/tool_action",
                               "ARES tool service name"),
    BT::InputPort<double>("timeout_sec", 30.0,
                          "Abort service wait after this many seconds"),
    BT::InputPort<double>("arg0", 0.0, "ARES tool arg[0]"),
    BT::InputPort<double>("arg1", 0.0, "ARES tool arg[1]"),
    BT::InputPort<double>("arg2", 0.0, "ARES tool arg[2]"),
    BT::InputPort<double>("arg3", 0.0, "ARES tool arg[3]"),
    BT::OutputPort<std::string>("message"),
  };
}

BT::NodeStatus AresToolAction::onStart()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    request_done_ = false;
    request_sent_ = false;
    result_status_ = BT::NodeStatus::FAILURE;
    result_message_.clear();
  }

  if (!config().blackboard->rootBlackboard()->get("ros_node", node_) || !node_)
  {
    result_message_ = "Missing ros_node on blackboard";
    setOutput("message", result_message_);
    return BT::NodeStatus::FAILURE;
  }

  const auto action = getInput<std::string>("action").value_or("grasp");
  service_name_ =
      getInput<std::string>("service_name").value_or("/ares_tool_node/tool_action");
  timeout_sec_ = getInput<double>("timeout_sec").value_or(30.0);

  if (!client_ || client_->get_service_name() != service_name_)
  {
    rclcpp::Client<ServiceT>::SharedPtr shared_client;
    std::string shared_client_name;
    if (config().blackboard->get("tool_action_client", shared_client) &&
        config().blackboard->get("tool_action_client_name", shared_client_name) &&
        shared_client && shared_client_name == service_name_)
    {
      client_ = shared_client;
      RCLCPP_DEBUG(node_->get_logger(),
                   "[AresToolAction] Reusing prewarmed service client: %s",
                   service_name_.c_str());
    }
    else
    {
      client_ = node_->create_client<ServiceT>(service_name_);
      RCLCPP_WARN(node_->get_logger(),
                  "[AresToolAction] Created local service client for %s; "
                  "prewarmed client was missing or name-mismatched",
                  service_name_.c_str());
    }
  }

  if (!client_->service_is_ready())
  {
    result_message_ = "AresToolAction service not available: " + service_name_;
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    RCLCPP_ERROR(node_->get_logger(), "[AresToolAction] %s",
                 result_message_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<ServiceT::Request>();
  request->action = action;
  request->args[0] = static_cast<float>(getInput<double>("arg0").value_or(0.0));
  request->args[1] = static_cast<float>(getInput<double>("arg1").value_or(0.0));
  request->args[2] = static_cast<float>(getInput<double>("arg2").value_or(0.0));
  request->args[3] = static_cast<float>(getInput<double>("arg3").value_or(0.0));

  start_time_ = std::chrono::steady_clock::now();
  config().blackboard->set("active_action", std::string{"AresToolAction:"} + action);
  config().blackboard->set("execution_state", std::string{"ACTION_RUNNING"});

  client_->async_send_request(
      request,
      [this](rclcpp::Client<ServiceT>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> lock(mutex_);
        request_done_ = true;
        result_message_ = response->message;
        result_status_ =
            (response->success && response->ret == 0) ? BT::NodeStatus::SUCCESS
                                                      : BT::NodeStatus::FAILURE;
      });

  {
    std::lock_guard<std::mutex> lock(mutex_);
    request_sent_ = true;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[AresToolAction] Request sent: service=%s action=%s timeout=%.1f",
              service_name_.c_str(), action.c_str(), timeout_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AresToolAction::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!request_sent_)
  {
    return BT::NodeStatus::RUNNING;
  }

  const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
  if (timeout_sec_ > 0.0 && elapsed > timeout_sec_)
  {
    result_message_ = "AresToolAction timed out";
    setOutput("message", result_message_);
    config().blackboard->set("last_error", result_message_);
    config().blackboard->set("execution_state", std::string{"ACTION_FAILED"});
    return BT::NodeStatus::FAILURE;
  }

  if (!request_done_)
  {
    return BT::NodeStatus::RUNNING;
  }

  setOutput("message", result_message_);
  config().blackboard->set(
      "execution_state",
      result_status_ == BT::NodeStatus::SUCCESS ? std::string{"ACTION_SUCCESS"}
                                                : std::string{"ACTION_FAILED"});
  if (result_status_ == BT::NodeStatus::FAILURE)
  {
    config().blackboard->set("last_error", result_message_);
  }
  return result_status_;
}

void AresToolAction::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  request_done_ = true;
  result_status_ = BT::NodeStatus::FAILURE;
  if (node_)
  {
    RCLCPP_INFO(node_->get_logger(), "[AresToolAction] Halted");
  }
}

}  // namespace r2_bt
