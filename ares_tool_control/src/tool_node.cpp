// ROS2 node exposing a single dispatch service that drives the ARES R2 tool.
//
// One textual `action` maps to one ARES SYNC command frame sent over USB to the
// MCU running app/ares_r2_tool. The service blocks until the MCU sends the
// matching feedback frame (action completed) or the timeout elapses.
//
// arm_grasp 高度差映射：
//   调用者通过 args[0] 传入原始高度差值。本节点只做简单映射：
//     height_diff > 0  → MCU args[0] = 200
//     height_diff < 0  → MCU args[0] = -200
//   悬挂逻辑由上层 Fetch 节点处理。
#include <cerrno>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "r2_interfaces/srv/tool_action.hpp"

#include "ares_tool_control/ares_usb.hpp"

namespace {

// action name -> {command id, feedback id, action enum}.
struct ActionSpec {
	uint16_t command_sync_id;
	uint16_t feedback_sync_id;
	uint32_t action;
};

const std::map<std::string, ActionSpec> kActionTable = {
	{"prepare", {ares::kSyncIdSpearCmd, ares::kSyncIdSpearDone, ares::kSpearPrepare}},
	{"grasp", {ares::kSyncIdSpearCmd, ares::kSyncIdSpearDone, ares::kSpearGrasp}},
	{"dock_extend",
	 {ares::kSyncIdSpearCmd, ares::kSyncIdSpearDone, ares::kSpearDockExtend}},
	{"arm_grasp", {ares::kSyncIdArmCmd, ares::kSyncIdArmDone, ares::kArmGrasp}},
	{"arm_get_body", {ares::kSyncIdArmCmd, ares::kSyncIdArmDone, ares::kArmGetBody}},
	{"arm_place_mid", {ares::kSyncIdArmCmd, ares::kSyncIdArmDone, ares::kArmPlaceMid}},
	{"arm_place_high",
	 {ares::kSyncIdArmCmd, ares::kSyncIdArmDone, ares::kArmPlaceHigh}},
};

// arm_grasp 高度差 → MCU 映射常量
constexpr double kHeightDiffUpMapped = 200.0;
constexpr double kHeightDiffDownMapped = -200.0;

std::string valid_actions()
{
	std::string list;
	for (const auto &entry : kActionTable) {
		if (!list.empty()) {
			list += ", ";
		}
		list += entry.first;
	}
	return list;
}

}  // namespace

using r2_interfaces::srv::ToolAction;
using r2_interfaces::action::SuspensionControl;

class AresToolNode : public rclcpp::Node {
      public:
	AresToolNode()
	    : Node("ares_tool_node"),
	      usb_(static_cast<uint16_t>(declare_parameter<int>("vid", ares::kUsbVid)),
		   static_cast<uint16_t>(declare_parameter<int>("pid", ares::kUsbPid)))
	{
		timeout_ms_ = declare_parameter<int>("completion_timeout_ms", 15000);
		try_open();

		service_ = create_service<ToolAction>(
			"~/tool_action",
			std::bind(&AresToolNode::handle, this, std::placeholders::_1,
				  std::placeholders::_2));

		RCLCPP_INFO(get_logger(), "ares_tool_node ready; service: %s; actions: %s",
			    service_->get_service_name(), valid_actions().c_str());
	}

      private:
	bool try_open()
	{
		const std::string err = usb_.open();
		if (err.empty()) {
			open_error_.clear();
			RCLCPP_INFO(get_logger(), "USB device opened");
			return true;
		}
		open_error_ = err;
		RCLCPP_WARN(get_logger(), "USB open failed (will retry on request): %s",
			    err.c_str());
		return false;
	}

	void handle(const std::shared_ptr<ToolAction::Request> req,
		    std::shared_ptr<ToolAction::Response> res)
	{
		const auto it = kActionTable.find(req->action);
		if (it == kActionTable.end()) {
			res->success = false;
			res->ret = -EINVAL;
			res->message = "unknown action '" + req->action +
				       "'; valid: " + valid_actions();
			RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
			return;
		}

		if (!usb_.is_open() && !try_open()) {
			res->success = false;
			res->ret = -EIO;
			res->message = "USB device not available: " + open_error_;
			RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
			return;
		}

		// ---- arm_grasp 高度差映射 ----
		// 上层传入原始 height_diff，本节点只做简单映射：
		//   正值 → 200，负值 → -200
		// 悬挂逻辑由上层 Fetch 节点处理。
		std::array<float, 4> args = req->args;

		if (req->action == "arm_grasp") {
			const double height_diff = static_cast<double>(req->args[0]);

			if (height_diff > 0.0) {
				args[0] = static_cast<float>(kHeightDiffUpMapped);
				RCLCPP_INFO(get_logger(),
					    "arm_grasp height_diff=+%.0f -> MCU args[0]=200",
					    height_diff);
			} else if (height_diff < 0.0) {
				args[0] = static_cast<float>(kHeightDiffDownMapped);
				RCLCPP_INFO(get_logger(),
					    "arm_grasp height_diff=%.0f -> MCU args[0]=-200",
					    height_diff);
			}
			// height_diff == 0: 透传
		}

		// ---- 发送 USB 命令 ----
		const ares::CommandResult r = usb_.send_command(
			it->second.command_sync_id, it->second.feedback_sync_id,
			it->second.action, args, timeout_ms_);
		res->ret = r.status;
		res->success = r.completed;

		if (r.completed) {
			res->message = "action '" + req->action + "' completed";
			RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
		} else if (r.status == -ETIMEDOUT) {
			res->message = "action '" + req->action +
				       "' sent but no completion within timeout";
			RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
		} else {
			usb_.close();
			res->message =
				"USB transfer failed for action '" + req->action + "'";
			RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
		}
	}

	ares::AresUsb usb_;
	int timeout_ms_;
	std::string open_error_;
	rclcpp::Service<ToolAction>::SharedPtr service_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AresToolNode>());
	rclcpp::shutdown();
	return 0;
}
