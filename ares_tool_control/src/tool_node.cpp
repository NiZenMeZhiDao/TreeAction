// ROS2 node exposing a single dispatch service that drives the ARES R2 tool.
//
// One textual `action` maps to one ARES SYNC command frame sent over USB to the
// MCU running app/ares_r2_tool. The service blocks until the MCU sends the
// matching feedback frame (action completed) or the timeout elapses.
//
// arm_grasp 高度差联动：
//   调用者通过 args[0] 传入原始高度差值（来自 BT 层 PopNextMeilinSegment
//   输出的 height_diff）。本节点负责映射：正值(200/400) → MCU args[0]=200，
//   负值 → MCU args[0]=-200。若原始值为 400，则通过 suspension_control
//   action 先将整车抬升至 200 mm，抓取完成后再复位到 30 mm 正常行驶高度。
#include <cerrno>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

<<<<<<< Updated upstream
#include "r2_interfaces/srv/tool_action.hpp"
#include "r2_interfaces/action/suspension_control.hpp"
=======
#include "ares_tool_interfaces/srv/tool_action.hpp"
>>>>>>> Stashed changes

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

// arm_grasp 使用的高度差映射常量 (mm)
constexpr double kHeightDiffThreshold = 1.0;
constexpr double kHeightDiffUpMapped = 200.0;
constexpr double kHeightDiffDownMapped = -200.0;
constexpr double kHeightDiff400Suspension = 400.0;
constexpr double kSuspensionLiftHeight = 200.0;      // 高度差 400 时抬升到的绝对高度
constexpr double kSuspensionNormalHeight = 30.0;      // 复位到的正常行驶高度
constexpr double kSuspensionTimeoutSec = 10.0;

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

<<<<<<< Updated upstream
using r2_interfaces::srv::ToolAction;
using r2_interfaces::action::SuspensionControl;
=======
using ares_tool_interfaces::srv::ToolAction;
>>>>>>> Stashed changes

class AresToolNode : public rclcpp::Node {
      public:
	AresToolNode()
	    : Node("ares_tool_node"),
	      usb_(static_cast<uint16_t>(declare_parameter<int>("vid", ares::kUsbVid)),
		   static_cast<uint16_t>(declare_parameter<int>("pid", ares::kUsbPid)))
	{
		timeout_ms_ = declare_parameter<int>("completion_timeout_ms", 15000);
		try_open();

		// 注册 service
		service_ = create_service<ToolAction>(
			"~/tool_action",
			std::bind(&AresToolNode::handle, this, std::placeholders::_1,
				  std::placeholders::_2));

		// 创建 suspension action client（供 arm_grasp 高度差 400 时使用）
		suspension_client_ =
			rclcpp_action::create_client<SuspensionControl>(
				this, "suspension_control");

		RCLCPP_INFO(get_logger(), "ares_tool_node ready; service: %s; actions: %s",
			    service_->get_service_name(), valid_actions().c_str());
	}

      private:
	// ---- 悬挂控制 ----
	// 同步等待 suspension action 完成；返回 true 表示成功。
	bool call_suspension(double target_height_mm)
	{
		if (!suspension_client_->action_server_is_ready()) {
			RCLCPP_ERROR(get_logger(),
				     "suspension_control action server not available");
			return false;
		}

		auto goal = SuspensionControl::Goal();
		goal.mode = SuspensionControl::Goal::MODE_DIRECT;    // 3
		goal.direction = SuspensionControl::Goal::DIR_FORWARD; // 0
		goal.height = static_cast<float>(target_height_mm);
		goal.timeout_sec = static_cast<float>(kSuspensionTimeoutSec);

		RCLCPP_INFO(get_logger(),
			    "Sending suspension goal: MODE_DIRECT height=%.1f mm",
			    goal.height);

		auto future_goal_handle =
			suspension_client_->async_send_goal(goal);

		// 等待 goal response
		auto goal_handle = future_goal_handle.get();
		if (!goal_handle) {
			RCLCPP_ERROR(get_logger(),
				     "Suspension goal rejected by server");
			return false;
		}

		// 等待 result
		auto result_future =
			suspension_client_->async_get_result(goal_handle);
		auto result = result_future.get();
		if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
		    result.result && result.result->success) {
			RCLCPP_INFO(get_logger(),
				    "Suspension done: %s", result.result->message.c_str());
			return true;
		}

		RCLCPP_ERROR(get_logger(), "Suspension failed: code=%d",
			     static_cast<int>(result.code));
		return false;
	}

	// ---- USB 设备管理 ----
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

	// ---- Service 处理 ----
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

		// ---- arm_grasp 高度差处理 ----
		// 调用者通过 args[0] 传入原始高度差值（来自 PopNextMeilinSegment）。
		std::array<float, 4> args = req->args;
		bool suspension_triggered = false;

		if (req->action == "arm_grasp") {
			const double height_diff = static_cast<double>(req->args[0]);

			if (std::abs(height_diff) > kHeightDiffThreshold) {
				// 正值映射为 200（给 MCU），负值映射为 -200
				if (height_diff > 0.0) {
					args[0] = static_cast<float>(kHeightDiffUpMapped);
				} else {
					args[0] = static_cast<float>(kHeightDiffDownMapped);
				}
				RCLCPP_INFO(get_logger(),
					    "arm_grasp height_diff=%.0f -> MCU args[0]=%.0f",
					    height_diff, args[0]);

				// 高度差 400: 先触发悬挂抬升，抓取完成后复位
				if (std::abs(height_diff - kHeightDiff400Suspension) <
				    kHeightDiffThreshold) {
					RCLCPP_INFO(get_logger(),
						    "arm_grasp: height_diff==400, triggering suspension lift to %.0f mm",
						    kSuspensionLiftHeight);
					suspension_triggered =
						call_suspension(kSuspensionLiftHeight);
					if (!suspension_triggered) {
						res->success = false;
						res->ret = -EIO;
						res->message =
							"arm_grasp suspension lift failed";
						RCLCPP_ERROR(get_logger(), "%s",
							     res->message.c_str());
						return;
					}
				}
			} else {
				RCLCPP_INFO(get_logger(),
					    "arm_grasp: no height_diff (args[0]=%.0f), passing through",
					    args[0]);
			}
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
			usb_.close();  // drop handle so the next call retries opening
			res->message =
				"USB transfer failed for action '" + req->action + "'";
			RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
		}

		// ---- arm_grasp 完成后复位悬挂 ----
		if (suspension_triggered) {
			RCLCPP_INFO(get_logger(),
				    "arm_grasp done, resetting suspension to %.0f mm",
				    kSuspensionNormalHeight);
			if (!call_suspension(kSuspensionNormalHeight)) {
				RCLCPP_ERROR(get_logger(),
					     "suspension reset failed (non-fatal)");
			}
		}
	}

	ares::AresUsb usb_;
	int timeout_ms_;
	std::string open_error_;
	rclcpp::Service<ToolAction>::SharedPtr service_;

	// 悬挂 action client（仅 arm_grasp + 高度差 400 时使用）
	rclcpp_action::Client<SuspensionControl>::SharedPtr suspension_client_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AresToolNode>());
	rclcpp::shutdown();
	return 0;
}
