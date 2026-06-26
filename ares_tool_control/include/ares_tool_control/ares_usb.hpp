// USB transport for the ARES "dual protocol" SYNC frames used by ares_r2_tool.
//
// Command frame  (host -> MCU): head=0x5A5A, id=动作命令 ID, payload = action(uint32)
//                               + 4 reserved float32(=0)  -> 24 bytes
// Feedback frame (MCU -> host): feedback id, payload action echoes the completed action.
//
// arm: send 0x0211, receive 0x0221. connector/spear: send 0x0212, receive 0x0222.
// send_command() writes the command frame and then waits for the matching feedback
// frame (feedback id, same action), i.e. it blocks until the MCU reports completion.
#pragma once

#include <array>
#include <cstdint>
#include <string>

struct libusb_context;
struct libusb_device_handle;

namespace ares {

// USB identity (matches ares_r2_tool / ares_controller).
constexpr uint16_t kUsbVid = 0x1209;
constexpr uint16_t kUsbPid = 0x0002;

// SYNC frame fields (see app/ares_r2_tool/src/main.c and note.md).
constexpr uint16_t kSyncHead = 0x5A5A;
constexpr uint16_t kSyncIdArmCmd = 0x0211;    // host -> MCU: arm command
constexpr uint16_t kSyncIdArmDone = 0x0221;   // MCU -> host: arm feedback
constexpr uint16_t kSyncIdSpearCmd = 0x0212;  // host -> MCU: connector/spear command
constexpr uint16_t kSyncIdSpearDone = 0x0222; // MCU -> host: connector/spear feedback

// connector/spear action enum (ares_r2_tool: enum spear_cmd).
constexpr uint32_t kSpearPrepare = 1;
constexpr uint32_t kSpearGrasp = 2;
constexpr uint32_t kSpearDockExtend = 3;

// arm action enum (ares_r2_tool: enum arm_cmd). The ROS2 node exposes grasp/get/place;
// store actions are kept as protocol values but normal storage is automatic after grasp.
constexpr uint32_t kArmGrasp = 1;
constexpr uint32_t kArmStoreToBody = 2;
constexpr uint32_t kArmStoreOnArm = 3;
constexpr uint32_t kArmGetBody = 4;
constexpr uint32_t kArmPlaceMid = 5;
constexpr uint32_t kArmPlaceHigh = 6;

struct CommandResult {
	bool completed = false; // matching feedback frame received
	int32_t status = 0;     // 0 = completed; negative = -errno
};

class AresUsb {
      public:
	AresUsb(uint16_t vid = kUsbVid, uint16_t pid = kUsbPid);
	~AresUsb();

	AresUsb(const AresUsb &) = delete;
	AresUsb &operator=(const AresUsb &) = delete;

	bool is_open() const { return handle_ != nullptr; }

	// Returns empty string on success, otherwise a human-readable error.
	std::string open();
	void close();

	// Send the SYNC command frame and wait up to timeout_ms for the matching feedback
	// frame (feedback id, same action). args are the 4 reserved
	// float32 of the payload (action-specific meaning; e.g. prepare args[0]=length(m)).
	CommandResult send_command(uint16_t command_sync_id, uint16_t feedback_sync_id,
				   uint32_t action,
				   const std::array<float, 4> &args, int timeout_ms);

      private:
	bool write_frame(uint16_t sync_id, uint32_t action, const std::array<float, 4> &args);
	// 保活帧：刷新下位机 last_receive，避免长动作期间被判定为断连(online=false)
	// 而抑制完成帧。下位机把它当作 no-op 的 REPL 帧解析。
	bool write_keepalive();

	uint16_t vid_;
	uint16_t pid_;
	libusb_context *ctx_ = nullptr;
	libusb_device_handle *handle_ = nullptr;
	bool claimed_ = false;
};

}  // namespace ares
