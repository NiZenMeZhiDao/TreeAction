#include "ares_tool_control/ares_usb.hpp"

#include <libusb-1.0/libusb.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace ares {
namespace {

constexpr uint8_t kEpOut = 0x01;
constexpr uint8_t kEpIn = 0x81;
constexpr int kWriteTimeoutMs = 50;
constexpr int kReadPollMs = 50;
constexpr int kKeepaliveMs = 500; // 远小于下位机断连判定 10*HEART_BEAT_DELAY=2000ms
constexpr int kOnlinePrimeMs = 250; // > 一个心跳周期(200ms)，确保下位机 online 已置真
constexpr int kFrameSize = 24;    // head(2) + id(2) + action(4) + 4×float32(16)
constexpr int kTransferRetries = 3;
constexpr int kRetryDelayMs = 20;
constexpr int kReopenDelayMs = 200;

constexpr uint16_t kReplHead = 0xDEC0;  // 保活帧伪装成 no-op REPL 帧
constexpr int kKeepaliveSize = 10;      // REPL 帧长度

void put_u16_le(uint8_t *buf, uint16_t value)
{
	buf[0] = static_cast<uint8_t>(value & 0xffU);
	buf[1] = static_cast<uint8_t>((value >> 8U) & 0xffU);
}

void put_u32_le(uint8_t *buf, uint32_t value)
{
	buf[0] = static_cast<uint8_t>(value & 0xffU);
	buf[1] = static_cast<uint8_t>((value >> 8U) & 0xffU);
	buf[2] = static_cast<uint8_t>((value >> 16U) & 0xffU);
	buf[3] = static_cast<uint8_t>((value >> 24U) & 0xffU);
}

void put_f32_le(uint8_t *buf, float value)
{
	uint32_t raw;
	std::memcpy(&raw, &value, sizeof(raw));
	put_u32_le(buf, raw);
}

uint16_t get_u16_le(const uint8_t *buf)
{
	return static_cast<uint16_t>(buf[0]) | static_cast<uint16_t>(buf[1] << 8U);
}

uint32_t get_u32_le(const uint8_t *buf)
{
	return static_cast<uint32_t>(buf[0]) | static_cast<uint32_t>(buf[1] << 8U) |
	       static_cast<uint32_t>(buf[2] << 16U) | static_cast<uint32_t>(buf[3] << 24U);
}

bool is_retryable_libusb_error(int ret)
{
	return ret == LIBUSB_ERROR_IO || ret == LIBUSB_ERROR_BUSY ||
	       ret == LIBUSB_ERROR_TIMEOUT || ret == LIBUSB_ERROR_OVERFLOW;
}

std::string transfer_detail(unsigned char endpoint, int ret, int attempt,
			    int transferred, int length, const char *reason)
{
	char msg[192];
	std::snprintf(msg, sizeof(msg),
		      "%s: ep=0x%02X ret=%s attempt=%d/%d transferred=%d/%d",
		      reason, endpoint, libusb_error_name(ret), attempt,
		      kTransferRetries + 1, transferred, length);
	return msg;
}

int bulk_transfer_with_retry(libusb_device_handle *handle, unsigned char endpoint,
			     unsigned char *data, int length, int *transferred,
			     unsigned int timeout_ms, bool require_full_transfer,
			     std::string *detail)
{
	int last_ret = LIBUSB_ERROR_OTHER;
	for (int attempt = 0; attempt <= kTransferRetries; ++attempt) {
		*transferred = 0;
		const int ret = libusb_bulk_transfer(handle, endpoint, data, length, transferred,
						     timeout_ms);
		last_ret = ret;
		if (ret == 0 && (!require_full_transfer || *transferred == length)) {
			if (attempt > 0) {
				std::fprintf(stderr,
					     "ARES USB bulk transfer recovered: ep=0x%02X attempt=%d/%d transferred=%d/%d\n",
					     endpoint, attempt + 1, kTransferRetries + 1,
					     *transferred, length);
			}
			return ret;
		}

		// A partial command frame may already have reached the MCU. Do not blindly
		// retry it, because repeating a non-idempotent tool command is worse than
		// surfacing the transport failure.
		if (*transferred > 0) {
			if (detail != nullptr) {
				*detail = transfer_detail(endpoint, ret, attempt + 1,
							  *transferred, length,
							  "partial bulk transfer, not retrying");
			}
			std::fprintf(stderr, "ARES USB %s\n",
				     (detail != nullptr ? detail->c_str()
							 : transfer_detail(endpoint, ret, attempt + 1,
									   *transferred, length,
									   "partial bulk transfer, not retrying").c_str()));
			return ret == 0 ? LIBUSB_ERROR_IO : ret;
		}
		if (!is_retryable_libusb_error(ret) || attempt == kTransferRetries) {
			if (detail != nullptr) {
				*detail = transfer_detail(endpoint, ret, attempt + 1,
							  *transferred, length,
							  "bulk transfer failed");
			}
			std::fprintf(stderr, "ARES USB %s\n",
				     (detail != nullptr ? detail->c_str()
							 : transfer_detail(endpoint, ret, attempt + 1,
									   *transferred, length,
									   "bulk transfer failed").c_str()));
			return ret;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(kRetryDelayMs));
	}
	return last_ret;
}

}  // namespace

AresUsb::AresUsb(uint16_t vid, uint16_t pid) : vid_(vid), pid_(pid) {}

AresUsb::~AresUsb()
{
	close();
}

std::string AresUsb::open()
{
	int ret = libusb_init(&ctx_);
	if (ret != 0) {
		ctx_ = nullptr;
		return std::string("libusb_init failed: ") + libusb_error_name(ret);
	}

	/* 枚举以区分"设备不存在"与"存在但打不开(权限)" */
	libusb_device **list = nullptr;
	const ssize_t count = libusb_get_device_list(ctx_, &list);
	libusb_device *dev = nullptr;
	for (ssize_t i = 0; i < count; ++i) {
		libusb_device_descriptor desc{};
		if (libusb_get_device_descriptor(list[i], &desc) == 0 && desc.idVendor == vid_ &&
		    desc.idProduct == pid_) {
			dev = list[i];
			break;
		}
	}

	if (dev == nullptr) {
		if (list != nullptr) {
			libusb_free_device_list(list, 1);
		}
		char msg[96];
		std::snprintf(msg, sizeof(msg), "ARES USB device not found (VID=0x%04X PID=0x%04X)",
			      static_cast<unsigned>(vid_), static_cast<unsigned>(pid_));
		close();
		return msg;
	}

	ret = libusb_open(dev, &handle_);
	if (list != nullptr) {
		libusb_free_device_list(list, 1);
	}
	if (ret != 0) {
		handle_ = nullptr;
		std::string msg =
			std::string("device present but open failed: ") + libusb_error_name(ret);
		if (ret == LIBUSB_ERROR_ACCESS) {
			msg += " (permission denied: add a udev rule for VID 0x1209/PID 0x0002 or run as root)";
		}
		close();
		return msg;
	}

	if (libusb_kernel_driver_active(handle_, 0) == 1) {
		libusb_detach_kernel_driver(handle_, 0);
	}

	ret = libusb_claim_interface(handle_, 0);
	if (ret != 0) {
		const std::string err =
			std::string("libusb_claim_interface failed: ") + libusb_error_name(ret);
		close();
		return err;
	}

	claimed_ = true;
	return std::string();
}

void AresUsb::close()
{
	if (handle_ != nullptr) {
		if (claimed_) {
			libusb_release_interface(handle_, 0);
			claimed_ = false;
		}
		libusb_close(handle_);
		handle_ = nullptr;
	}

	if (ctx_ != nullptr) {
		libusb_exit(ctx_);
		ctx_ = nullptr;
	}
}

int AresUsb::write_frame(uint16_t sync_id, uint32_t action,
			 const std::array<float, 4> &args, std::string *detail)
{
	std::array<uint8_t, kFrameSize> frame{};
	put_u16_le(&frame[0], kSyncHead);
	put_u16_le(&frame[2], sync_id);
	put_u32_le(&frame[4], action);
	for (int i = 0; i < 4; ++i) {
		put_f32_le(&frame[8 + i * 4], args[i]); /* 4 个预留 float32 */
	}

	int transferred = 0;
	const int ret = bulk_transfer_with_retry(
		handle_, kEpOut, frame.data(), static_cast<int>(frame.size()), &transferred,
		kWriteTimeoutMs, true, detail);
	if (ret == 0 && transferred == static_cast<int>(frame.size())) {
		return 0;
	}
	return ret == 0 ? LIBUSB_ERROR_IO : ret;
}

int AresUsb::write_keepalive(std::string *detail)
{
	std::array<uint8_t, kKeepaliveSize> frame{};
	put_u16_le(&frame[0], kReplHead);
	/* 其余字节为 0：下位机按 REPL 帧解析，func_ret_cb 未设置 → no-op */

	int transferred = 0;
	const int ret = bulk_transfer_with_retry(
		handle_, kEpOut, frame.data(), static_cast<int>(frame.size()), &transferred,
		kWriteTimeoutMs, true, detail);
	if (ret == 0 && transferred == static_cast<int>(frame.size())) {
		return 0;
	}
	return ret == 0 ? LIBUSB_ERROR_IO : ret;
}

CommandResult AresUsb::send_command(uint16_t command_sync_id, uint16_t feedback_sync_id,
				    uint32_t action,
				    const std::array<float, 4> &args, int timeout_ms)
{
	CommandResult result;
	if (handle_ == nullptr) {
		result.status = -EIO;
		result.detail = "USB handle is null before send_command";
		return result;
	}

	/* 预热：先发保活帧并等一个心跳周期，确保下位机 online=true，避免极快动作(如 grasp)
	 * 在 online 尚未恢复时就发完成帧而被抑制。 */
	auto reopen_after_disconnect = [this, &result](const std::string &context) -> bool {
		close();
		std::this_thread::sleep_for(std::chrono::milliseconds(kReopenDelayMs));
		const std::string err = open();
		if (!err.empty()) {
			result.status = -EIO;
			result.detail = context + ": reopen failed after USB disconnect: " + err;
			return false;
		}
		std::fprintf(stderr, "ARES USB reopened after disconnect during %s\n",
			     context.c_str());
		return true;
	};

	std::string detail;
	int ret = write_keepalive(&detail);
	if (ret == LIBUSB_ERROR_NO_DEVICE) {
		if (!reopen_after_disconnect("pre-command keepalive")) {
			return result;
		}
		detail.clear();
		ret = write_keepalive(&detail);
		if (ret != 0) {
			result.status = -EIO;
			result.detail = detail.empty() ? "keepalive write failed after reopen" : detail;
			return result;
		}
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(kOnlinePrimeMs));

	detail.clear();
	ret = write_frame(command_sync_id, action, args, &detail);
	if (ret == LIBUSB_ERROR_NO_DEVICE &&
	    detail.find("partial bulk transfer") == std::string::npos) {
		if (!reopen_after_disconnect("command frame write")) {
			return result;
		}
		detail.clear();
		write_keepalive(&detail);
		std::this_thread::sleep_for(std::chrono::milliseconds(kOnlinePrimeMs));
		detail.clear();
		ret = write_frame(command_sync_id, action, args, &detail);
	}
	if (ret != 0) {
		result.status = -EIO;
		result.detail = detail.empty() ? "command frame write failed" : detail;
		return result;
	}

	const auto deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
	auto next_keepalive =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(kKeepaliveMs);
	std::array<uint8_t, 64> buf{};

	while (std::chrono::steady_clock::now() < deadline) {
		/* 周期性保活：长动作期间上位机无上行数据会被下位机判定为断连(online=false)，
		 * 从而抑制完成帧。定期发一帧 no-op 刷新其 last_receive。 */
		if (std::chrono::steady_clock::now() >= next_keepalive) {
			write_keepalive(&detail);
			next_keepalive = std::chrono::steady_clock::now() +
					 std::chrono::milliseconds(kKeepaliveMs);
		}

		int transferred = 0;
		detail.clear();
		const int ret = bulk_transfer_with_retry(
			handle_, kEpIn, buf.data(), static_cast<int>(buf.size()), &transferred,
			kReadPollMs, false, &detail);
		if (ret == LIBUSB_ERROR_TIMEOUT) {
			continue;
		}
		if (ret != 0) {
			result.status = -EIO;
			result.detail = detail.empty() ? "feedback read failed" : detail;
			return result;
		}
		/* 仅匹配本动作对象的反馈帧(反馈 ID、同 action)，忽略心跳等其它帧 */
		if (transferred >= kFrameSize && get_u16_le(buf.data()) == kSyncHead &&
		    get_u16_le(buf.data() + 2) == feedback_sync_id &&
		    get_u32_le(buf.data() + 4) == action) {
			result.completed = true;
			result.status = 0;
			return result;
		}
	}

	result.status = -ETIMEDOUT;
	result.detail = "matching completion frame not received before timeout";
	return result;
}

}  // namespace ares
