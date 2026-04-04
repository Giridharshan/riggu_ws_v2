#include "drivelink_hardware/drivelink_comm.hpp"

#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "drivelink_hardware/constants.hpp"
#include "drivelink_hardware/crc16.hpp"

namespace drivelink_hardware
{
namespace
{

rclcpp::Logger comm_logger()
{
  return rclcpp::get_logger("drivelink_hardware.comm");
}

template<typename T>
bool read_le(const std::vector<uint8_t> & bytes, size_t offset, T & out)
{
  if (offset + sizeof(T) > bytes.size()) {
    return false;
  }
  std::memcpy(&out, bytes.data() + offset, sizeof(T));
  return true;
}

}  // namespace

void DriveLinkComm::reader_loop()
{
  int local_fd = -1;
  {
    std::lock_guard<std::mutex> fd_lock(fd_mutex_);
    local_fd = fd_;
  }

  while (running_.load()) {
    pollfd pfd;
    pfd.fd = local_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    const int ret = ::poll(&pfd, 1, constants::kReaderPollTimeoutMs);
    if (!running_.load()) {
      break;
    }

    if (ret == 0) {
      continue;
    }

    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(comm_logger(), "DriveLinkComm: poll failed: %s", std::strerror(errno));
      break;
    }

    if ((pfd.revents & POLLIN) == 0) {
      continue;
    }

    uint8_t rx_buffer[256];
    const ssize_t bytes_read = ::read(local_fd, rx_buffer, sizeof(rx_buffer));
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(comm_logger(), "DriveLinkComm: read failed: %s", std::strerror(errno));
      break;
    }

    if (bytes_read == 0) {
      continue;
    }

    for (ssize_t i = 0; i < bytes_read; ++i) {
      process_rx_byte(rx_buffer[i]);
    }
  }

  {
    std::lock_guard<std::mutex> lock(reader_cv_mutex_);
    reader_exited_ = true;
  }
  reader_cv_.notify_all();
}

void DriveLinkComm::process_rx_byte(uint8_t byte)
{
  switch (parser_state_) {
    case ParserState::kSeekStart0:
      parser_state_ = (byte == kDriveLinkStartByte) ? ParserState::kSeekStart1 : ParserState::kSeekStart0;
      break;

    case ParserState::kSeekStart1:
      if (byte == kDriveLinkStartByte) {
        parser_state_ = ParserState::kSeekStart2;
      } else {
        parser_state_ = (byte == kDriveLinkStartByte) ? ParserState::kSeekStart1 : ParserState::kSeekStart0;
      }
      break;

    case ParserState::kSeekStart2:
      if (byte == kDriveLinkStartByte) {
        parser_state_ = ParserState::kReadMsgType;
      } else {
        parser_state_ = (byte == kDriveLinkStartByte) ? ParserState::kSeekStart1 : ParserState::kSeekStart0;
      }
      break;

    case ParserState::kReadMsgType:
      parser_msg_type_ = byte;
      parser_state_ = ParserState::kReadLength;
      break;

    case ParserState::kReadLength:
      parser_payload_len_ = byte;
      if (parser_payload_len_ > kMaxPayloadLength) {
        RCLCPP_ERROR(
          comm_logger(), "DriveLinkComm: invalid payload length %d, dropping frame",
          static_cast<int>(parser_payload_len_));
        reset_parser_state();
        break;
      }
      parser_payload_.clear();
      parser_payload_.reserve(parser_payload_len_);
      parser_state_ = (parser_payload_len_ == 0) ? ParserState::kReadCrcLow : ParserState::kReadPayload;
      break;

    case ParserState::kReadPayload:
      parser_payload_.push_back(byte);
      if (parser_payload_.size() >= parser_payload_len_) {
        parser_state_ = ParserState::kReadCrcLow;
      }
      break;

    case ParserState::kReadCrcLow:
      parser_crc_low_ = byte;
      parser_state_ = ParserState::kReadCrcHigh;
      break;

    case ParserState::kReadCrcHigh:
    {
      const uint16_t expected_crc =
        static_cast<uint16_t>(parser_crc_low_ | (static_cast<uint16_t>(byte) << 8));

      std::vector<uint8_t> crc_input;
      crc_input.reserve(2 + parser_payload_.size());
      crc_input.push_back(parser_msg_type_);
      crc_input.push_back(parser_payload_len_);
      crc_input.insert(crc_input.end(), parser_payload_.begin(), parser_payload_.end());

      const uint16_t computed_crc = crc16(crc_input.data(), crc_input.size());
      if (computed_crc != expected_crc) {
        crc_error_count_.fetch_add(1);
        RCLCPP_ERROR(
          comm_logger(), "DriveLinkComm: CRC mismatch expected=0x%04X got=0x%04X",
          static_cast<unsigned int>(expected_crc), static_cast<unsigned int>(computed_crc));
        reset_parser_state();
        break;
      }

      dispatch_frame(parser_msg_type_, parser_payload_);
      reset_parser_state();
      break;
    }
  }
}

void DriveLinkComm::reset_parser_state()
{
  parser_state_ = ParserState::kSeekStart0;
  parser_msg_type_ = 0;
  parser_payload_len_ = 0;
  parser_payload_.clear();
  parser_crc_low_ = 0;
}

void DriveLinkComm::dispatch_frame(uint8_t msg_type, const std::vector<uint8_t> & payload)
{
  if (msg_type == static_cast<uint8_t>(MessageType::kMotorTelemetryStream)) {
    parse_motor_telemetry_payload(payload);
    return;
  }

  if (msg_type == static_cast<uint8_t>(MessageType::kMotorConfigureResponse)) {
    if (payload.size() >= sizeof(MotorConfigureResponse)) {
      MotorConfigureResponse response{};
      std::memcpy(&response, payload.data(), sizeof(response));

      MotorResponseCallback callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = motor_response_callback_;
      }
      if (callback) {
        callback(response);
      }
    }
    return;
  }

  if (msg_type == static_cast<uint8_t>(MessageType::kImuTelemetryStream)) {
    ImuTelemetryCallback callback;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback = imu_telemetry_callback_;
    }
    if (callback) {
      callback(payload);
    }
    return;
  }

  if (msg_type == static_cast<uint8_t>(MessageType::kTimeSyncRequest)) {
    handle_time_sync_request(payload);
    return;
  }
}

void DriveLinkComm::handle_time_sync_request(const std::vector<uint8_t> & payload)
{
  if (payload.size() != sizeof(TimeSyncRequestPayload)) {
    RCLCPP_ERROR(
      comm_logger(), "DriveLinkComm: invalid time sync request payload size: %zu", payload.size());
    return;
  }

  TimeSyncRequestPayload request{};
  std::memcpy(&request, payload.data(), sizeof(request));

  const auto now_rx = std::chrono::system_clock::now();
  const auto rx_us = std::chrono::duration_cast<std::chrono::microseconds>(
    now_rx.time_since_epoch()).count();

  TimeSyncResponsePayload response{};
  response.version = request.version;
  response.status = 0;
  response.request_id = request.request_id;
  response.echoed_client_unix_send_us = request.client_unix_send_us;
  response.server_unix_rx_us = static_cast<int64_t>(rx_us);

  const auto now_tx = std::chrono::system_clock::now();
  const auto tx_us = std::chrono::duration_cast<std::chrono::microseconds>(
    now_tx.time_since_epoch()).count();
  response.server_unix_tx_us = static_cast<int64_t>(tx_us);
  response.reserved0 = 0;
  response.reserved1 = 0;

  if (!send_frame(
      static_cast<uint8_t>(MessageType::kTimeSyncResponse),
      reinterpret_cast<const uint8_t *>(&response), sizeof(response)))
  {
    RCLCPP_ERROR(
      comm_logger(), "DriveLinkComm: failed to send time sync response request_id=%u",
      static_cast<unsigned int>(request.request_id));
    return;
  }

  TimeSyncCallback callback;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback = time_sync_callback_;
  }
  if (callback) {
    callback(request, response);
  }
}

void DriveLinkComm::parse_motor_telemetry_payload(const std::vector<uint8_t> & payload)
{
  if (payload.size() < sizeof(MotorTelemetryHeader)) {
    return;
  }

  uint8_t version = 0;
  uint8_t entry_count = 0;
  uint64_t timestamp_us = 0;
  if (!read_le(payload, 0, version) || !read_le(payload, 1, entry_count) ||
    !read_le(payload, 2, timestamp_us))
  {
    return;
  }

  size_t offset = sizeof(MotorTelemetryHeader);
  for (uint8_t i = 0; i < entry_count; ++i) {
    if (offset + sizeof(MotorTelemetryEntryHeader) > payload.size()) {
      return;
    }

    uint8_t motor_id = payload[offset];
    uint8_t field_mask = payload[offset + 1];
    offset += sizeof(MotorTelemetryEntryHeader);

    float angle_rad = 0.0F;
    float velocity_rad_s = 0.0F;

    if ((field_mask & static_cast<uint8_t>(MotorTelemetryFieldMask::kAngle)) != 0U) {
      if (!read_le(payload, offset, angle_rad)) {
        return;
      }
      offset += sizeof(float);
    }

    if ((field_mask & static_cast<uint8_t>(MotorTelemetryFieldMask::kVelocity)) != 0U) {
      if (!read_le(payload, offset, velocity_rad_s)) {
        return;
      }
      offset += sizeof(float);
    }

    MotorStateTelemetry telemetry{};
    telemetry.version = version;
    telemetry.entry_count = 1;
    telemetry.timestamp_us = timestamp_us;
    telemetry.motor_id = motor_id;
    telemetry.field_mask = field_mask;
    telemetry.angle_rad = angle_rad;
    telemetry.angular_velocity_rad_s = velocity_rad_s;

    MotorStateCallback callback;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback = motor_state_callback_;
    }
    if (callback) {
      callback(telemetry);
    }
  }
}

}  // namespace drivelink_hardware
