#include "drivelink_hardware/drivelink_comm.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "drivelink_hardware/crc16.hpp"

namespace drivelink_hardware
{
namespace
{

speed_t to_termios_baud(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return B115200;
  }
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

DriveLinkComm::DriveLinkComm(std::string port, int baudrate)
: port_(std::move(port)),
  baudrate_(baudrate),
  fd_(-1),
  running_(false),
  reader_exited_(true),
  crc_error_count_(0),
  parser_state_(ParserState::kSeekStart0),
  parser_msg_type_(0),
  parser_payload_len_(0),
  parser_crc_low_(0)
{
}

DriveLinkComm::~DriveLinkComm()
{
  close();
}

bool DriveLinkComm::configure_serial_fd(int fd) const
{
  termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    std::cerr << "DriveLinkComm: tcgetattr failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  const speed_t baud = to_termios_baud(baudrate_);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cerr << "DriveLinkComm: tcsetattr failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  if (tcflush(fd, TCIOFLUSH) != 0) {
    std::cerr << "DriveLinkComm: tcflush failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool DriveLinkComm::open()
{
  if (running_.load()) {
    std::cerr << "DriveLinkComm: open() called while already running; no-op" << std::endl;
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ < 0) {
      const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) {
        std::cerr << "DriveLinkComm: open(" << port_ << ") failed: " << std::strerror(errno)
                  << std::endl;
        return false;
      }
      if (!configure_serial_fd(fd)) {
        ::close(fd);
        return false;
      }
      fd_ = fd;
    }
  }

  reset_parser_state();
  running_.store(true);
  {
    std::lock_guard<std::mutex> lock(reader_cv_mutex_);
    reader_exited_ = false;
  }
  reader_thread_ = std::thread(&DriveLinkComm::reader_loop, this);
  return true;
}

void DriveLinkComm::close()
{
  if (!running_.load() && fd_ < 0) {
    std::cerr << "DriveLinkComm: close() called while already closed; no-op" << std::endl;
    return;
  }

  running_.store(false);

  if (reader_thread_.joinable()) {
    std::unique_lock<std::mutex> lock(reader_cv_mutex_);
    const bool exited = reader_cv_.wait_for(lock, std::chrono::seconds(2), [this]() {
      return reader_exited_;
    });
    lock.unlock();
    if (exited) {
      reader_thread_.join();
    } else {
      std::cerr << "DriveLinkComm: reader thread did not stop within 2 seconds; detaching"
                << std::endl;
      reader_thread_.detach();
    }
  }

  std::lock_guard<std::mutex> fd_lock(fd_mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool DriveLinkComm::is_open() const
{
  return running_.load();
}

void DriveLinkComm::inject_test_fd(int fd)
{
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
  }
  fd_ = fd;
}

void DriveLinkComm::set_motor_state_callback(MotorStateCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  motor_state_callback_ = std::move(callback);
}

void DriveLinkComm::set_motor_response_callback(MotorResponseCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  motor_response_callback_ = std::move(callback);
}

void DriveLinkComm::set_imu_telemetry_callback(ImuTelemetryCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  imu_telemetry_callback_ = std::move(callback);
}

void DriveLinkComm::set_time_sync_callback(TimeSyncCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  time_sync_callback_ = std::move(callback);
}

bool DriveLinkComm::send_frame(uint8_t msg_type, const std::vector<uint8_t> & payload)
{
  return send_frame(msg_type, payload.data(), payload.size());
}

bool DriveLinkComm::send_frame(uint8_t msg_type, const uint8_t * payload, size_t payload_len)
{
  if (payload_len > kMaxPayloadLength || payload_len > std::numeric_limits<uint8_t>::max()) {
    std::cerr << "DriveLinkComm: payload too large: " << payload_len << std::endl;
    return false;
  }

  int local_fd = -1;
  {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    local_fd = fd_;
  }
  if (local_fd < 0) {
    std::cerr << "DriveLinkComm: send_frame called with closed fd" << std::endl;
    return false;
  }

  std::vector<uint8_t> frame;
  frame.reserve(7 + payload_len);
  frame.push_back(kDriveLinkStartByte);
  frame.push_back(kDriveLinkStartByte);
  frame.push_back(kDriveLinkStartByte);
  frame.push_back(msg_type);
  frame.push_back(static_cast<uint8_t>(payload_len));
  if (payload_len > 0) {
    frame.insert(frame.end(), payload, payload + payload_len);
  }

  std::vector<uint8_t> crc_input;
  crc_input.reserve(2 + payload_len);
  crc_input.push_back(msg_type);
  crc_input.push_back(static_cast<uint8_t>(payload_len));
  if (payload_len > 0) {
    crc_input.insert(crc_input.end(), payload, payload + payload_len);
  }

  const uint16_t crc = crc16(crc_input.data(), crc_input.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));

  std::lock_guard<std::mutex> write_lock(write_mutex_);
  size_t offset = 0;
  int eagain_retries = 0;
  while (offset < frame.size()) {
    const ssize_t written = ::write(local_fd, frame.data() + offset, frame.size() - offset);
    if (written > 0) {
      offset += static_cast<size_t>(written);
      continue;
    }

    if (written < 0 && errno == EINTR) {
      continue;
    }

    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      ++eagain_retries;
      if (eagain_retries > 3) {
        std::cerr << "DriveLinkComm: write retry limit exceeded" << std::endl;
        return false;
      }
      std::this_thread::yield();
      continue;
    }

    std::cerr << "DriveLinkComm: write failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool DriveLinkComm::send_motor_configure(
  MotorConfigureSubcommand subcommand, uint8_t motor_id, const uint8_t * body, size_t body_len)
{
  std::vector<uint8_t> payload;
  payload.reserve(3 + body_len);
  payload.push_back(kProtocolVersion);
  payload.push_back(static_cast<uint8_t>(subcommand));
  payload.push_back(motor_id);
  if (body_len > 0) {
    payload.insert(payload.end(), body, body + body_len);
  }
  return send_frame(static_cast<uint8_t>(MessageType::kMotorConfigureCommand), payload);
}

bool DriveLinkComm::send_motor_configure(MotorConfigureSubcommand subcommand, uint8_t motor_id)
{
  return send_motor_configure(subcommand, motor_id, nullptr, 0);
}

bool DriveLinkComm::send_imu_configure(
  ImuConfigureSubcommand subcommand, const uint8_t * body, size_t body_len)
{
  std::vector<uint8_t> payload;
  payload.reserve(2 + body_len);
  payload.push_back(kProtocolVersion);
  payload.push_back(static_cast<uint8_t>(subcommand));
  if (body_len > 0) {
    payload.insert(payload.end(), body, body + body_len);
  }
  return send_frame(static_cast<uint8_t>(MessageType::kImuConfigureCommand), payload);
}

bool DriveLinkComm::send_motor_create(uint8_t motor_id, const MotorCreateOrConfigureBody & body)
{
  return send_motor_configure(
    MotorConfigureSubcommand::kCreateMotor, motor_id,
    reinterpret_cast<const uint8_t *>(&body), sizeof(body));
}

bool DriveLinkComm::send_motor_set_mode(uint8_t motor_id, MotorControlMode mode)
{
  const MotorSetModeBody body{static_cast<uint8_t>(mode)};
  return send_motor_configure(
    MotorConfigureSubcommand::kSetMode, motor_id,
    reinterpret_cast<const uint8_t *>(&body), sizeof(body));
}

bool DriveLinkComm::send_motor_setpoint(uint8_t motor_id, MotorSetpointKind kind, float value)
{
  const MotorSetpointBody body{static_cast<uint8_t>(kind), value};
  return send_motor_configure(
    MotorConfigureSubcommand::kSetSetpoint, motor_id,
    reinterpret_cast<const uint8_t *>(&body), sizeof(body));
}

bool DriveLinkComm::send_motor_stop(uint8_t motor_id)
{
  return send_motor_configure(MotorConfigureSubcommand::kStopMotor, motor_id);
}

bool DriveLinkComm::send_motor_telemetry_global(bool enabled, uint8_t field_mask)
{
  const MotorSetTelemetryGlobalBody body{static_cast<uint8_t>(enabled ? 1U : 0U), field_mask};
  return send_motor_configure(
    MotorConfigureSubcommand::kSetTelemetryGlobal, 0,
    reinterpret_cast<const uint8_t *>(&body), sizeof(body));
}

bool DriveLinkComm::send_imu_stream_cfg(bool enabled, uint16_t period_ms, uint16_t field_mask)
{
  const ImuSetStreamEnabledBody enabled_body{static_cast<uint8_t>(enabled ? 1U : 0U)};
  if (!send_imu_configure(
      ImuConfigureSubcommand::kSetStreamEnabled,
      reinterpret_cast<const uint8_t *>(&enabled_body), sizeof(enabled_body)))
  {
    return false;
  }

  const ImuSetStreamFieldMaskBody mask_body{field_mask};
  if (!send_imu_configure(
      ImuConfigureSubcommand::kSetStreamFieldMask,
      reinterpret_cast<const uint8_t *>(&mask_body), sizeof(mask_body)))
  {
    return false;
  }

  const ImuSetStreamPeriodBody period_body{period_ms};
  return send_imu_configure(
    ImuConfigureSubcommand::kSetStreamPeriodMs,
    reinterpret_cast<const uint8_t *>(&period_body), sizeof(period_body));
}

bool DriveLinkComm::send_setpoint(const MotorSetpointCmd & cmd)
{
  return send_frame(
    static_cast<uint8_t>(MessageType::kMotorSetpointStream),
    reinterpret_cast<const uint8_t *>(&cmd), sizeof(cmd));
}

uint64_t DriveLinkComm::crc_error_count() const
{
  return crc_error_count_.load();
}

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

    const int ret = ::poll(&pfd, 1, 50);
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
      std::cerr << "DriveLinkComm: poll failed: " << std::strerror(errno) << std::endl;
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
      std::cerr << "DriveLinkComm: read failed: " << std::strerror(errno) << std::endl;
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
        std::cerr << "DriveLinkComm: invalid payload length " << static_cast<int>(parser_payload_len_)
                  << ", dropping frame and resyncing" << std::endl;
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
        std::cerr << "DriveLinkComm: CRC mismatch expected=0x" << std::hex << expected_crc
                  << " got=0x" << computed_crc << std::dec << std::endl;
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
    std::cerr << "DriveLinkComm: invalid time sync request payload size: " << payload.size()
              << std::endl;
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

  // ⚠ VERIFY: firmware expects UNIX microseconds. This implementation maps host
  // system_clock to UNIX epoch microseconds for both RX and TX timestamps.
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
    std::cerr << "DriveLinkComm: failed to send time sync response for request_id="
              << request.request_id << std::endl;
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
