#include "drivelink_hardware/drivelink_comm.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <thread>
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
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default: return B115200;
  }
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
    RCLCPP_ERROR(comm_logger(), "DriveLinkComm: tcgetattr failed: %s", std::strerror(errno));
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
    RCLCPP_ERROR(comm_logger(), "DriveLinkComm: tcsetattr failed: %s", std::strerror(errno));
    return false;
  }

  if (tcflush(fd, TCIOFLUSH) != 0) {
    RCLCPP_ERROR(comm_logger(), "DriveLinkComm: tcflush failed: %s", std::strerror(errno));
    return false;
  }

  return true;
}

bool DriveLinkComm::open()
{
  if (running_.load()) {
    RCLCPP_DEBUG(comm_logger(), "DriveLinkComm: open() called while already running");
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ < 0) {
      const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) {
        RCLCPP_ERROR(comm_logger(), "DriveLinkComm: open(%s) failed: %s", port_.c_str(), std::strerror(errno));
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
    RCLCPP_DEBUG(comm_logger(), "DriveLinkComm: close() called while already closed");
    return;
  }

  running_.store(false);

  if (reader_thread_.joinable()) {
    std::unique_lock<std::mutex> lock(reader_cv_mutex_);
    const bool exited = reader_cv_.wait_for(lock, constants::kReaderShutdownTimeout, [this]() {
      return reader_exited_;
    });
    lock.unlock();
    if (exited) {
      reader_thread_.join();
    } else {
      RCLCPP_WARN(comm_logger(), "DriveLinkComm: reader thread did not stop in time; detaching");
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
    RCLCPP_ERROR(comm_logger(), "DriveLinkComm: payload too large: %zu", payload_len);
    return false;
  }

  int local_fd = -1;
  {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    local_fd = fd_;
  }
  if (local_fd < 0) {
    RCLCPP_ERROR(comm_logger(), "DriveLinkComm: send_frame called with closed fd");
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
  size_t eagain_retries = 0;
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
      if (eagain_retries > constants::kWriteRetryLimit) {
        RCLCPP_ERROR(comm_logger(), "DriveLinkComm: write retry limit exceeded");
        return false;
      }
      std::this_thread::yield();
      continue;
    }

    RCLCPP_ERROR(comm_logger(), "DriveLinkComm: write failed: %s", std::strerror(errno));
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

}  // namespace drivelink_hardware
