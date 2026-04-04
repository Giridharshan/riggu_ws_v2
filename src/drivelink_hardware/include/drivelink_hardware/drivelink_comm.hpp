#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "drivelink_hardware/drivelink_messages.hpp"

namespace drivelink_hardware
{

class DriveLinkComm
{
public:
  using MotorStateCallback = std::function<void(const MotorStateTelemetry &)>;
  using MotorResponseCallback = std::function<void(const MotorConfigureResponse &)>;
  using ImuTelemetryCallback = std::function<void(const std::vector<uint8_t> &)>;

  explicit DriveLinkComm(std::string port = "/dev/ttyUSB0", int baudrate = 115200);
  ~DriveLinkComm();

  bool open();
  void close();
  bool is_open() const;

  void inject_test_fd(int fd);

  void set_motor_state_callback(MotorStateCallback callback);
  void set_motor_response_callback(MotorResponseCallback callback);
  void set_imu_telemetry_callback(ImuTelemetryCallback callback);

  bool send_frame(uint8_t msg_type, const uint8_t * payload, size_t payload_len);
  bool send_frame(uint8_t msg_type, const std::vector<uint8_t> & payload);

  bool send_motor_create(uint8_t motor_id, const MotorCreateOrConfigureBody & body);
  bool send_motor_set_mode(uint8_t motor_id, MotorControlMode mode);
  bool send_motor_setpoint(uint8_t motor_id, MotorSetpointKind kind, float value);
  bool send_motor_stop(uint8_t motor_id);
  bool send_motor_telemetry_global(bool enabled, uint8_t field_mask);
  bool send_imu_stream_cfg(bool enabled, uint16_t period_ms, uint16_t field_mask);
  bool send_setpoint(const MotorSetpointCmd & cmd);

  uint64_t crc_error_count() const;

private:
  enum class ParserState : uint8_t
  {
    kSeekStart0,
    kSeekStart1,
    kSeekStart2,
    kReadMsgType,
    kReadLength,
    kReadPayload,
    kReadCrcLow,
    kReadCrcHigh,
  };

  static constexpr uint8_t kProtocolVersion = 0x01;
  static constexpr size_t kMaxPayloadLength = 128;

  bool configure_serial_fd(int fd) const;
  bool send_motor_configure(MotorConfigureSubcommand subcommand, uint8_t motor_id);
  bool send_motor_configure(
    MotorConfigureSubcommand subcommand, uint8_t motor_id, const uint8_t * body,
    size_t body_len);
  bool send_imu_configure(ImuConfigureSubcommand subcommand, const uint8_t * body, size_t body_len);

  void reader_loop();
  void process_rx_byte(uint8_t byte);
  void reset_parser_state();
  void dispatch_frame(uint8_t msg_type, const std::vector<uint8_t> & payload);
  void parse_motor_telemetry_payload(const std::vector<uint8_t> & payload);

  std::string port_;
  int baudrate_;

  mutable std::mutex fd_mutex_;
  int fd_;

  std::mutex write_mutex_;

  std::atomic<bool> running_;
  std::thread reader_thread_;
  std::mutex reader_cv_mutex_;
  std::condition_variable reader_cv_;
  bool reader_exited_;

  std::atomic<uint64_t> crc_error_count_;

  mutable std::mutex callback_mutex_;
  MotorStateCallback motor_state_callback_;
  MotorResponseCallback motor_response_callback_;
  ImuTelemetryCallback imu_telemetry_callback_;

  ParserState parser_state_;
  uint8_t parser_msg_type_;
  uint8_t parser_payload_len_;
  std::vector<uint8_t> parser_payload_;
  uint8_t parser_crc_low_;
};

}  // namespace drivelink_hardware
