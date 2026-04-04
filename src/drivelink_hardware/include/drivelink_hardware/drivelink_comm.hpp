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

/**
 * @brief Serial transport and frame parser for DriveLink firmware protocol.
 *
 * Owns UART file descriptor lifecycle, asynchronous read loop, frame parsing, and command
 * serialization for motor/IMU/time-sync protocol messages.
 */
class DriveLinkComm
{
public:
  using MotorStateCallback = std::function<void(const MotorStateTelemetry &)>;
  using MotorResponseCallback = std::function<void(const MotorConfigureResponse &)>;
  using ImuTelemetryCallback = std::function<void(const std::vector<uint8_t> &)>;
  using TimeSyncCallback =
    std::function<void(const TimeSyncRequestPayload &, const TimeSyncResponsePayload &)>;

  /**
   * @brief Construct a communication object for a UART endpoint.
   * @param port UART device path.
   * @param baudrate UART baud rate.
   */
  explicit DriveLinkComm(std::string port = "/dev/ttyUSB0", int baudrate = 115200);

  /**
   * @brief Destroy the communication object and close transport resources.
   */
  ~DriveLinkComm();

  /**
   * @brief Open the serial endpoint and start the reader thread.
   * @return True if communication is open or was already running.
   */
  bool open();

  /**
   * @brief Stop the reader thread and close the serial endpoint.
   * @return None.
   */
  void close();

  /**
   * @brief Check whether communication is currently running.
   * @return True if open and reader loop is active.
   */
  bool is_open() const;

  /**
   * @brief Inject a test file descriptor instead of opening a real UART.
   * @param fd File descriptor to use for I/O.
   * @return None.
   */
  void inject_test_fd(int fd);

  /**
   * @brief Register callback for decoded motor telemetry entries.
   * @param callback Callback invoked with parsed motor state telemetry.
   * @return None.
   */
  void set_motor_state_callback(MotorStateCallback callback);

  /**
   * @brief Register callback for motor configure responses.
   * @param callback Callback invoked with motor configure response payloads.
   * @return None.
   */
  void set_motor_response_callback(MotorResponseCallback callback);

  /**
   * @brief Register callback for IMU telemetry payloads.
   * @param callback Callback invoked with raw IMU telemetry payload bytes.
   * @return None.
   */
  void set_imu_telemetry_callback(ImuTelemetryCallback callback);

  /**
   * @brief Register callback for time-sync request/response pairs.
   * @param callback Callback invoked after sending a time-sync response.
   * @return None.
   */
  void set_time_sync_callback(TimeSyncCallback callback);

  /**
   * @brief Send a protocol frame with raw payload pointer.
   * @param msg_type Message type byte.
   * @param payload Pointer to payload bytes (may be null when payload_len is zero).
   * @param payload_len Number of payload bytes.
   * @return True if the frame was fully written.
   */
  bool send_frame(uint8_t msg_type, const uint8_t * payload, size_t payload_len);

  /**
   * @brief Send a protocol frame from payload vector.
   * @param msg_type Message type byte.
   * @param payload Payload bytes.
   * @return True if the frame was fully written.
   */
  bool send_frame(uint8_t msg_type, const std::vector<uint8_t> & payload);

  /**
   * @brief Send motor create/configure payload for a motor ID.
   * @param motor_id Firmware motor identifier.
   * @param body Motor create/configure body.
   * @return True if frame send succeeded.
   */
  bool send_motor_create(uint8_t motor_id, const MotorCreateOrConfigureBody & body);

  /**
   * @brief Send motor control mode command.
   * @param motor_id Firmware motor identifier.
   * @param mode Target control mode.
   * @return True if frame send succeeded.
   */
  bool send_motor_set_mode(uint8_t motor_id, MotorControlMode mode);

  /**
   * @brief Send one motor setpoint command.
   * @param motor_id Firmware motor identifier.
   * @param kind Setpoint type.
   * @param value Setpoint numeric value.
   * @return True if frame send succeeded.
   */
  bool send_motor_setpoint(uint8_t motor_id, MotorSetpointKind kind, float value);

  /**
   * @brief Send stop command for one motor.
   * @param motor_id Firmware motor identifier.
   * @return True if frame send succeeded.
   */
  bool send_motor_stop(uint8_t motor_id);

  /**
   * @brief Configure global motor telemetry enable/mask.
   * @param enabled Whether telemetry stream is enabled.
   * @param field_mask Bitmask of requested telemetry fields.
   * @return True if frame send succeeded.
   */
  bool send_motor_telemetry_global(bool enabled, uint8_t field_mask);

  /**
   * @brief Configure IMU stream enable, period, and field mask.
   * @param enabled Whether IMU stream should be enabled.
   * @param period_ms Stream period in milliseconds.
   * @param field_mask IMU field selection mask.
   * @return True if all required IMU configure frames were sent successfully.
   */
  bool send_imu_stream_cfg(bool enabled, uint16_t period_ms, uint16_t field_mask);

  /**
   * @brief Send a batched setpoint stream command.
   * @param cmd Packed setpoint stream command payload.
   * @return True if frame send succeeded.
   */
  bool send_setpoint(const MotorSetpointCmd & cmd);

  /**
   * @brief Return count of frames rejected due to CRC mismatch.
   * @return Total CRC mismatch count.
   */
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
  void handle_time_sync_request(const std::vector<uint8_t> & payload);

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
  TimeSyncCallback time_sync_callback_;

  ParserState parser_state_;
  uint8_t parser_msg_type_;
  uint8_t parser_payload_len_;
  std::vector<uint8_t> parser_payload_;
  uint8_t parser_crc_low_;
};

}  // namespace drivelink_hardware
