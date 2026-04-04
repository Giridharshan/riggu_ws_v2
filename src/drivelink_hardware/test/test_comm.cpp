#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "drivelink_hardware/crc16.hpp"
#include "drivelink_hardware/drivelink_comm.hpp"
#include "drivelink_hardware/drivelink_messages.hpp"

namespace
{

std::vector<uint8_t> make_frame(uint8_t msg_type, const uint8_t * payload, size_t payload_len)
{
  std::vector<uint8_t> frame;
  frame.reserve(7 + payload_len);
  frame.push_back(kDriveLinkStartByte);
  frame.push_back(kDriveLinkStartByte);
  frame.push_back(kDriveLinkStartByte);
  frame.push_back(msg_type);
  frame.push_back(static_cast<uint8_t>(payload_len));
  frame.insert(frame.end(), payload, payload + payload_len);

  std::vector<uint8_t> crc_input;
  crc_input.reserve(2 + payload_len);
  crc_input.push_back(msg_type);
  crc_input.push_back(static_cast<uint8_t>(payload_len));
  crc_input.insert(crc_input.end(), payload, payload + payload_len);

  const uint16_t crc = drivelink_hardware::crc16(crc_input.data(), crc_input.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
  return frame;
}

bool write_all(int fd, const std::vector<uint8_t> & bytes)
{
  size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t n = ::write(fd, bytes.data() + offset, bytes.size() - offset);
    if (n <= 0) {
      return false;
    }
    offset += static_cast<size_t>(n);
  }
  return true;
}

bool approx_equal(float a, float b, float eps = 1e-6F)
{
  return std::fabs(a - b) <= eps;
}

void print_case(const std::string & name, bool pass)
{
  std::cout << name << ": " << (pass ? "PASS" : "FAIL") << std::endl;
}

}  // namespace

int main()
{
  bool all_pass = true;

  int fds[2] = {-1, -1};
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
    std::cerr << "socketpair failed" << std::endl;
    return 1;
  }

  drivelink_hardware::DriveLinkComm comm("/dev/null", 115200);
  comm.inject_test_fd(fds[0]);

  std::atomic<bool> callback_fired{false};
  drivelink_hardware::MotorStateTelemetry captured{};

  comm.set_motor_state_callback([&](const drivelink_hardware::MotorStateTelemetry & msg) {
    captured = msg;
    callback_fired.store(true);
  });

  const bool open_ok = comm.open();
  print_case("Open with injected fd", open_ok);
  all_pass = all_pass && open_ok;

  const drivelink_hardware::MotorStateTelemetry telemetry = {
    0x01, 0x01, 123456ULL, 2, 0x03, 1.5F, -2.25F};

  const auto valid_frame = make_frame(
    static_cast<uint8_t>(drivelink_hardware::MessageType::kMotorTelemetryStream),
    reinterpret_cast<const uint8_t *>(&telemetry), sizeof(telemetry));

  const bool wrote_valid = write_all(fds[1], valid_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const bool valid_callback =
    wrote_valid && callback_fired.load() && captured.motor_id == telemetry.motor_id &&
    captured.field_mask == telemetry.field_mask && approx_equal(captured.angle_rad, telemetry.angle_rad) &&
    approx_equal(captured.angular_velocity_rad_s, telemetry.angular_velocity_rad_s);
  print_case("Valid telemetry callback", valid_callback);
  all_pass = all_pass && valid_callback;

  callback_fired.store(false);
  auto bad_crc_frame = valid_frame;
  bad_crc_frame.back() ^= 0xFFU;
  const bool wrote_bad_crc = write_all(fds[1], bad_crc_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const bool bad_crc_rejected = wrote_bad_crc && !callback_fired.load() && comm.is_open();
  print_case("CRC mismatch rejected", bad_crc_rejected);
  all_pass = all_pass && bad_crc_rejected;

  const drivelink_hardware::MotorSetpointCmd setpoint = {
    0x01,
    0x01,
    {2, static_cast<uint8_t>(drivelink_hardware::MotorSetpointKind::kVelocityRadS), 3.5F}};

  const bool send_ok = comm.send_setpoint(setpoint);

  uint8_t rx_buf[64];
  const ssize_t read_n = ::read(fds[1], rx_buf, sizeof(rx_buf));
  bool send_parse_ok = false;
  if (send_ok && read_n > 0) {
    const size_t n = static_cast<size_t>(read_n);
    if (
      n >= 7 + sizeof(setpoint) && rx_buf[0] == kDriveLinkStartByte &&
      rx_buf[1] == kDriveLinkStartByte && rx_buf[2] == kDriveLinkStartByte &&
      rx_buf[3] == static_cast<uint8_t>(drivelink_hardware::MessageType::kMotorSetpointStream) &&
      rx_buf[4] == sizeof(setpoint))
    {
      drivelink_hardware::MotorSetpointCmd parsed{};
      std::memcpy(&parsed, rx_buf + 5, sizeof(parsed));
      send_parse_ok =
        parsed.version == setpoint.version && parsed.entry_count == setpoint.entry_count &&
        parsed.entry.motor_id == setpoint.entry.motor_id &&
        parsed.entry.setpoint_kind == setpoint.entry.setpoint_kind &&
        approx_equal(parsed.entry.value, setpoint.entry.value);
    }
  }
  print_case("Setpoint send/parse", send_parse_ok);
  all_pass = all_pass && send_parse_ok;

  const drivelink_hardware::TimeSyncRequestPayload time_sync_req = {
    0x01,
    0x00,
    0x1234,
    1700000000000000LL};
  const auto time_sync_frame = make_frame(
    static_cast<uint8_t>(drivelink_hardware::MessageType::kTimeSyncRequest),
    reinterpret_cast<const uint8_t *>(&time_sync_req), sizeof(time_sync_req));

  const bool wrote_time_sync = write_all(fds[1], time_sync_frame);

  uint8_t ts_rx_buf[96];
  const ssize_t ts_read_n = ::read(fds[1], ts_rx_buf, sizeof(ts_rx_buf));
  bool time_sync_ok = false;
  if (wrote_time_sync && ts_read_n > 0) {
    const size_t n = static_cast<size_t>(ts_read_n);
    if (
      n >= 7 + sizeof(drivelink_hardware::TimeSyncResponsePayload) &&
      ts_rx_buf[0] == kDriveLinkStartByte && ts_rx_buf[1] == kDriveLinkStartByte &&
      ts_rx_buf[2] == kDriveLinkStartByte &&
      ts_rx_buf[3] == static_cast<uint8_t>(drivelink_hardware::MessageType::kTimeSyncResponse) &&
      ts_rx_buf[4] == sizeof(drivelink_hardware::TimeSyncResponsePayload))
    {
      drivelink_hardware::TimeSyncResponsePayload response{};
      std::memcpy(&response, ts_rx_buf + 5, sizeof(response));
      time_sync_ok =
        response.version == time_sync_req.version && response.status == 0 &&
        response.request_id == time_sync_req.request_id &&
        response.echoed_client_unix_send_us == time_sync_req.client_unix_send_us &&
        response.server_unix_tx_us >= response.server_unix_rx_us;
    }
  }
  print_case("Time sync request/response", time_sync_ok);
  all_pass = all_pass && time_sync_ok;

  comm.close();
  ::close(fds[1]);

  std::cout << (all_pass ? "PASS" : "FAIL") << std::endl;
  return all_pass ? 0 : 1;
}
