#include "drivelink_hardware/drivelink_imu_sensor.hpp"

#include <cstring>

#include "drivelink_hardware/constants.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drivelink_hardware
{
namespace
{

int16_t read_i16(const std::vector<uint8_t> & payload, size_t offset)
{
  int16_t value = 0;
  std::memcpy(&value, payload.data() + offset, sizeof(value));
  return value;
}

}  // namespace

void DriveLinkImuSensor::parse_imu_telemetry(const std::vector<uint8_t> & payload)
{
  if (payload.size() < sizeof(ImuTelemetryHeader)) {
    return;
  }

  uint16_t field_mask = 0;
  std::memcpy(&field_mask, payload.data() + 1, sizeof(field_mask));
  size_t offset = sizeof(ImuTelemetryHeader);

  ImuSnapshot snapshot = imu_snapshot_;

  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kEuler)) != 0U) {
    offset += 6;
  }

  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kQuaternion)) != 0U) {
    if (offset + 8 > payload.size()) {
      return;
    }
    const int16_t w = read_i16(payload, offset + 0);
    const int16_t x = read_i16(payload, offset + 2);
    const int16_t y = read_i16(payload, offset + 4);
    const int16_t z = read_i16(payload, offset + 6);
    snapshot.orientation_w = static_cast<double>(w) * constants::kImuQuaternionScale;
    snapshot.orientation_x = static_cast<double>(x) * constants::kImuQuaternionScale;
    snapshot.orientation_y = static_cast<double>(y) * constants::kImuQuaternionScale;
    snapshot.orientation_z = static_cast<double>(z) * constants::kImuQuaternionScale;
    offset += 8;
  }
	
  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kLinearAccel)) != 0U) {
    if (offset + 6 > payload.size()) {
      return;
    }
    snapshot.linear_acceleration_x = static_cast<double>(read_i16(payload, offset + 0)) * imu_accel_scale_;
    snapshot.linear_acceleration_y = static_cast<double>(read_i16(payload, offset + 2)) * imu_accel_scale_;
    snapshot.linear_acceleration_z = static_cast<double>(read_i16(payload, offset + 4)) * imu_accel_scale_;
    offset += 6;
  }

  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kGravity)) != 0U) {
    offset += 6;
  }

  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kRawAccel)) != 0U) {
    offset += 6;
  }

  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kRawGyro)) != 0U) {
    if (offset + 6 > payload.size()) {
      return;
    }
    snapshot.angular_velocity_x = static_cast<double>(read_i16(payload, offset + 0)) * imu_gyro_scale_;
    snapshot.angular_velocity_y = static_cast<double>(read_i16(payload, offset + 2)) * imu_gyro_scale_;
    snapshot.angular_velocity_z = static_cast<double>(read_i16(payload, offset + 4)) * imu_gyro_scale_;
    offset += 6;
  }

  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kRawMag)) != 0U) {
    offset += 6;
  }

  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kTemperature)) != 0U) {
    offset += 1;
  }

  if ((field_mask & static_cast<uint16_t>(ImuStreamFieldMask::kCalibrationStatus)) != 0U) {
    offset += 4;
  }

  if (offset > payload.size()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  imu_snapshot_ = snapshot;
}

}  // namespace drivelink_hardware
