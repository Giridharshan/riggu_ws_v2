#pragma once

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"

#include "drivelink_hardware/drivelink_comm.hpp"

namespace drivelink_hardware
{

class DriveLinkImuSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct ImuSnapshot
  {
    double orientation_x{0.0};
    double orientation_y{0.0};
    double orientation_z{0.0};
    double orientation_w{1.0};

    double angular_velocity_x{0.0};
    double angular_velocity_y{0.0};
    double angular_velocity_z{0.0};

    double linear_acceleration_x{0.0};
    double linear_acceleration_y{0.0};
    double linear_acceleration_z{0.0};
  };

  bool validate_required_interfaces(const hardware_interface::ComponentInfo & sensor) const;
  bool extract_required_param(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    std::string & out) const;
  bool parse_int_param(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    int & out) const;
  bool parse_double_param(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    double & out) const;

  void parse_imu_telemetry(const std::vector<uint8_t> & payload);
  void release_comm();

  std::string sensor_name_;
  std::string uart_port_;
  int uart_baud_{115200};
  int imu_stream_period_ms_{10};
  uint16_t imu_stream_field_mask_{0};
  double imu_gyro_scale_{1.0};
  double imu_accel_scale_{1.0};

  std::shared_ptr<DriveLinkComm> comm_;

  mutable std::mutex state_mutex_;
  ImuSnapshot imu_snapshot_;

  double orientation_x_{0.0};
  double orientation_y_{0.0};
  double orientation_z_{0.0};
  double orientation_w_{1.0};
  double angular_velocity_x_{0.0};
  double angular_velocity_y_{0.0};
  double angular_velocity_z_{0.0};
  double linear_acceleration_x_{0.0};
  double linear_acceleration_y_{0.0};
  double linear_acceleration_z_{0.0};
};

}  // namespace drivelink_hardware
