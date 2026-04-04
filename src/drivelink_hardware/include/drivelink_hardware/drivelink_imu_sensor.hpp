#pragma once

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"

#include "drivelink_hardware/constants.hpp"
#include "drivelink_hardware/drivelink_comm.hpp"

namespace drivelink_hardware
{

/**
 * @brief ROS 2 control sensor plugin for DriveLink IMU telemetry.
 *
 * Parses IMU stream payloads from DriveLinkComm and publishes quaternion, angular velocity,
 * and linear acceleration state interfaces.
 */
class DriveLinkImuSensor : public hardware_interface::SensorInterface
{
public:
  /**
   * @brief Initialize sensor from URDF hardware info and parameters.
   * @param info Hardware sensor metadata and parameters.
   * @return SUCCESS when required parameters/interfaces are valid.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Export sensor state interfaces.
   * @return Vector of exported IMU state interfaces.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Configure communication and IMU stream settings.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when configuration completes.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate IMU sensor interface.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when activation completes.
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate IMU stream activity.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when deactivation completes.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Cleanup shared communication resources.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when cleanup completes.
   */
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Shutdown sensor interface and release resources.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when shutdown completes.
   */
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Handle error transition and release communication resources.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS after error handling.
   */
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Publish latest decoded IMU snapshot to state interfaces.
   * @param time Current controller time.
   * @param period Control period.
   * @return OK when read step completes.
   */
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
  int uart_baud_{constants::kDefaultUartBaud};
  int imu_stream_period_ms_{constants::kDefaultImuStreamPeriodMs};
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
