#include "drivelink_hardware/drivelink_imu_sensor.hpp"

#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "drivelink_hardware/drivelink_comm_registry.hpp"

namespace drivelink_hardware
{
namespace
{

bool has_state_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  for (const auto & interface : interfaces) {
    if (interface.name == name) {
      return true;
    }
  }
  return false;
}

}  // namespace

hardware_interface::CallbackReturn DriveLinkImuSensor::on_init(
  const hardware_interface::HardwareInfo & info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto parent_init = hardware_interface::SensorInterface::on_init(info);
#pragma GCC diagnostic pop
  if (parent_init != hardware_interface::CallbackReturn::SUCCESS) {
    return parent_init;
  }

  if (info.sensors.empty()) {
    RCLCPP_ERROR(get_logger(), "DriveLinkImuSensor requires one sensor in hardware info.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & sensor_info = info.sensors.front();
  const std::string urdf_sensor_name = sensor_info.name;

  if (!validate_required_interfaces(sensor_info)) {
    RCLCPP_ERROR(
      get_logger(),
      "Sensor '%s' must declare all 10 IMU semantic state interfaces.",
      sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string param_sensor_name;
  if (!extract_required_param(info.hardware_parameters, "sensor_name", param_sensor_name)) {
    RCLCPP_ERROR(get_logger(), "Missing required param sensor_name in hardware parameters.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (param_sensor_name != urdf_sensor_name) {
    RCLCPP_ERROR(
      get_logger(),
      "sensor_name param ('%s') does not match URDF sensor name ('%s').",
      param_sensor_name.c_str(), urdf_sensor_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  sensor_name_ = urdf_sensor_name;

  if (!parse_int_param(info.hardware_parameters, "imu_stream_period_ms", imu_stream_period_ms_)) {
    RCLCPP_ERROR(get_logger(), "Missing or invalid required param imu_stream_period_ms.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!extract_required_param(info.hardware_parameters, "uart_port", uart_port_) ||
    !parse_int_param(info.hardware_parameters, "uart_baud", uart_baud_))
  {
    RCLCPP_ERROR(get_logger(), "Missing required shared UART params: uart_port, uart_baud.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  parse_double_param(info.hardware_parameters, "imu_gyro_scale", imu_gyro_scale_);
  parse_double_param(info.hardware_parameters, "imu_accel_scale", imu_accel_scale_);

  imu_stream_field_mask_ =
    static_cast<uint16_t>(
    static_cast<uint16_t>(ImuStreamFieldMask::kQuaternion) |
    static_cast<uint16_t>(ImuStreamFieldMask::kLinearAccel) |
    static_cast<uint16_t>(ImuStreamFieldMask::kRawGyro));

  comm_ = DriveLinkCommRegistry::acquire(uart_port_, uart_baud_);
  if (!comm_) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to acquire shared DriveLinkComm for IMU on %s:%d.",
      uart_port_.c_str(), uart_baud_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DriveLinkImuSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(10);

  interfaces.emplace_back(sensor_name_, "orientation.x", &orientation_x_);
  interfaces.emplace_back(sensor_name_, "orientation.y", &orientation_y_);
  interfaces.emplace_back(sensor_name_, "orientation.z", &orientation_z_);
  interfaces.emplace_back(sensor_name_, "orientation.w", &orientation_w_);

  interfaces.emplace_back(sensor_name_, "angular_velocity.x", &angular_velocity_x_);
  interfaces.emplace_back(sensor_name_, "angular_velocity.y", &angular_velocity_y_);
  interfaces.emplace_back(sensor_name_, "angular_velocity.z", &angular_velocity_z_);

  interfaces.emplace_back(sensor_name_, "linear_acceleration.x", &linear_acceleration_x_);
  interfaces.emplace_back(sensor_name_, "linear_acceleration.y", &linear_acceleration_y_);
  interfaces.emplace_back(sensor_name_, "linear_acceleration.z", &linear_acceleration_z_);

  return interfaces;
}

hardware_interface::CallbackReturn DriveLinkImuSensor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!comm_) {
    comm_ = DriveLinkCommRegistry::acquire(uart_port_, uart_baud_);
    if (!comm_) {
      RCLCPP_ERROR(get_logger(), "Unable to acquire shared comm in IMU on_configure.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  comm_->set_imu_telemetry_callback(
    [this](const std::vector<uint8_t> & payload) { parse_imu_telemetry(payload); });

  if (!comm_->open()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to open serial port '%s' in IMU on_configure.",
      uart_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!comm_->send_imu_stream_cfg(true, static_cast<uint16_t>(imu_stream_period_ms_), imu_stream_field_mask_)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure IMU stream command sequence.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkImuSensor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkImuSensor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (comm_ && comm_->is_open()) {
    comm_->send_imu_stream_cfg(false, static_cast<uint16_t>(imu_stream_period_ms_), imu_stream_field_mask_);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkImuSensor::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_comm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkImuSensor::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_comm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkImuSensor::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_comm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DriveLinkImuSensor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!state_mutex_.try_lock()) {
    return hardware_interface::return_type::OK;
  }

  orientation_x_ = imu_snapshot_.orientation_x;
  orientation_y_ = imu_snapshot_.orientation_y;
  orientation_z_ = imu_snapshot_.orientation_z;
  orientation_w_ = imu_snapshot_.orientation_w;

  angular_velocity_x_ = imu_snapshot_.angular_velocity_x;
  angular_velocity_y_ = imu_snapshot_.angular_velocity_y;
  angular_velocity_z_ = imu_snapshot_.angular_velocity_z;

  linear_acceleration_x_ = imu_snapshot_.linear_acceleration_x;
  linear_acceleration_y_ = imu_snapshot_.linear_acceleration_y;
  linear_acceleration_z_ = imu_snapshot_.linear_acceleration_z;

  state_mutex_.unlock();
  return hardware_interface::return_type::OK;
}

bool DriveLinkImuSensor::validate_required_interfaces(const hardware_interface::ComponentInfo & sensor) const
{
  return
    has_state_interface(sensor.state_interfaces, "orientation.x") &&
    has_state_interface(sensor.state_interfaces, "orientation.y") &&
    has_state_interface(sensor.state_interfaces, "orientation.z") &&
    has_state_interface(sensor.state_interfaces, "orientation.w") &&
    has_state_interface(sensor.state_interfaces, "angular_velocity.x") &&
    has_state_interface(sensor.state_interfaces, "angular_velocity.y") &&
    has_state_interface(sensor.state_interfaces, "angular_velocity.z") &&
    has_state_interface(sensor.state_interfaces, "linear_acceleration.x") &&
    has_state_interface(sensor.state_interfaces, "linear_acceleration.y") &&
    has_state_interface(sensor.state_interfaces, "linear_acceleration.z");
}

bool DriveLinkImuSensor::extract_required_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  std::string & out) const
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return false;
  }
  out = it->second;
  return true;
}

bool DriveLinkImuSensor::parse_int_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  int & out) const
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }
  try {
    out = std::stoi(it->second);
    return true;
  } catch (...) {
    return false;
  }
}

bool DriveLinkImuSensor::parse_double_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double & out) const
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }
  try {
    out = std::stod(it->second);
    return true;
  } catch (...) {
    return false;
  }
}

void DriveLinkImuSensor::release_comm()
{
  if (comm_) {
    comm_->set_imu_telemetry_callback(nullptr);
    DriveLinkCommRegistry::release(comm_);
    comm_.reset();
  }
}

}  // namespace drivelink_hardware

PLUGINLIB_EXPORT_CLASS(drivelink_hardware::DriveLinkImuSensor, hardware_interface::SensorInterface)
