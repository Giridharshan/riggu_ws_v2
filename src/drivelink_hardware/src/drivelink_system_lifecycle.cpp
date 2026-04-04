#include "drivelink_hardware/drivelink_system.hpp"

#include <sstream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "drivelink_hardware/constants.hpp"
#include "drivelink_hardware/drivelink_comm_registry.hpp"

namespace drivelink_hardware
{

hardware_interface::CallbackReturn DriveLinkSystem::on_init(const hardware_interface::HardwareInfo & info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto parent_init = hardware_interface::SystemInterface::on_init(info);
#pragma GCC diagnostic pop
  if (parent_init != hardware_interface::CallbackReturn::SUCCESS) {
    return parent_init;
  }

  std::string uart_baud_str;
  std::string telemetry_period_str;
  if (!extract_required_param(info.hardware_parameters, "uart_port", uart_port_) ||
    !extract_required_param(info.hardware_parameters, "uart_baud", uart_baud_str) ||
    !extract_required_param(info.hardware_parameters, "telemetry_period_ms", telemetry_period_str))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Missing required hardware params. Required: uart_port, uart_baud, telemetry_period_ms.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    uart_baud_ = std::stoi(uart_baud_str);
    telemetry_period_ms_ = std::stoi(telemetry_period_str);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to parse numeric hardware params: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info.joints.size());

  for (const auto & joint_info : info.joints) {
    if (
      !has_interface(joint_info.command_interfaces, hardware_interface::HW_IF_VELOCITY) ||
      !has_interface(joint_info.command_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint_info.command_interfaces, hardware_interface::HW_IF_EFFORT) ||
      !has_interface(joint_info.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint_info.state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
      !has_interface(joint_info.state_interfaces, hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' must declare command interfaces {position, velocity, effort} and state interfaces {position, velocity, effort}.",
        joint_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    uint8_t motor_id = 0;
    if (!parse_uint8_param(joint_info.parameters, "motor_id", motor_id))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' missing required param: motor_id.",
        joint_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    MotorCreateOrConfigureBody create_config{};
    create_config.gear_ratio = 1.0F;
    create_config.output_min = -1.0F;
    create_config.output_max = 1.0F;
    create_config.sample_time_ms = static_cast<uint32_t>(telemetry_period_ms_);

    double counts_per_rev = 0.0;
    const bool has_counts_per_rev =
      parse_double_param(joint_info.parameters, "counts_per_rev", counts_per_rev);
    const bool has_legacy_ticks_per_rev =
      parse_double_param(joint_info.parameters, "ticks_per_rev", counts_per_rev);
    if (!has_counts_per_rev && !has_legacy_ticks_per_rev) {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' missing required param: counts_per_rev (legacy alias: ticks_per_rev).",
        joint_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    create_config.counts_per_rev = static_cast<float>(counts_per_rev);

    {
      uint8_t parsed = 0;
      if (parse_uint8_param(joint_info.parameters, "pwm_gpio", parsed)) {
        create_config.pwm_gpio = parsed;
      }
      if (parse_uint8_param(joint_info.parameters, "dir_gpio", parsed)) {
        create_config.dir_gpio = parsed;
      }
      if (parse_uint8_param(joint_info.parameters, "enc_a", parsed)) {
        create_config.enc_a = parsed;
      }
      if (parse_uint8_param(joint_info.parameters, "enc_b", parsed)) {
        create_config.enc_b = parsed;
      }
    }

    {
      double parsed = 0.0;
      if (parse_double_param(joint_info.parameters, "gear_ratio", parsed)) {
        create_config.gear_ratio = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "output_min", parsed)) {
        create_config.output_min = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "output_max", parsed)) {
        create_config.output_max = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "vel_kp", parsed)) {
        create_config.vel_kp = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "vel_ki", parsed)) {
        create_config.vel_ki = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "vel_kd", parsed)) {
        create_config.vel_kd = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "pos_kp", parsed)) {
        create_config.pos_kp = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "pos_ki", parsed)) {
        create_config.pos_ki = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "pos_kd", parsed)) {
        create_config.pos_kd = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "velocity_feedforward", parsed)) {
        create_config.velocity_feedforward = static_cast<float>(parsed);
      }
      if (parse_double_param(joint_info.parameters, "position_feedforward", parsed)) {
        create_config.position_feedforward = static_cast<float>(parsed);
      }
    }

    JointContext joint;
    joint.name = joint_info.name;
    joint.motor_id = motor_id;
    joint.motor_create_config = create_config;
    joints_.emplace_back(joint);
  }

  comm_ = DriveLinkCommRegistry::acquire(uart_port_, uart_baud_);
  if (!comm_) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to acquire shared DriveLinkComm for %s:%d. Check shared UART params across plugins.",
      uart_port_.c_str(), uart_baud_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DriveLinkSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joints_.size() * 3);
  for (auto & joint : joints_) {
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.state_position);
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state_velocity);
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_EFFORT, &joint.state_effort);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DriveLinkSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joints_.size() * 3);
  for (auto & joint : joints_) {
    command_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.cmd_velocity);
    command_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.cmd_position);
    command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.cmd_effort);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DriveLinkSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!comm_) {
    comm_ = DriveLinkCommRegistry::acquire(uart_port_, uart_baud_);
    if (!comm_) {
      RCLCPP_ERROR(get_logger(), "Unable to acquire shared comm object in on_configure.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  comm_->set_motor_state_callback(
    [this](const MotorStateTelemetry & telemetry) { on_motor_state_telemetry(telemetry); });
  comm_->set_motor_response_callback(
    [this](const MotorConfigureResponse & response) { on_motor_configure_response(response); });

  if (!comm_->open()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to open serial port '%s' in on_configure. Verify the device path and permissions.",
      uart_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : joints_) {
    MotorCreateOrConfigureBody body = joint.motor_create_config;
    if (!comm_->send_motor_create(joint.motor_id, body)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to send motor create command for joint '%s' (motor_id=%u).",
        joint.name.c_str(), joint.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!comm_->send_motor_telemetry_global(
      true,
      static_cast<uint8_t>(
        static_cast<uint8_t>(MotorTelemetryFieldMask::kAngle) |
        static_cast<uint8_t>(MotorTelemetryFieldMask::kVelocity))))
  {
    RCLCPP_ERROR(get_logger(), "Failed to configure motor telemetry stream.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_joint_states();
  mark_commands_nan();

  for (auto & joint : joints_) {
    if (!comm_ || !comm_->send_motor_set_mode(joint.motor_id, joint.active_mode)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to send MOTOR_SET_MODE for joint '%s' (motor_id=%u).",
        joint.name.c_str(), joint.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }

    MotorResultCode result = MotorResultCode::kInternalError;
    const bool got_response = wait_for_motor_response(
      static_cast<uint8_t>(MotorConfigureSubcommand::kSetMode), joint.motor_id,
      constants::kModeSwitchResponseTimeout, result);
    if (!got_response) {
      RCLCPP_ERROR(
        get_logger(),
        "Timed out waiting for SetMode response for joint '%s' (motor_id=%u).",
        joint.name.c_str(), joint.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (result != MotorResultCode::kOk) {
      RCLCPP_ERROR(
        get_logger(), "Firmware NACK for SetMode on joint '%s' (motor_id=%u, result=0x%02X).",
        joint.name.c_str(), joint.motor_id, static_cast<unsigned int>(result));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  safe_stop_all_motors();
  mark_commands_nan();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_comm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  safe_stop_all_motors();
  release_comm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DriveLinkSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  safe_stop_all_motors();
  release_comm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool DriveLinkSystem::extract_required_param(
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

bool DriveLinkSystem::parse_uint8_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  uint8_t & out) const
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }

  try {
    const int parsed = std::stoi(it->second);
    if (parsed < 0 || parsed > 255) {
      return false;
    }
    out = static_cast<uint8_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool DriveLinkSystem::parse_double_param(
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

bool DriveLinkSystem::has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name) const
{
  for (const auto & interface : interfaces) {
    if (interface.name == name) {
      return true;
    }
  }
  return false;
}

}  // namespace drivelink_hardware
