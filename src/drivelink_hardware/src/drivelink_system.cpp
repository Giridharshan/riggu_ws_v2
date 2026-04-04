#include "drivelink_hardware/drivelink_system.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

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
    double ticks_per_rev = 0.0;
    if (!parse_uint8_param(joint_info.parameters, "motor_id", motor_id) ||
      !parse_double_param(joint_info.parameters, "ticks_per_rev", ticks_per_rev))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' missing required params: motor_id and ticks_per_rev.",
        joint_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    JointContext joint;
    joint.name = joint_info.name;
    joint.motor_id = motor_id;
    joint.ticks_per_rev = ticks_per_rev;
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
    MotorCreateOrConfigureBody body{};
    body.counts_per_rev = static_cast<float>(joint.ticks_per_rev);
    body.gear_ratio = 1.0F;
    body.output_min = -1.0F;
    body.output_max = 1.0F;
    body.sample_time_ms = static_cast<uint32_t>(telemetry_period_ms_);

    // ⚠ VERIFY: firmware protocol does not expose an explicit telemetry-period command for motors.
    // Assumption: sample_time_ms in create/configure body provides desired update cadence.
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
      std::chrono::milliseconds(500), result);
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

hardware_interface::return_type DriveLinkSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!state_mutex_.try_lock()) {
    return hardware_interface::return_type::OK;
  }
  state_mutex_.unlock();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DriveLinkSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comm_ || !comm_->is_open()) {
    ++write_failure_count_;
    if ((write_failure_count_ % 100U) == 0U) {
      RCLCPP_WARN(
        get_logger(),
        "DriveLink write failed %llu times because comm is not connected.",
        static_cast<unsigned long long>(write_failure_count_));
    }
    return hardware_interface::return_type::ERROR;
  }

  for (auto & joint : joints_) {
    double cmd_value = std::numeric_limits<double>::quiet_NaN();
    double eps = 1e-6;
    MotorSetpointKind kind = MotorSetpointKind::kVelocityRadS;

    if (joint.active_mode == MotorControlMode::kVelocity) {
      cmd_value = joint.cmd_velocity;
      kind = MotorSetpointKind::kVelocityRadS;
      eps = 1e-6;
    } else if (joint.active_mode == MotorControlMode::kPosition) {
      cmd_value = joint.cmd_position;
      kind = MotorSetpointKind::kPositionRad;
      eps = 1e-6;
    } else {
      cmd_value = joint.cmd_effort;
      kind = MotorSetpointKind::kDuty;
      eps = 1e-4;
    }

    if (std::isnan(cmd_value)) {
      continue;
    }

    if (!std::isnan(joint.last_sent_setpoint) && std::fabs(cmd_value - joint.last_sent_setpoint) <= eps) {
      continue;
    }

    if (!comm_->send_motor_setpoint(joint.motor_id, kind, static_cast<float>(cmd_value))) {
      ++write_failure_count_;
      if ((write_failure_count_ % 100U) == 0U) {
        RCLCPP_WARN(
          get_logger(),
          "DriveLink setpoint send failed %llu times (throttled every 100 failures).",
          static_cast<unsigned long long>(write_failure_count_));
      }
      return hardware_interface::return_type::ERROR;
    }

    joint.last_sent_setpoint = cmd_value;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DriveLinkSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  pending_mode_changes_.clear();

  for (const auto & iface : start_interfaces) {
    const auto slash_pos = iface.find('/');
    if (slash_pos == std::string::npos) {
      RCLCPP_ERROR(get_logger(), "Invalid interface format in mode switch: '%s'", iface.c_str());
      return hardware_interface::return_type::ERROR;
    }

    const std::string joint_name = iface.substr(0, slash_pos);
    const std::string interface_type = iface.substr(slash_pos + 1);

    auto * joint = find_joint_by_name(joint_name);
    if (joint == nullptr) {
      RCLCPP_ERROR(
        get_logger(), "Mode switch requested unknown joint '%s' from interface '%s'.",
        joint_name.c_str(), iface.c_str());
      return hardware_interface::return_type::ERROR;
    }

    if (interface_type == hardware_interface::HW_IF_VELOCITY) {
      pending_mode_changes_[joint_name] = MotorControlMode::kVelocity;
    } else if (interface_type == hardware_interface::HW_IF_POSITION) {
      pending_mode_changes_[joint_name] = MotorControlMode::kPosition;
    } else if (interface_type == hardware_interface::HW_IF_EFFORT) {
      pending_mode_changes_[joint_name] = MotorControlMode::kOpenLoop;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DriveLinkSystem::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  for (const auto & pending : pending_mode_changes_) {
    auto * joint = find_joint_by_name(pending.first);
    if (joint == nullptr) {
      RCLCPP_ERROR(
        get_logger(), "perform_command_mode_switch cannot find joint '%s'.", pending.first.c_str());
      return hardware_interface::return_type::ERROR;
    }

    if (!comm_ || !comm_->send_motor_set_mode(joint->motor_id, pending.second)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to send mode switch to joint '%s' (motor_id=%u).",
        joint->name.c_str(), joint->motor_id);
      return hardware_interface::return_type::ERROR;
    }

    joint->active_mode = pending.second;
    if (pending.second == MotorControlMode::kVelocity) {
      joint->cmd_position = std::numeric_limits<double>::quiet_NaN();
      joint->cmd_effort = std::numeric_limits<double>::quiet_NaN();
    } else if (pending.second == MotorControlMode::kPosition) {
      joint->cmd_velocity = std::numeric_limits<double>::quiet_NaN();
      joint->cmd_effort = std::numeric_limits<double>::quiet_NaN();
    } else {
      joint->cmd_velocity = std::numeric_limits<double>::quiet_NaN();
      joint->cmd_position = std::numeric_limits<double>::quiet_NaN();
    }
  }

  pending_mode_changes_.clear();
  return hardware_interface::return_type::OK;
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

DriveLinkSystem::JointContext * DriveLinkSystem::find_joint_by_name(const std::string & name)
{
  for (auto & joint : joints_) {
    if (joint.name == name) {
      return &joint;
    }
  }
  return nullptr;
}

const DriveLinkSystem::JointContext * DriveLinkSystem::find_joint_by_name(const std::string & name) const
{
  for (const auto & joint : joints_) {
    if (joint.name == name) {
      return &joint;
    }
  }
  return nullptr;
}

DriveLinkSystem::JointContext * DriveLinkSystem::find_joint_by_motor_id(uint8_t motor_id)
{
  for (auto & joint : joints_) {
    if (joint.motor_id == motor_id) {
      return &joint;
    }
  }
  return nullptr;
}

void DriveLinkSystem::on_motor_state_telemetry(const MotorStateTelemetry & telemetry)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto * joint = find_joint_by_motor_id(telemetry.motor_id);
  if (joint == nullptr) {
    return;
  }

  if ((telemetry.field_mask & static_cast<uint8_t>(MotorTelemetryFieldMask::kAngle)) != 0U) {
    joint->state_position = telemetry.angle_rad;
  }
  if ((telemetry.field_mask & static_cast<uint8_t>(MotorTelemetryFieldMask::kVelocity)) != 0U) {
    joint->state_velocity = telemetry.angular_velocity_rad_s;
  }
  joint->state_effort = 0.0;
}

void DriveLinkSystem::on_motor_configure_response(const MotorConfigureResponse & response)
{
  std::lock_guard<std::mutex> lock(response_mutex_);
  last_motor_response_ = response;
  response_cv_.notify_all();
}

bool DriveLinkSystem::wait_for_motor_response(
  uint8_t subcommand, uint8_t motor_id, std::chrono::milliseconds timeout, MotorResultCode & result)
{
  std::unique_lock<std::mutex> lock(response_mutex_);
  const bool received = response_cv_.wait_for(lock, timeout, [&]() {
    if (!last_motor_response_.has_value()) {
      return false;
    }
    return last_motor_response_->subcommand_echo == subcommand &&
           last_motor_response_->motor_id == motor_id;
  });

  if (!received) {
    return false;
  }

  result = static_cast<MotorResultCode>(last_motor_response_->result_code);
  last_motor_response_.reset();
  return true;
}

void DriveLinkSystem::mark_commands_nan()
{
  for (auto & joint : joints_) {
    joint.cmd_velocity = std::numeric_limits<double>::quiet_NaN();
    joint.cmd_position = std::numeric_limits<double>::quiet_NaN();
    joint.cmd_effort = std::numeric_limits<double>::quiet_NaN();
    joint.last_sent_setpoint = std::numeric_limits<double>::quiet_NaN();
  }
}

void DriveLinkSystem::reset_joint_states()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (auto & joint : joints_) {
    joint.state_position = 0.0;
    joint.state_velocity = 0.0;
    joint.state_effort = 0.0;
  }
}

void DriveLinkSystem::safe_stop_all_motors()
{
  if (!comm_ || !comm_->is_open()) {
    return;
  }

  for (const auto & joint : joints_) {
    MotorSetpointKind setpoint_kind = MotorSetpointKind::kVelocityRadS;
    if (joint.active_mode == MotorControlMode::kPosition) {
      setpoint_kind = MotorSetpointKind::kPositionRad;
    } else if (joint.active_mode == MotorControlMode::kOpenLoop) {
      setpoint_kind = MotorSetpointKind::kDuty;
    }

    if (!comm_->send_motor_setpoint(joint.motor_id, setpoint_kind, 0.0F)) {
      RCLCPP_WARN(
        get_logger(), "Failed to send zero setpoint for joint '%s' (motor_id=%u).",
        joint.name.c_str(), joint.motor_id);
    }
    comm_->send_motor_stop(joint.motor_id);
  }
}

void DriveLinkSystem::release_comm()
{
  if (comm_) {
    comm_->set_motor_state_callback(nullptr);
    comm_->set_motor_response_callback(nullptr);
    DriveLinkCommRegistry::release(comm_);
    comm_.reset();
  }
}

}  // namespace drivelink_hardware

PLUGINLIB_EXPORT_CLASS(drivelink_hardware::DriveLinkSystem, hardware_interface::SystemInterface)
