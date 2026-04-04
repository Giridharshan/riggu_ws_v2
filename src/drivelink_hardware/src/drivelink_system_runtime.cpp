#include "drivelink_hardware/drivelink_system.hpp"

#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "drivelink_hardware/constants.hpp"
#include "drivelink_hardware/drivelink_comm_registry.hpp"

namespace drivelink_hardware
{

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
    double eps = constants::kVelocitySetpointEpsilon;
    MotorSetpointKind kind = MotorSetpointKind::kVelocityRadS;

    if (joint.active_mode == MotorControlMode::kVelocity) {
      cmd_value = joint.cmd_velocity;
      kind = MotorSetpointKind::kVelocityRadS;
      eps = constants::kVelocitySetpointEpsilon;
    } else if (joint.active_mode == MotorControlMode::kPosition) {
      cmd_value = joint.cmd_position;
      kind = MotorSetpointKind::kPositionRad;
      eps = constants::kPositionSetpointEpsilon;
    } else {
      cmd_value = joint.cmd_effort;
      kind = MotorSetpointKind::kDuty;
      eps = constants::kEffortSetpointEpsilon;
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
