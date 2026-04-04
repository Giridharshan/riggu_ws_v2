#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "drivelink_hardware/drivelink_comm.hpp"

namespace drivelink_hardware
{

class DriveLinkSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  struct JointContext
  {
    std::string name;
    uint8_t motor_id{0};
    MotorCreateOrConfigureBody motor_create_config{};

    double state_position{0.0};
    double state_velocity{0.0};
    double state_effort{0.0};

    double cmd_velocity{std::numeric_limits<double>::quiet_NaN()};
    double cmd_position{std::numeric_limits<double>::quiet_NaN()};
    double cmd_effort{std::numeric_limits<double>::quiet_NaN()};

    MotorControlMode active_mode{MotorControlMode::kVelocity};
    double last_sent_setpoint{std::numeric_limits<double>::quiet_NaN()};
  };

  bool extract_required_param(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    std::string & out) const;
  bool parse_uint8_param(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    uint8_t & out) const;
  bool parse_double_param(
    const std::unordered_map<std::string, std::string> & params, const std::string & key,
    double & out) const;
  bool has_interface(
    const std::vector<hardware_interface::InterfaceInfo> & interfaces,
    const std::string & name) const;

  JointContext * find_joint_by_name(const std::string & name);
  const JointContext * find_joint_by_name(const std::string & name) const;
  JointContext * find_joint_by_motor_id(uint8_t motor_id);

  void on_motor_state_telemetry(const MotorStateTelemetry & telemetry);
  void on_motor_configure_response(const MotorConfigureResponse & response);
  bool wait_for_motor_response(
    uint8_t subcommand, uint8_t motor_id, std::chrono::milliseconds timeout,
    MotorResultCode & result);

  void mark_commands_nan();
  void reset_joint_states();
  void safe_stop_all_motors();
  void release_comm();

  std::vector<JointContext> joints_;

  std::string uart_port_;
  int uart_baud_{115200};
  int telemetry_period_ms_{100};

  std::shared_ptr<DriveLinkComm> comm_;

  mutable std::mutex state_mutex_;
  mutable std::mutex response_mutex_;
  std::condition_variable response_cv_;
  std::optional<MotorConfigureResponse> last_motor_response_;

  std::unordered_map<std::string, MotorControlMode> pending_mode_changes_;
  uint64_t write_failure_count_{0};
};

}  // namespace drivelink_hardware
