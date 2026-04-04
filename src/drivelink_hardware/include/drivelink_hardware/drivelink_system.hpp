#pragma once

#include <chrono>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "drivelink_hardware/constants.hpp"
#include "drivelink_hardware/drivelink_comm.hpp"

namespace drivelink_hardware
{

/**
 * @brief ROS 2 control system plugin for DriveLink motor joints.
 *
 * Owns joint configuration parsing, lifecycle transitions, command-mode switching, and runtime
 * read/write interactions with firmware through DriveLinkComm.
 */
class DriveLinkSystem : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Initialize hardware configuration from URDF hardware info.
   * @param info Hardware component metadata and parameters.
   * @return SUCCESS when all required parameters/interfaces are valid.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Export joint state interfaces.
   * @return Vector of exported state interfaces.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export joint command interfaces.
   * @return Vector of exported command interfaces.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Configure communication and firmware-side motor configuration.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when configuration completes.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate command streaming and initialize runtime state.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when activation completes.
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate command streaming and stop motors safely.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when deactivation completes.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Cleanup resources after deactivation.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when cleanup completes.
   */
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Shutdown the hardware interface and release resources.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS when shutdown completes.
   */
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Handle error transition by stopping motors and releasing resources.
   * @param previous_state Previous lifecycle state.
   * @return SUCCESS after error handling.
   */
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Publish latest motor states into exported state interfaces.
   * @param time Current controller time.
   * @param period Control period.
   * @return OK when read step is complete.
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Push commands from exported command interfaces to firmware.
   * @param time Current controller time.
   * @param period Control period.
   * @return OK on successful command write cycle, ERROR otherwise.
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Validate and stage command-mode transitions.
   * @param start_interfaces Interfaces requested to start.
   * @param stop_interfaces Interfaces requested to stop.
   * @return OK when requested transitions are valid.
   */
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  /**
   * @brief Execute staged command-mode transitions on firmware.
   * @param start_interfaces Interfaces requested to start.
   * @param stop_interfaces Interfaces requested to stop.
   * @return OK when transitions are successfully applied.
   */
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
  int uart_baud_{constants::kDefaultUartBaud};
  int telemetry_period_ms_{constants::kDefaultTelemetryPeriodMs};

  std::shared_ptr<DriveLinkComm> comm_;

  mutable std::mutex state_mutex_;
  mutable std::mutex response_mutex_;
  std::condition_variable response_cv_;
  std::optional<MotorConfigureResponse> last_motor_response_;

  std::unordered_map<std::string, MotorControlMode> pending_mode_changes_;
  uint64_t write_failure_count_{0};
};

}  // namespace drivelink_hardware
