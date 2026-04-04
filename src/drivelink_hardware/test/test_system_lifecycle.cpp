#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include "drivelink_hardware/drivelink_system.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  drivelink_hardware::DriveLinkSystem system;

  hardware_interface::HardwareInfo info;
  info.name = "drivelink_system";
  info.type = "system";
  info.hardware_parameters["uart_port"] = "/dev/null";
  info.hardware_parameters["uart_baud"] = "115200";
  info.hardware_parameters["telemetry_period_ms"] = "100";

  hardware_interface::ComponentInfo left_joint;
  left_joint.name = "left_wheel_joint";
  left_joint.type = "joint";
  left_joint.parameters["motor_id"] = "1";
  left_joint.parameters["ticks_per_rev"] = "2048";

  hardware_interface::InterfaceInfo iface;
  iface.name = "position";
  left_joint.command_interfaces.push_back(iface);
  left_joint.state_interfaces.push_back(iface);
  iface.name = "velocity";
  left_joint.command_interfaces.push_back(iface);
  left_joint.state_interfaces.push_back(iface);
  iface.name = "effort";
  left_joint.command_interfaces.push_back(iface);
  left_joint.state_interfaces.push_back(iface);

  hardware_interface::ComponentInfo right_joint = left_joint;
  right_joint.name = "right_wheel_joint";
  right_joint.parameters["motor_id"] = "2";

  info.joints = {left_joint, right_joint};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto init_ret = system.init(
    info, rclcpp::get_logger("test_system_lifecycle"), std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
#pragma GCC diagnostic pop
  const auto configure_ret = system.on_configure(rclcpp_lifecycle::State{});

  const bool pass =
    init_ret == hardware_interface::CallbackReturn::SUCCESS &&
    configure_ret == hardware_interface::CallbackReturn::ERROR;

  std::cout << (pass ? "PASS" : "FAIL") << std::endl;

  rclcpp::shutdown();
  return pass ? 0 : 1;
}
