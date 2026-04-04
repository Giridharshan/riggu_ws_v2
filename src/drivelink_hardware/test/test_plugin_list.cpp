#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{

bool contains(const std::vector<std::string> & values, const std::string & target)
{
  return std::find(values.begin(), values.end(), target) != values.end();
}

}  // namespace

int main()
{
  try {
    pluginlib::ClassLoader<hardware_interface::SystemInterface> system_loader(
      "hardware_interface", "hardware_interface::SystemInterface");
    pluginlib::ClassLoader<hardware_interface::SensorInterface> sensor_loader(
      "hardware_interface", "hardware_interface::SensorInterface");

    const auto system_classes = system_loader.getDeclaredClasses();
    const auto sensor_classes = sensor_loader.getDeclaredClasses();

    std::cout << "System classes:" << std::endl;
    for (const auto & cls : system_classes) {
      std::cout << "  " << cls << std::endl;
    }

    std::cout << "Sensor classes:" << std::endl;
    for (const auto & cls : sensor_classes) {
      std::cout << "  " << cls << std::endl;
    }

    const bool system_ok = contains(system_classes, "drivelink_hardware::DriveLinkSystem") ||
      contains(system_classes, "drivelink_hardware/DriveLinkSystem");
    const bool imu_ok = contains(sensor_classes, "drivelink_hardware::DriveLinkImuSensor") ||
      contains(sensor_classes, "drivelink_hardware/DriveLinkImuSensor");

    std::cout << ((system_ok && imu_ok) ? "PASS" : "FAIL") << std::endl;
    return (system_ok && imu_ok) ? 0 : 1;
  } catch (const std::exception & ex) {
    std::cerr << "Plugin list check failed: " << ex.what() << std::endl;
    return 1;
  }
}
