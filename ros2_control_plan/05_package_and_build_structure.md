# Package and Build Structure

Goal: define a clean ROS 2 package layout and build wiring for implementing DriveLink ros2_control hardware plugins.

## 1. Recommended Workspace Layout

Create a dedicated ROS 2 workspace (outside ESP-IDF firmware tree), for example:

```text
<host_ros2_ws>/
  src/
    drivelink_ros2_control/
      CMakeLists.txt
      package.xml
      ros2_control_plugin.xml
      include/drivelink_ros2_control/
        drivelink_system_hardware.hpp
        drivelink_imu_sensor_hardware.hpp
        drivelink_client_registry.hpp
        drivelink_comm/
          client.hpp
          codec.hpp
          protocol.hpp
      src/
        drivelink_system_hardware.cpp
        drivelink_imu_sensor_hardware.cpp
        drivelink_client_registry.cpp
        drivelink_comm/
          client.cpp
          codec.cpp
          parser.cpp
          serial_port.cpp
      config/
        drivelink_controllers.yaml
      urdf/
        drivelink_ros2_control.urdf.xacro
      launch/
        drivelink_control.launch.py
      test/
        test_codec.cpp
        test_parser.cpp
        test_load_hardware_plugin.cpp
      README.md
```

## 2. Package Split Recommendation

Single package is acceptable for first implementation:
- `drivelink_ros2_control` containing both comm + plugins.

Optional later split:
1. `drivelink_comm` (pure transport/protocol library)
2. `drivelink_ros2_control` (plugins depending on `drivelink_comm`)

Use split only if reuse beyond ros2_control is expected.

## 3. `package.xml` Requirements

Minimum dependencies:

- Build tool: `ament_cmake`
- Runtime/build: `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, `controller_manager`
- Controller side: `diff_drive_controller`, `imu_sensor_broadcaster`, `joint_state_broadcaster` (for deployment, typically not direct code dependency)
- Tests: `ament_cmake_gtest` or `ament_cmake_gmock`, `ros2_control_test_assets`

Add export:

```xml
<export>
  <build_type>ament_cmake</build_type>
</export>
```

## 4. `CMakeLists.txt` Requirements

### 4.1 Core build

1. `find_package(...)` for all required dependencies.
2. Build shared library target with all plugin + comm sources.
3. `ament_target_dependencies(...)` on core ROS deps.
4. `target_compile_features(... cxx_std_17)`.

### 4.2 Plugin export and install

Must include:

```cmake
pluginlib_export_plugin_description_file(hardware_interface ros2_control_plugin.xml)
```

Install rules:
- headers to `include`
- library to `lib`
- plugin XML and config/urdf/launch to `share/${PROJECT_NAME}`

Export rules:
- `ament_export_targets(... HAS_LIBRARY_TARGET)`
- `ament_export_dependencies(...)`

### 4.3 Suggested quality flags

- `-Wall -Wextra -Wpedantic`
- optionally `-Werror` after first stabilization

## 5. `ros2_control_plugin.xml` Content

Declare two classes in one library:

```xml
<library path="drivelink_ros2_control">
  <class
    name="drivelink_ros2_control/DriveLinkSystemHardware"
    type="drivelink_ros2_control::DriveLinkSystemHardware"
    base_class_type="hardware_interface::SystemInterface">
    <description>DriveLink system hardware plugin</description>
  </class>

  <class
    name="drivelink_ros2_control/DriveLinkImuSensorHardware"
    type="drivelink_ros2_control::DriveLinkImuSensorHardware"
    base_class_type="hardware_interface::SensorInterface">
    <description>DriveLink IMU sensor hardware plugin</description>
  </class>
</library>
```

## 6. Source File Conventions

### 6.1 Plugin source

Each plugin `.cpp` must end with pluginlib export macro:

```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(drivelink_ros2_control::DriveLinkSystemHardware, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(drivelink_ros2_control::DriveLinkImuSensorHardware, hardware_interface::SensorInterface)
```

### 6.2 Comm layer source

Keep transport/protocol free of ROS control-loop details. ROS logging usage is acceptable, but avoid direct controller dependencies.

## 7. Config and Launch Assets

### 7.1 Controller YAML (`config/drivelink_controllers.yaml`)

Should define:
- `controller_manager.update_rate`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `imu_sensor_broadcaster`

### 7.2 URDF/Xacro

Define two `<ros2_control>` hardware blocks if separate plugins are used, with shared serial params and explicit interface declarations.

### 7.3 Launch

`drivelink_control.launch.py` should:
1. start `robot_state_publisher`
2. start `ros2_control_node`
3. spawn broadcasters/controllers in safe order

Recommended spawn order:
1. `joint_state_broadcaster`
2. `imu_sensor_broadcaster`
3. `diff_drive_controller`

## 8. Test Layout

1. Unit tests (no hardware)
- codec/parser/correlation

2. Plugin loading test
- verifies plugin name from XML loads via pluginlib

3. Optional integration test
- pseudo-tty simulated firmware responder

4. HIL smoke script
- manual command sequence against real ESP board

## 9. CI and Build Commands

### 9.1 Local build

```bash
colcon build --packages-select drivelink_ros2_control
```

### 9.2 Local test

```bash
colcon test --packages-select drivelink_ros2_control
colcon test-result --verbose
```

### 9.3 Runtime

```bash
source install/setup.bash
ros2 launch drivelink_ros2_control drivelink_control.launch.py
```

## 10. Versioning and Compatibility

1. Pin target ROS distro in README and CI matrix.
2. Keep one branch per distro if API differences become large.
3. Guard API differences in source where practical.

⚠ VERIFY final target distro before creating package skeleton; Humble vs Jazzy/Rolling affects method signatures and export style.
