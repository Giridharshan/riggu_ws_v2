# ros2_control Framework Notes (Source-Cited)

This document captures the ros2_control contracts that the DriveLink host plugin must obey, with explicit source citations.

## 1. Core Hardware API Contracts

### 1.1 Base class and lifecycle contract

All hardware plugins derive from `hardware_interface::HardwareComponentInterface` (directly or via `SystemInterface` / `SensorInterface`) and participate in ROS 2 lifecycle transitions (`on_init`, `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown`, `on_error`).

Sources:
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/hardware_component_interface.hpp
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/system_interface.hpp
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/sensor_interface.hpp
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html

### 1.2 `SystemInterface` and `SensorInterface` specifics

`SystemInterface` is intended for multi-joint systems and requires `write()` implementation (it remains pure virtual there).

`SensorInterface` provides no command interfaces and final `write()` returns `OK`; state-only sensor plugins should normally use this.

Sources:
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/system_interface.hpp
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/sensor_interface.hpp

### 1.3 Current rolling/Jazzy-style init and mode-switch signatures

From current `hardware_component_interface.hpp`:

- `virtual CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params);`
- `virtual return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces);`
- `virtual return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces);`
- `virtual return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;`
- `virtual return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);`

Source:
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/hardware_component_interface.hpp

### 1.4 Distro/API differences that matter

Humble-era plugins commonly use:
- `on_init(const hardware_interface::HardwareInfo & info)`
- old-style `export_state_interfaces()` and `export_command_interfaces()` returning value vectors

Example (third-party plugin):
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/hardware/include/robotiq_hande_driver/hande_hardware_interface.hpp

The official writing guide explicitly warns some scripts/tutorial scaffolding are Foxy/Humble-oriented and not compatible with Jazzy+ API shape.

Source:
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

## 2. How `HardwareInfo` should be used in `on_init`

Expected pattern:
1. Call parent `on_init(...)` first.
2. Parse and validate required parameters/joints/interfaces from `info_` / `get_hardware_info()`.
3. Return `CallbackReturn::ERROR` or `FAILURE` on invalid config.

Evidence:
- Guide recommends parent `on_init` call and parameter validation in `on_init`.
- Third-party example calls parent first and pulls hardware parameters from `info_.hardware_parameters`.

Sources:
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/hardware/src/hande_hardware_interface.cpp

## 3. Command-Mode Switching Semantics

### 3.1 What each callback means

- `prepare_command_mode_switch(...)`: non-realtime feasibility/preparation stage.
- `perform_command_mode_switch(...)`: realtime-path switch execution, should be fast.

Source:
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/hardware_component_interface.hpp

### 3.2 Where Controller Manager calls them

Controller Manager calls `resource_manager_->prepare_command_mode_switch(...)` in switch setup, and `resource_manager_->perform_command_mode_switch(...)` in switch execution (`manage_switch`) and failure rollback paths.

Source:
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/controller_manager/src/controller_manager.cpp

### 3.3 Order relative to controller lifecycle

In switch flow, command mode switch in hardware occurs before deactivation/activation bookkeeping is finalized in `manage_switch()`. The code path also includes rollback calls if activation fails under strictness constraints.

Source:
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/controller_manager/src/controller_manager.cpp

## 4. Realtime vs Non-Realtime Expectations

### 4.1 Realtime-critical loop

Controller Manager `update_rate` defines the realtime loop that performs: hardware `read` -> controller `update` -> hardware `write`.

Source:
- https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html

### 4.2 Non-realtime operations

Lifecycle transitions (`on_configure`, `on_activate`, etc.) and plugin loading/configuration are non-realtime control-plane operations.

Sources:
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

### 4.3 Thread-safety implications

Because mode switching and fallback handling can execute in/near the main control loop and induce jitter, hardware implementations should:
- avoid blocking calls in `read`/`write`/`perform_command_mode_switch`
- keep cross-thread data exchange lock-minimal and bounded

Sources:
- https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/hardware_component_interface.hpp

## 5. Plugin Registration and Package Wiring

### 5.1 CMake essentials

Typical hardware plugin package uses:
- shared library target
- dependency discovery (`hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, etc.)
- `pluginlib_export_plugin_description_file(hardware_interface <plugin_xml>)`
- install/export targets

Evidence (third-party):
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/CMakeLists.txt

### 5.2 Plugin XML shape

Canonical XML form:

- `<library path="...">`
- `<class name="pkg/PluginName" type="namespace::Class" base_class_type="hardware_interface::SystemInterface|SensorInterface|ActuatorInterface">`

Evidence:
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/ros2_control_plugin.xml

### 5.3 Source export macro

At source tail, plugin exports class via:
- `PLUGINLIB_EXPORT_CLASS(<ImplClass>, hardware_interface::SystemInterface)`

Evidence:
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/hardware/src/hande_hardware_interface.cpp
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

### 5.4 package.xml essentials

Package should include build tool and plugin dependencies (`ament_cmake`, `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`) and export build type.

Evidence:
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/package.xml
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

## 6. Controller Interface Expectations Relevant to DriveLink

### 6.1 DiffDrive wheel interfaces

`diff_drive_controller` commands wheel `velocity` interfaces (`<wheel>/velocity`) and uses state feedback either from position or velocity depending on `position_feedback`.

Sources:
- https://raw.githubusercontent.com/ros-controls/ros2_controllers/master/diff_drive_controller/src/diff_drive_controller.cpp
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp

### 6.2 IMU broadcaster required state interface names

Canonical IMU semantic interface names are:
- `orientation.x`, `orientation.y`, `orientation.z`, `orientation.w`
- `angular_velocity.x`, `angular_velocity.y`, `angular_velocity.z`
- `linear_acceleration.x`, `linear_acceleration.y`, `linear_acceleration.z`

Sources:
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/controller_interface/include/semantic_components/imu_sensor.hpp
- https://raw.githubusercontent.com/ros-controls/ros2_controllers/master/imu_sensor_broadcaster/src/imu_sensor_broadcaster.cpp

## 7. Shared Communication Object Across System + Sensor Plugins

### 7.1 Why shared comm is needed for DriveLink

DriveLink firmware exposes both wheel/motor control and IMU over one UART framing/protocol stack. A single serialized transport with coherent parser/dispatcher is required to avoid interleaved framing conflicts.

Firmware evidence:
- `components/uart_comm/*`
- `components/motor_manager/*`
- `components/imu_bmo_055/*`
- detailed in `01_firmware_api_reference.md`

### 7.2 Recommended ownership model

Use one process-wide transport manager object (shared pointer) that owns:
- serial FD/port and framing state
- RX thread/parser and callback dispatch
- request/response correlation and stream fanout

Then inject lightweight handles into:
- `DriveLinkSystemHardware` (`SystemInterface`) for motor command/state
- `DriveLinkImuSensorHardware` (`SensorInterface`) for IMU state

Synchronization model:
- transport internal mutex(es) for write serialization and callback maps
- lock-free or bounded-lock queues for state handoff into realtime `read`/`write`

Rationale from ros2_control threading guidance:
- keep realtime methods fast/non-blocking
- isolate blocking serial I/O to background thread

Sources:
- https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/asynchronous_components.html
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/hardware/src/hande_hardware_interface.cpp

⚠ VERIFY: exact lifetime boundary if controller manager loads/unloads only one of the two plugins first; design should keep transport alive until last user releases it.

## 8. Logging and Error-Handling Guidance for Hardware Plugins

### 8.1 Logging approach

Use `RCLCPP_*` logs in lifecycle callbacks and fault paths; include enough context for diagnosis (interface names, IDs, request IDs). Third-party examples log lifecycle transitions and exceptions explicitly.

Sources:
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/hardware/src/hande_hardware_interface.cpp
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

### 8.2 Serial unavailable during `on_configure`

Preferred handling is retry with bounded attempts, then return `CallbackReturn::FAILURE` from `on_configure` (not crash), allowing lifecycle retry/recovery flows.

Evidence:
- Robotiq example retries and returns `FAILURE` when configuration cannot be established.

Source:
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/hardware/src/hande_hardware_interface.cpp

## 9. Distro-Compatibility Guidance for This Project

Given current ros2_control docs/source are rolling-style while many examples online are Humble-oriented:
- pin target distro first (likely Humble/Jazzy depending host environment)
- align API style (old export interfaces + `HardwareInfo` init vs new interface-description API)
- do not mix signatures across distro boundaries

Sources:
- https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
- https://raw.githubusercontent.com/ros-controls/ros2_control/master/hardware_interface/include/hardware_interface/hardware_component_interface.hpp
- https://raw.githubusercontent.com/AGH-CEAI/robotiq_hande_driver/humble-devel/robotiq_hande_driver/hardware/include/robotiq_hande_driver/hande_hardware_interface.hpp

⚠ VERIFY: final target ROS 2 distro before coding; this single decision changes class signatures and export patterns throughout the implementation.
