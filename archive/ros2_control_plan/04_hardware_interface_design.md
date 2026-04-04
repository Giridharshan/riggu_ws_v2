# Hardware Interface Design (ros2_control Plugin Layer)

Goal: map DriveLink firmware capabilities to ros2_control `SystemInterface` + `SensorInterface` plugins with deterministic behavior and clean lifecycle semantics.

## 1. Plugin Topology

Two plugins in one ROS 2 package:

1. `drivelink_ros2_control/DriveLinkSystemHardware`
- Base: `hardware_interface::SystemInterface`
- Responsibility: motor command/state surfaces for controllers like `diff_drive_controller`

2. `drivelink_ros2_control/DriveLinkImuSensorHardware`
- Base: `hardware_interface::SensorInterface`
- Responsibility: IMU state interfaces for `imu_sensor_broadcaster`

Shared dependency:
- both plugins depend on a shared `std::shared_ptr<DriveLinkClient>` from Phase 3.

## 2. URDF/ros2_control Interface Mapping

### 2.1 Wheel joints (System plugin)

Per drive wheel joint expose:
- command interface: `velocity`
- state interfaces: at least `position` and `velocity`

Rationale:
- `diff_drive_controller` claims command `HW_IF_VELOCITY` and uses state feedback from `HW_IF_POSITION` or `HW_IF_VELOCITY` depending on `position_feedback`.

### 2.2 IMU sensor (Sensor plugin)

Expose IMU state interfaces exactly as semantic component expects:
- `orientation.x`, `orientation.y`, `orientation.z`, `orientation.w`
- `angular_velocity.x`, `angular_velocity.y`, `angular_velocity.z`
- `linear_acceleration.x`, `linear_acceleration.y`, `linear_acceleration.z`

## 3. Internal Class Design

### 3.1 `DriveLinkSystemHardware`

State members:
- wheel command buffers: `cmd_vel_left`, `cmd_vel_right`
- wheel state buffers: `state_pos_left/right`, `state_vel_left/right`
- optional per-motor diagnostics and freshness timestamps
- `std::shared_ptr<DriveLinkClient> client_`
- mapping config `joint_name -> motor_id`
- mode config (`velocity control` expected for diff drive)

Core responsibilities by lifecycle:

1. `on_init`
- parse and validate hardware params
- validate wheel/motor mapping and joint interface declarations
- create or attach shared client handle

2. `on_configure`
- open/start transport if owner
- issue motor create/configure sequence
- set telemetry masks and stream enable
- set control mode to velocity where needed

3. `on_activate`
- clear stale command/state
- optionally reset motor controllers
- mark command path active

4. `read`
- pull latest telemetry from client cache
- integrate position from velocity if needed when firmware angle unavailable for a wheel
- update exported state interface values only

5. `write`
- translate wheel velocity commands to firmware setpoint semantics
- publish batched setpoints over channel `0x80` preferred
- fallback to configure command `SetSetpoint` on stream path failure

6. `on_deactivate`
- send stop per motor
- disable command output

7. `on_cleanup` / `on_shutdown`
- release ownership and optionally close transport

### 3.2 `DriveLinkImuSensorHardware`

State members:
- imu state interface backing fields (10 doubles)
- conversion settings / frame config
- `std::shared_ptr<DriveLinkClient> client_`

Lifecycle:

1. `on_init`
- parse IMU params, sensor name, conversion policy
- attach to shared client

2. `on_configure`
- configure IMU stream mask and period via firmware `0x22`

3. `on_activate`
- enable IMU stream

4. `read`
- copy latest IMU cache into interface fields
- preserve last valid sample if fresh sample missing (bounded stale tolerance)

5. `on_deactivate`
- disable IMU stream

6. `on_cleanup` / `on_shutdown`
- detach from client

## 4. Shared Transport Ownership Strategy

Implement a process-local singleton/factory:

```cpp
class DriveLinkClientRegistry {
public:
  static std::shared_ptr<DriveLinkClient> acquire(const CommConfig& cfg);
  static void release(const std::shared_ptr<DriveLinkClient>& client);
};
```

Behavior:
- first plugin acquisition starts transport
- reference count tracks users
- final release stops transport

Why:
- avoids double-opening same serial port
- ensures both plugins observe coherent parser state

⚠ VERIFY behavior when Controller Manager transitions only one plugin repeatedly while the other remains active.

## 5. Command Mode and Controller Switching

### 5.1 Expected control mode for this robot

For diff drive, operational mode should be velocity control in firmware (`SetMode=velocity`).

### 5.2 `prepare_command_mode_switch` policy

In system plugin:
- inspect requested interfaces (`start_interfaces`, `stop_interfaces`)
- allow switches among known wheel velocity interfaces
- reject unsupported command mode combinations

Return:
- `OK` for supported transitions
- `ERROR` for invalid transitions

### 5.3 `perform_command_mode_switch` policy

- execute minimal state transition work
- avoid serial blocking calls if possible (precompute in prepare)
- if firmware mode flip is required, keep bounded timeout and return `ERROR` on failure

## 6. Data Conversion and Kinematics Rules

### 6.1 Motor telemetry to ros2 state

From firmware telemetry:
- angle rad -> joint position
- angular velocity rad/s -> joint velocity

If wheel direction inversion needed, apply sign per-joint config.

### 6.2 IMU raw conversion

Firmware can stream raw integer fields; host should convert to SI for ros2 interfaces.

Recommended config parameters:
- `imu_use_raw_quaternion` (bool)
- `imu_gyro_scale` (double)
- `imu_accel_scale` (double)
- `imu_frame_id` (string)

⚠ VERIFY conversion constants against BNO055 mode and vendor scaling tables before release.

## 7. Realtime Safety Rules

In `read`/`write`:
- no dynamic allocation
- no long mutex hold
- no serial I/O blocking

Use only cached values and preallocated command buffers.

Non-realtime work belongs in:
- lifecycle callbacks
- background comm threads in `drivelink_comm`

## 8. Failure and Recovery Policy

### 8.1 Transport failures

If transport disconnected:
- `write` returns `OK` or `ERROR` based on strictness mode chosen for safety policy
- states become stale with diagnostics flags
- lifecycle may trigger `on_error` path if configured to hard-fail

### 8.2 Firmware result-code failures

Map firmware `ResultCode` to plugin errors:
- `InvalidArgument`, `NotSupported` -> configuration error (`FAILURE` in configure path)
- `InternalError`, repeated timeout -> runtime fault (`ERROR`)

### 8.3 Safe stop behavior

On deactivate/error:
- send `StopMotor` for every mapped motor
- optionally send zero velocity batch
- mark command interfaces inactive

## 9. Diagnostics and Status Publishing

Recommended diagnostics fields:
- transport connected state
- last command RTT
- motor telemetry freshness per motor
- IMU freshness
- CRC/frame error counters

If implementing framework hardware status publisher:
- pre-allocate message in `init_hardware_status_message`
- fill dynamic values in `update_hardware_status_message`

## 10. Parameter Schema (Proposed)

System plugin params:
- `serial_port` (string)
- `serial_baud` (int, default 115200)
- `motor_ids` (array<int>)
- `left_wheel_joint` / `right_wheel_joint` mapping
- `setpoint_publish_hz` (double)
- `control_mode` (string: velocity/open_loop)
- `telemetry_field_mask` (int)

IMU plugin params:
- `imu_stream_period_ms` (int)
- `imu_stream_field_mask` (int)
- `imu_operation_mode` (int)
- `imu_frame_id` (string)
- conversion scale params as above

Shared params (must match across both plugins):
- `serial_port`
- `serial_baud`

⚠ VERIFY at runtime that both plugin instances receive identical shared params; fail fast if mismatched.

## 11. Integration with Controllers

Minimum controller set:

1. `joint_state_broadcaster`
- publishes wheel + imu state interfaces

2. `diff_drive_controller`
- wheel velocity command and wheel state feedback

3. `imu_sensor_broadcaster`
- publishes `sensor_msgs/Imu` from semantic IMU interfaces

## 12. Implementation Notes for Agent

1. Start with `DriveLinkSystemHardware` only and validate diff-drive loop.
2. Add `DriveLinkImuSensorHardware` using same client registry.
3. Add strict parameter validation and deterministic errors before optimization.
4. Only after stable behavior, enable optional async improvements.
