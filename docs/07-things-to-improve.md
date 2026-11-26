# 7. Things That Can Be Improved (Future Roadmap)

This document captures key architectural and navigation improvements planned for **Riggu**.

---

## 1. Re-Integrate the BNO055 IMU Back onto the ESP32-S3 Cleanly

Currently, the BNO055 runs on Raspberry Pi I2C (`/dev/i2c-1` via `bno055_pi.py`) because hosting it on the ESP32 caused crashes and orientation issues. Moving it back onto the ESP32 frees Pi GPIO/CPU and provides tighter hardware-timestamped fusion (`time_sync` + IMU on one MCU) **if these 5 fixes are applied first**:

1. **Fix the Operation Mode Value (`0x08` vs `0x0C`):**
   * In `firmware/drivelink_v2/main/main.cpp` and `sdkconfig.defaults`, `default_operation_mode` is set to `0x0C` with a comment claiming `IMUPLUS (fixes norm=0.44, disables mag)`.
   * Per `components/imu_bmo_055/include/bno055.h`, `0x0C` is actually `NDOF` (magnetometer ON), while **`0x08` is `BNO055_OPERATION_MODE_IMUPLUS`**. Changing to `0x08` eliminates indoor magnetic interference from Riggu's DC motors.
2. **Verify Raw Unit Conversions (`⚠ VERIFY` in `archive/IMPL_LOG.md`):**
   * Audit `drivelink_hardware/src/drivelink_imu_sensor_parser.cpp` against `components/imu_bmo_055/bno055_support.c` to ensure quaternion scaling (`1/16384`), gyroscope units (`rad/s` vs `deg/s`), and linear acceleration units (`m/s²`) match ROS `sensor_msgs/Imu` REP-145 conventions.
3. **Increase UART Baud Rate from `115200` to `921600`:**
   * As noted in `firmware/drivelink_v2/components/uart_comm/Kconfig`, `921600` baud is recommended for real-time motor + IMU streaming so `0x81` (motors) and `0x82` (IMU at 100 Hz) don't saturate `115200` baud.
4. **Decouple IMU Startup Failures from Killing `DriveLinkSystem`:**
   * In `DriveLinkImuSensor::on_configure()`, an IMU stream failure currently returns `CallbackReturn::ERROR`, which causes Jazzy's `controller_manager` to abort the entire `ros2_control_node` (killing wheel drive too). Make IMU stream failure warn/retry instead of crashing motor control, **or** keep IMU telemetry in a standalone non-`ros2_control` node.
5. **Disable Silent Mock Fallback in Production:**
   * Set `CONFIG_IMU_BMO_055_FALLBACK_TO_MOCK_ON_INIT_FAIL=n` on real hardware so a loose I2C wire (`GPIO 8/9`) reports an explicit error instead of silently feeding fake telemetry to the EKF.

---

## 2. Integrate BMS (Battery Management System) into Navigation

Right now, Nav2 has no awareness of battery state of charge (SoC) or voltage dips, and `docking_server` sits idle with dummy settings. Connecting a **BMS** directly into the navigation stack prevents unexpected mid-run brownouts and enables self-charging behavior:

1. **Publish Live Battery State (`sensor_msgs/msg/BatteryState`):**
   * Read pack voltage, current draw, and SoC from the BMS (via UART/I2C/CAN on the Pi or an ESP32 telemetry channel) and publish to `/battery_state`.
2. **Low-Battery Behavior Tree & Auto-Return:**
   * Add a battery-monitoring condition node (`IsBatteryLow`) inside `bt_navigator`'s Behavior Tree XML (or a Python supervisor node wrapping `NavigateToPose`).
   * When SoC drops below a configurable threshold (e.g., `20%`), automatically cancel or pause active navigation goals, navigate back to a saved home/charging pose, and trigger `docking_server` (`/dock_robot`) once a physical charging station is built.
3. **Critical Under-Voltage Safeguard:**
   * Add a lower hard threshold (e.g., `10%` or under-voltage alarm) that halts motion cleanly and alerts the operator before motor current spikes cause Pi 5 brownouts, SD card corruption, or ESP32 `NACK 0x02` lockouts.

---

## 3. Tune Navigation & Controller Parameters for Sharp Turns

In tight rooms, hallways, and $90^\circ$ corners, default trajectory settings can make a differential-drive robot hesitate, oscillate, overshoot turns, or graze corners. Tuning Nav2's planner and local controller parameters specifically for sharp turns will make cornering much cleaner:

1. **Tune `DWBLocalPlanner` Angular Dynamics & Critics (`controller_server` in `nav2_params.yaml`):**
   * **Allow crisp in-place / tight turns:** Tune `max_vel_theta`, `acc_lim_theta`, `decel_lim_theta`, and `min_theta_velocity_threshold` (with `min_speed_theta` tuned so motors overcome static friction when turning in place).
   * **Balance alignment vs forward progress (`critics`):** Increase weights on `RotateToGoal.scale`, `PathAlign.scale`, and `GoalAlign.scale` relative to `PathDist.scale`, and tune `forward_point_distance` so Riggu rotates cleanly onto sharp path bends before accelerating forward.
   * **Prevent progress-checker false aborts on turns:** Ensure `SimpleProgressChecker` (`required_movement_radius` / `movement_time_allowance`) gives enough time when Riggu rotates in place at a tight corner (note: rotating in place doesn't translate `(x, y)` distance).
2. **Consider `RegulatedPurePursuitController` (`nav2_regulated_pure_pursuit_controller`) + `SmacPlanner2D`:**
   * Switching from `DWBLocalPlanner` to **Regulated Pure Pursuit** with `use_rotate_to_heading: true` and `rotate_to_heading_min_angle` (e.g., `0.785 rad` / $45^\circ$) makes differential-drive robots **stop, pivot smoothly in place to face sharp corners, and scale down linear speed proportionally to curvature (`use_regulated_linear_velocity_scaling: true`)** — ideal for avoiding wall scrapes on tight turns.
3. **Match `velocity_smoother` & `costmap` Margins to Cornering:**
   * Ensure `velocity_smoother`'s `max_velocity` / `max_accel` / `max_decel` angular limits (`[vx, 0.0, omega]`) match `controller_server` so sharp turn commands aren't sluggishly clipped, and keep `inflation_radius` / `cost_scaling_factor` balanced so Riggu swings wide enough around doorways without cutting inside corners.
