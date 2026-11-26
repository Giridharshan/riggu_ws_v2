# 7. Things That Can Be Improved (Engineering Roadmap)

This document captures concrete hardware, firmware, ROS 2, and DevOps improvements identified from building and debugging Riggu.

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

## 2. Physical Encoders, PID Tuning & Hardware Safety

* **Verify & Secure Quadrature Encoder Wiring:** Ensure both encoders (`GPIO 4/5` Left, `GPIO 7/6` Right) consistently publish ticks on `/joint_states` (`position` and `velocity` must never freeze at `[0.0, 0.0]`), and calibrate exact `counts_per_rev` + `gear_ratio` with a measured 1-meter roll test (`odom` distance error < 2%).
* **Tune Closed-Loop PID + Feedforward:** Once encoders are verified on both wheels, tune `vel_kp`, `vel_ki`, `vel_kd`, and add non-zero `velocity_feedforward` in `robot.urdf.xacro` so slow Nav2 speeds (`0.15 m/s`) start smoothly without deadband lag.
* **Firmware Watchdog & Battery Telemetry:**
  * Add an automatic setpoint timeout inside `motor_manager` (e.g., stop motors if no `0x80` packet arrives for 500 ms) as a second layer below `diff_drive_controller`'s timeout.
  * Read motor battery voltage via an ESP32 ADC pin and publish `sensor_msgs/BatteryState` so low-voltage brownouts are caught before SD card corruption or NACK `0x02` faults occur.

---

## 3. Persistent USB Port Mapping (`udev` Rules) & Power Stability

* **Add Linux `udev` Rules on the Pi:** Map the ESP32-S3 and Slamtec SLLidar C1 by USB vendor/product/serial ID to permanent symlinks (e.g., `/dev/riggu_esp` and `/dev/riggu_lidar`) with `MODE="0666"`. This prevents `/dev/ttyACM0`/`ttyUSB0` swaps on reboot and eliminates running `sudo chmod 666`.
* **Power Isolation for Raspberry Pi 5:** Power the Pi 5 from a dedicated high-current 5V/5A regulated buck converter separate from motor power spikes to prevent under-voltage brownouts and SD card corruption.

---

## 4. Nav2 Performance & Safety Tuning for Raspberry Pi 5

* **Reduce Pi 5 CPU Load:**
  * Lower `planner_server`'s `expected_planner_frequency` (e.g., `1.0–2.0 Hz` instead of `5.0 Hz`) and `global_costmap` update rate (`1.0 Hz` or `0.5 Hz`) to eliminate `Planner loop missed its desired rate` and `compute_path_to_pose` action timeouts.
  * Replace the generic `nav2_bringup/bringup_launch.py` include with a slim custom launch file that omits unused servers (`docking_server`, `route_server`, `waypoint_follower`).
* **Switch from `robot_radius` to Exact Rectangular `footprint`:** Riggu's acrylic frame is rectangular (`0.33 m x 0.30 m`). Defining `footprint` in `local_costmap` / `global_costmap` instead of a circular `robot_radius` allows tighter doorway clearance while maintaining safe side/front margins.
* **Add `PolygonSlow` in `collision_monitor`:** Add an outer slowdown zone ahead of `PolygonStop` so Riggu decelerates smoothly before entering an emergency stop, and tune `PolygonStop` carefully so chassis corners/wires never trigger false stops.

---

## 5. Mechanical Alignment (Front vs Back & LiDAR Position)

* **Mount the LiDAR Physically at the Front:** If the LiDAR remains at the rear edge (`x = -0.01`), consider physically mounting it near the front edge and updating `lidar_mount_joint` in `robot.urdf.xacro` (remembering no re-mapping is needed when only URDF offsets change).
* **Align `robot.urdf.xacro` Visual/Collision Meshes:** Ensure visual chassis plates, wheel offsets (`±0.15 m`), and caster location accurately reflect the physical frame for clean RViz2 visualization.

---

## 6. Codebase & Git Hygiene

* **Run `drivelink_hardware` Tests in CI:** Automate `colcon test --packages-select drivelink_hardware` (`test_crc16`, `test_comm`, `test_plugin_list`, `test_system_lifecycle`) and `check_urdf` via GitHub Actions.
* **Finish `archive/REFACTOR_LOG.md` Cleanups:** Split `include/drivelink_hardware/drivelink_messages.hpp` (333 lines) into focused headers under `include/drivelink_hardware/protocol/`, gate test targets cleanly under `BUILD_TESTING`, and manage external `sllidar_ros2` via a `.repos` file instead of vendoring it in `src/`.
