# 6. Technical Issues & Fixes (Engineering Log)

A technical reference of the hardware, ROS 2 Jazzy, SLAM, and Nav2 problems encountered while building **Riggu** and how each was resolved.

---

## Deep Dive: What Happened With the BNO055 IMU on the ESP32?

Originally, `drivelink_v2` was built to host the Bosch **BNO055 IMU directly on the ESP32-S3** (`I2C0`, `GPIO 8` SDA / `GPIO 9` SCL, address `0x28`), stream IMU packets (`0x82`) over UART (`/dev/ttyACM0`), and feed ROS 2 via `drivelink_hardware/DriveLinkImuSensor` (`imu_broadcaster`).

In testing, that architecture suffered from four issues:

1. **Operation Mode Bug (`0x0C` vs `0x08` / `norm=0.44`):**
   `firmware/drivelink_v2/main/main.cpp` and `sdkconfig.defaults` configured:
   ```cpp
   imu_cfg.default_operation_mode = 0x0C; // BNO055_OPERATION_MODE_IMUPLUS (fixes norm=0.44, disables mag)
   ```
   However, per `components/imu_bmo_055/include/bno055.h`, **`0x0C` is `NDOF` (magnetometer enabled)**, whereas **`0x08` is `IMUPLUS`**. Running `NDOF` near DC motors causes magnetic distortion and poor quaternion normalization, while quaternion/gyro/accel host scaling (`drivelink_imu_sensor_parser.cpp`) still had open verification checks.
2. **Cold-Boot I2C Timeouts (`ESP_ERR_TIMEOUT`):**
   Early BNO055 page-ID I2C transactions intermittently timed out during startup (`bno055_device.cpp`), requiring retry logic, and `CONFIG_IMU_BMO_055_FALLBACK_TO_MOCK_ON_INIT_FAIL=y` risked masking real sensor failures with synthetic data.
3. **Coupled Crash Failure (`ros2_control_node` `exit code -6`):**
   Because ROS 2 Jazzy's `ControllerManager::set_initial_hardware_components_state()` throws an unhandled exception whenever any hardware plugin returns `CallbackReturn::ERROR` in `on_configure()`, any failure in `DriveLinkImuSensor` (`send_imu_stream_cfg` failure or port error) crashed the entire `ros2_control_node` and took down wheel drive too.
4. **Shared `115200` Baud UART Contention:**
   Both motor telemetry/setpoints (`0x80`/`0x81`) and high-rate IMU telemetry (`0x82`) shared one `115200` baud link through `DriveLinkCommRegistry`.

### Resolution Taken
We moved the IMU off the ESP32 onto the **Raspberry Pi 5 hardware I2C bus (`/dev/i2c-1`: GPIO 2 SDA, GPIO 3 SCL)**:
1. `bno055_pi.py` runs as an independent ROS 2 node publishing `/bno055/imu` at **50.0 Hz** in NDOF mode (`0x0C`).
2. `<ros2_control name="DriveLinkImuSensor">` is commented out in `robot.urdf.xacro` so `DriveLinkSystem` has exclusive ownership of `/dev/ttyACM0`, while `ekf_node` fuses `/bno055/imu` with wheel odometry.

---

## Phase 1: SLAM & EKF Bringup

### 1. Real-Hardware Clock Freeze (`use_sim_time: true`)
* **Symptom:** `slam_toolbox` and TF lookups reject real LiDAR scans (`Failed to compute odom pose`).
* **Cause:** With `use_sim_time: true`, nodes wait for `/clock` from Gazebo and keep internal time frozen at `0.0` while real hardware timestamps are Unix time.
* **Fix:** Set `use_sim_time: false` across `ekf.yaml`, `slam_toolbox.yaml`, and `nav2_params.yaml`.

### 2. EKF Update Overload, Clock Skew & Scan Extrapolation
* **Symptoms:**
  * `[ekf_node]: Failed to meet update rate! Took 0.0611 seconds.`
  * `[velocity_controller]: Ignoring received message because it is older than the current time by 1.282 seconds`
  * `[rviz]: Message Filter dropping message: frame 'laser' ... queue is full`
  * `Lookup would require extrapolation into the future (8.6ms)`
* **Cause:** At 30 Hz (33 ms budget), EKF's 61 ms compute cycle fell behind real time; additionally, Raspberry Pi and Workstation clocks were skewed by 1.28 s.
* **Fix:**
  1. Lower `frequency: 15.0` in `config/ekf.yaml`.
  2. Set `predict_to_current_time: true` and `transform_time_offset: 0.05` in `config/ekf.yaml` so `odom -> base_link` stays valid ahead of incoming `/scan` stamps.
  3. Synchronize Pi and Workstation clocks via NTP or `date -s`.

### 3. `slam_toolbox` Ignoring Config (`base_footprint` Fallback)
* **Symptom:** `Failed to compute odom pose` even with `base_frame: base_link` in `slam_toolbox.yaml`; `ros2 param get /slam_toolbox base_frame` returned `base_footprint`.
* **Cause:** `slam_toolbox/launch/online_async_launch.py` reads the **`slam_params_file`** launch argument, not `params_file`.
* **Fix:** Pass `'slam_params_file': ...` in `launch/slam.launch.py`.

---

## Phase 2: Motors, Encoders & `ros2_control`

### 4. `ros2_control_node` Crashes (`exit code -6` / `SIGABRT`)
* **Symptom:** `ros2_control_node` aborts inside `set_initial_hardware_components_state()` at startup (`open(/dev/ttyACM0) failed`) or dies mid-run (`0.0 Hz`, `exit code -6`).
* **Cause:** Active `DriveLinkImuSensor` plugin failing over `/dev/ttyACM0`, serial port disconnect/brownout during motor load, or missing `dialout` permissions.
* **Fix:** Keep `DriveLinkImuSensor` disabled when using Pi I2C, ensure persistent `dialout` permissions, reset the ESP32, and kill stale `ros2_control_node` instances before relaunching.

### 5. `Firmware NACK for SetMode (result=0x02)`
* **Symptom:** `Firmware NACK for SetMode on right_wheel_joint (motor_id=2, result=0x02)` during hardware activation.
* **Cause:** ESP32 safety/watchdog lockout triggered after a sudden Pi reboot, serial disconnect, or motor power brownout.
* **Fix:** Verify motor bus power is ON and press **`EN` / `RST`** on the ESP32-S3 before launching `drivelink.launch.py`.

### 6. Zero Encoder Feedback (`/joint_states` = `[0.0, 0.0]`) & Bang-Bang PID Saturation
* **Symptoms:** `/joint_states` position stays `[0.0, 0.0]` and `/odometry/filtered` stays `0.0` during manual wheel rotation; motors jump to full speed on any joystick input and cut immediately to `0` when idle, or one wheel spins forward regardless of command.
* **Cause:** Disconnected/loose quadrature encoder signals (`GPIO 4/5` Left, `GPIO 7/6` Right) or swapped PWM/DIR wires. With feedback stuck at `0.0`, the ESP32 velocity PID (`kp=0.6`) saturates to `1.0` (100% duty) for any non-zero setpoint and resets output to `0` at `0.0`.
* **Fix:** Verify encoder `VCC`/`GND`/`A`/`B` and PWM/DIR wiring and confirm changing positions with `ros2 topic echo /joint_states --field position`.

---

## Phase 3: Raspberry Pi 5 OS, Storage & RP1 Bus Recovery

### 7. MicroSD Filesystem Corruption (`Input/output error`)
* **Symptom:** Commands fail with `-bash: /opt/ros/jazzy/bin/ros2: Input/output error`.
* **Cause:** Under-voltage power dip or dislodged MicroSD card forcing FAT32/ext4 errors.
* **Fix:** Repair both partitions (`fsck.vfat` / `fsck.ext4`) on a Linux PC and ensure stable 5V power.

### 8. Lost `/dev/i2c-1` and All USB Ports (`brcm-pcie` Failure on Pi 5)
* **Symptoms:** `FileNotFoundError: '/dev/i2c-1'`, missing `/dev/ttyACM0` & `/dev/ttyUSB0`, blank `lsusb`, and `dmesg` showing `brcm-pcie: Unable to find MSI PCI address`.
* **Cause:** Overwriting `/boot/firmware/bcm2712-rpi-5-b.dtb` with a Raspberry Pi OS DTB breaks the Ubuntu 24.04 (`6.8.0-raspi`) PCIe link to the Pi 5 **RP1 southbridge**, disabling all USB ports and I2C controllers. Additionally, on Ubuntu Pi 5, `dtparam=i2c_arm=on` must be placed under `[all]` in `/boot/firmware/config.txt` (not `[pi4]`) with `i2c-dev` in `/etc/modules`.
* **Fix:** Restore Ubuntu's kernel-matched `/usr/lib/firmware/6.8.0-1068-raspi/device-tree/broadcom/bcm2712-rpi-5-b.dtb` to `/boot/firmware/bcm2712-rpi-5-b.dtb`, enable I2C under `[all]`, and reboot.

### 9. Headless Pi `rviz2` Crash (`qt.qpa.xcb`)
* **Symptom:** Launching `rviz2` over SSH fails with `Could not load the Qt platform plugin "xcb"`.
* **Fix:** Run `rviz2` remotely on the Ubuntu workstation/laptop with `ros-jazzy-nav2-rviz-plugins` installed.

---

## Phase 4: Nav2 Jazzy Bringup & Autonomous Navigation

### 10. `Frame [map] does not exist` & AMCL Transform Wait
* **Symptom:** RViz shows `Frame [map] does not exist`, and AMCL logs `AMCL cannot publish a pose or update the transform. Please set the initial pose...`.
* **Cause:** AMCL withholds `map -> odom` until an initial pose is published on `/initialpose`, and plain `rviz2` has no `Map` display configured.
* **Fix:** Launch RViz2 with `nav2_default_view.rviz` (or add `Map` `/map`, `LaserScan` `/scan`, `TF`, `Path` `/plan`) and set Riggu's starting pose with **`2D Pose Estimate`**.

### 11. Nav2 Lifecycle Aborts on `collision_monitor` and `docking_server`
* **Symptoms:** `parameter 'observation_sources' is not initialized` (`collision_monitor`) and `Charging dock plugins not given!` (`opennav_docking`).
* **Cause:** `nav2_bringup` in Jazzy brings up both servers by default and aborts if their parameters are missing from `nav2_params.yaml`.
* **Fix:** Add explicit `collision_monitor` and minimal `docking_server` blocks to `nav2_params.yaml`.

### 12. 4000 ms Bond Timeout & `Magick: abort due to signal 11 (SIGSEGV)`
* **Symptom:** `CRITICAL FAILURE: SERVER amcl IS DOWN after not receiving a heartbeat for 4000 ms` followed by `Magick: abort due to signal 11 (SIGSEGV)` and `[component_container_isolated-2]: process has died [exit code -6]`.
* **Cause:** Default `use_composition: True` runs all Nav2 nodes inside one shared container (`component_container_isolated`). Pi 5 startup CPU spikes exceed the 4.0 s bond timeout, triggering a teardown during which `map_server`'s GraphicsMagick unload segfaults and kills the whole stack.
* **Fix:** Set `'use_composition': 'False'` in `navigation.launch.py` and `bond_timeout: 0.0` in `lifecycle_manager_localization` and `lifecycle_manager_navigation`.

### 13. Path Planned, But Robot Does Not Move (`Velocity command timed out. Braking.` + `Failed to make progress`)
* **Symptom:** `controller_server` logs `Passing new path to controller`, then aborts (`Failed to make progress`, `spin failed`, `backup failed`), while `velocity_controller` repeats `Velocity command timed out. Braking.`.
* **Root Causes (diagnosed with `ros2 topic info -v`):**
  1. **Message type mismatch:** `diff_drive_controller` in Jazzy expects **`TwistStamped`**, whereas Nav2 defaults to unstamped **`Twist`** (`['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped']`).
  2. **Topic name mismatch:** In Jazzy `nav2_bringup`, `velocity_smoother` publishes to **`/cmd_vel_smoothed`** (not `/cmd_vel` or `/cmd_vel_raw`), so `collision_monitor` received no messages.
* **Fix:**
  1. Set `enable_stamped_cmd_vel: true` across `bt_navigator`, `controller_server`, `behavior_server`, `velocity_smoother`, `collision_monitor`, and `docking_server`.
  2. Set `cmd_vel_in_topic: "cmd_vel_smoothed"` and `cmd_vel_out_topic: "/velocity_controller/cmd_vel"` under `collision_monitor`.

### 14. Planner Action Timeout Under Pi Load & Obstacle Grazing
* **Symptoms:** `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`, `Planner loop missed its desired rate of 5.0000 Hz (1.54 Hz actual)`, and `Collision Ahead - Exiting DriveOnHeading` / path cutting too close to walls.
* **Fix:** Adjust costmap inflation (`robot_radius: 0.26–0.28`, `inflation_radius: 0.34–0.50`, `cost_scaling_factor: 2.0–4.0`) and `BaseObstacle.scale` to bias paths toward open hallway centers while keeping `10 Hz` control loops within Pi 5 CPU capacity.
