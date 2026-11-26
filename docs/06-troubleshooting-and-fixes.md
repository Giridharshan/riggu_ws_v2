# 6. Errors Encountered & How They Were Solved

A complete reference of the hardware, ROS 2 Jazzy, and Nav2 issues solved while building Riggu.

---

## Nav2 & Autonomous Driving

### 1. Path plans in RViz, but bot doesn't move (`Twist` vs `TwistStamped`)
* **Symptom:** Terminal 1 repeats `[velocity_controller]: Velocity command timed out. Braking.` and Terminal 2 aborts with `Failed to make progress`. Running `ros2 topic info /velocity_controller/cmd_vel` reports `Type: ['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped']`.
* **Cause:** In ROS 2 Jazzy, `diff_drive_controller` (`velocity_controller`) subscribes to `TwistStamped`, while Nav2 defaults to unstamped `Twist` (`enable_stamped_cmd_vel: false`). ROS 2 drops mismatched types silently.
* **Fix:** Add `enable_stamped_cmd_vel: true` to **all** velocity-handling Nav2 nodes in `nav2_params.yaml`: `bt_navigator`, `controller_server`, `behavior_server`, `velocity_smoother`, `collision_monitor`, and `docking_server`.

### 2. Nav2 commands never reach `/velocity_controller/cmd_vel` (`cmd_vel_smoothed`)
* **Symptom:** `TwistStamped` types match, yet `timeout 3 ros2 topic echo /cmd_vel` prints nothing during navigation (`ros2 topic info -v /cmd_vel` only shows `docking_server` publishing).
* **Cause:** In ROS 2 Jazzy `nav2_bringup`, `controller_server` publishes to `/cmd_vel_nav` and `velocity_smoother` outputs to **`/cmd_vel_smoothed`** (not `/cmd_vel`).
* **Fix:** Set `cmd_vel_in_topic: "cmd_vel_smoothed"` and `cmd_vel_out_topic: "/velocity_controller/cmd_vel"` in `collision_monitor` inside `nav2_params.yaml`.

### 3. Nav2 startup aborts on `collision_monitor` or `docking_server`
* **Symptom:**
  * `[collision_monitor]: Error while getting parameters: parameter 'observation_sources' is not initialized`
  * `[docking_server]: Charging dock plugins not given!`
* **Cause:** `nav2_bringup` in Jazzy enables both `collision_monitor` and `docking_server` in its default lifecycle manager and aborts bringup if their parameter sections are missing from `nav2_params.yaml`.
* **Fix:** Include full `collision_monitor` (pointing `/scan` and `PolygonStop`) and minimal `docking_server` (`dock_plugins: ["simple_charging_dock"]`) blocks in `nav2_params.yaml`.

### 4. Nav2 crashes on Pi 5 with 4000 ms heartbeat error & `Magick` SIGSEGV
* **Symptom:** `[lifecycle_manager_localization]: CRITICAL FAILURE: SERVER amcl IS DOWN after not receiving a heartbeat for 4000 ms.` followed by `Magick: abort due to signal 11 (SIGSEGV)` and `[component_container_isolated-2]: process has died [exit code -6]`.
* **Cause:** Default `use_composition: True` loads all Nav2 nodes into a single `component_container_isolated` process, and startup CPU spikes on the Pi 5 exceed the 4.0 s bond heartbeat timeout, triggering a teardown crash in `map_server`.
* **Fix:**
  1. Pass `'use_composition': 'False'` in `navigation.launch.py`.
  2. Set `bond_timeout: 0.0` under `lifecycle_manager_localization` and `lifecycle_manager_navigation` in `nav2_params.yaml`.

### 5. `Frame [map] does not exist` in RViz2
* **Symptom:** RViz shows red `Global Status: Error -> Frame [map] does not exist`, and Terminal 2 logs `AMCL cannot publish a pose or update the transform. Please set the initial pose...`.
* **Cause:** AMCL does not publish the `map -> odom` transform until it receives an initial pose on `/initialpose` (and plain `rviz2` without a config has no `Map` display added).
* **Fix:** Launch RViz2 with `nav2_default_view.rviz` (or add **Map** `/map`, **LaserScan** `/scan`, **TF**, **Path** `/plan`) and use **`2D Pose Estimate`** to place the robot on the map.

---

## SLAM, EKF & Visualization

### 6. `slam_toolbox: Failed to compute odom pose` (`base_footprint` fallback)
* **Symptom:** `slam_toolbox` prints `Failed to compute odom pose` and looks for `base_footprint` instead of `base_link`.
* **Cause:** `slam_toolbox`'s `online_async_launch.py` reads the argument **`slam_params_file`**, not `params_file`. Passing `params_file:=...` causes it to silently fall back to factory defaults (`base_footprint`).
* **Fix:** Use `'slam_params_file': ...` in `slam.launch.py`.

### 7. EKF rate errors & RViz TF extrapolation (`Lookup would require extrapolation into the future`)
* **Symptom:** `[ekf_node]: Failed to meet update rate! Took 0.0611 seconds` and RViz / `slam_toolbox` drops `/scan` messages with timestamp extrapolation errors.
* **Cause:** Running EKF at 30 Hz overloaded the Pi CPU (~61 ms per cycle), and Laptop vs Pi system clocks drifted apart.
* **Fix:**
  1. In `config/ekf.yaml`, reduce `frequency: 15.0` and enable `predict_to_current_time: true` + `transform_time_offset: 0.05`.
  2. Keep Laptop and Raspberry Pi clocks synchronized via NTP or `sudo date -s`.

### 8. Running `rviz2` directly on Raspberry Pi crashes (`qt.qpa.xcb: could not connect to display`)
* **Symptom:** `Could not load the Qt platform plugin "xcb"` / `Aborted (core dumped)`.
* **Cause:** The Pi runs headless without an X11/Wayland desktop session.
* **Fix:** Always run `rviz2` on the Laptop over the same Wi-Fi network / `ROS_DOMAIN_ID`.

---

## `ros2_control`, ESP32-S3 & Raspberry Pi 5 Hardware

### 9. `ros2_control_node` crashes at startup (`SIGABRT exit code -6`)
* **Symptom:** Process dies in `ControllerManager::set_initial_hardware_components_state()`.
* **Cause:** `<ros2_control name="DriveLinkImuSensor">` in `robot.urdf.xacro` tried to configure the ESP32 IMU over `/dev/ttyACM0` after BNO055 was moved to Raspberry Pi I2C (`/dev/i2c-1`).
* **Fix:** Comment out the `DriveLinkImuSensor` block in `robot.urdf.xacro` so only `DriveLinkSystem` uses `/dev/ttyACM0`, and publish IMU data from `bno055_pi.py`.

### 10. `Firmware NACK for SetMode on right_wheel_joint (result=0x02)`
* **Symptom:** `ros2_control` fails to activate motor joints with firmware NACK `0x02`.
* **Cause:** ESP32 watchdog/safety lockout after a Pi reboot/serial interruption or motor bus power dip.
* **Fix:** Verify motor battery/power switch is ON, press **`EN` / `RST`** on the ESP32-S3, confirm `/dev/ttyACM0` permissions, and relaunch `drivelink.launch.py`.

### 11. Wheels spin uncontrollably or `/joint_states` stays `[0.0, 0.0]`
* **Symptom:** `/joint_states` position remains `[0.0, 0.0]` when wheels turn; PID saturates on joystick input or only one wheel spins.
* **Cause:** Open/loose quadrature encoder wires (`GPIO 4/5` for Left, `GPIO 7/6` for Right, or encoder `VCC`/`GND`), or swapped PWM/DIR/encoder pairings.
* **Fix:** Inspect physical encoder wiring and verify positive count changes via `ros2 topic echo /joint_states --field position` when rolling each wheel forward manually.

### 12. Pi 5 Ubuntu 24.04 loses `/dev/i2c-1` and all USB ports (`lsusb` blank)
* **Symptom:** `FileNotFoundError: '/dev/i2c-1'`, missing `/dev/ttyACM0` and `/dev/ttyUSB0`, kernel log `brcm-pcie: Unable to find MSI PCI address`.
* **Cause:** Overwriting `/boot/firmware/bcm2712-rpi-5-b.dtb` with a Raspberry Pi OS DTB breaks the PCIe link to the Pi 5 RP1 southbridge (which owns USB and I2C) on Ubuntu 24.04 (`6.8.0-raspi`).
* **Fix:** Copy Ubuntu's matching DTB back to `/boot/firmware/bcm2712-rpi-5-b.dtb` from `/usr/lib/firmware/<kernel>-raspi/device-tree/broadcom/bcm2712-rpi-5-b.dtb` (or reinstall `linux-image-raspi` / `linux-firmware-raspi`) and reboot. If SD card corruption caused read-only I/O errors, repair FAT32/ext4 on a PC with `fsck.vfat` / `fsck.ext4`.
