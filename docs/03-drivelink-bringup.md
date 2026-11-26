# 3. `drivelink_bringup` — Robot Config, Sensor Fusion, SLAM & Nav2

`src/drivelink_bringup` holds everything required to describe Riggu physically, bring up its sensors and motors, build 2D maps, and navigate autonomously.

---

## 1. Robot Description & Wiring (`urdf/robot.urdf.xacro`)

### Coordinate Frame Tree (TF2)
```text
map ──(slam_toolbox / amcl)──▶ odom ──(ekf_node)──▶ base_link ──(robot_state_publisher)──▶ laser
```
* `map -> odom`: Published by `slam_toolbox` during mapping or `amcl` during navigation (after setting an initial pose). Corrects wheel odometry drift against the static map.
* `odom -> base_link`: Published by `ekf_node` (`robot_localization`) from fused wheel encoders and BNO055 IMU yaw/angular velocity.
* `base_link -> laser`: Static transform defined by `lidar_mount_joint` in `robot.urdf.xacro` (`xyz="-0.01 0.0 0.152" rpy="0 0 0"`, relocated from the caster end at `+0.26`).

### ESP32-S3 Motor & Encoder GPIO Parameters
`DriveLinkSystem` reads these pin parameters from `robot.urdf.xacro` and sends them to the ESP32-S3 at startup:

| Joint / Motor | `motor_id` | `pwm_gpio` | `dir_gpio` | `enc_a` | `enc_b` |
| :--- | :---: | :---: | :---: | :---: | :---: |
| `left_wheel_joint` | `1` | `11` | `12` | `4` | `5` |
| `right_wheel_joint` | `2` | `15` | `16` | `7` | `6` |

> **Why `DriveLinkImuSensor` is commented out in `robot.urdf.xacro`:**  
> Riggu's BNO055 IMU was moved off the ESP32 onto the Raspberry Pi's native I2C bus (`/dev/i2c-1`). Commenting out `<ros2_control name="DriveLinkImuSensor">` prevents dual-plugin serial contention on `/dev/ttyACM0` (`ros2_control_node` `exit code -6`).

---

## 2. Base Bringup & Sensors (`launch/drivelink.launch.py`)

Starts Riggu's core hardware stack:
1. **`robot_state_publisher`**: Publishes the URDF and static TF transforms (`base_link -> laser`, wheels).
2. **`ros2_control_node` + `config/controllers.yaml`** (`100 Hz` update rate):
   * `joint_state_broadcaster`: Publishes wheel encoder angles/velocities to `/joint_states`.
   * `velocity_controller` (`diff_drive_controller/DiffDriveController`): Converts `geometry_msgs/msg/TwistStamped` commands on `/velocity_controller/cmd_vel` into left/right wheel angular velocity setpoints and computes differential wheel odometry. *(In ROS 2 Jazzy, `diff_drive_controller` expects `TwistStamped` by default.)*
3. **`bno055_pi.py`**: Reads the Bosch BNO055 directly from Raspberry Pi I2C (`/dev/i2c-1`, Pi `GPIO 2` SDA / `GPIO 3` SCL) in NDOF mode (`0x0C`) and publishes `/bno055/imu` at **50.0 Hz**.
4. **`sensor_validator.py`**: Runtime sensor validation & protection monitor.
5. **`ekf_node` (`config/ekf.yaml`)**: Fuses wheel odometry with `/bno055/imu` and publishes `/odometry/filtered` + `odom -> base_link` TF.
   * Tuned specifically for Raspberry Pi 5 CPU budget: `frequency: 15.0`, `predict_to_current_time: true`, `transform_time_offset: 0.05`.

---

## 3. 2D Mapping (`launch/slam.launch.py` & `config/slam_toolbox.yaml`)

Launches:
1. **Slamtec SLLidar C1 driver** (`sllidar_ros2/launch/sllidar_c1_launch.py`) on `/dev/ttyUSB0`, publishing 2D scans to `/scan` at 10 Hz (`5 kHz` sample rate).
2. **`slam_toolbox`** (`online_async_launch.py`), explicitly passing `slam_params_file:=.../config/slam_toolbox.yaml` (`odom_frame: odom`, `map_frame: map`, `base_frame: base_link`, `resolution: 0.05`, `max_laser_range: 12.0`).

---

## 4. Autonomous Navigation (`launch/navigation.launch.py` & `config/nav2_params.yaml`)

Launches the SLLidar C1 driver alongside `nav2_bringup/launch/bringup_launch.py` with:
* `use_sim_time: false`, `autostart: true`, **`use_composition: False`** (runs each Nav2 node as a separate OS process instead of a single `component_container_isolated` container).

### Key Configured Nav2 Modules (`config/nav2_params.yaml`)
* **`amcl`**: Particle filter localization (`DifferentialMotionModel`, `500–2000` particles, `/scan`, `base_link`, `odom`, `map`). Publishes `map -> odom` once given an initial pose.
* **`global_costmap` & `local_costmap`**: Static layer (`/map`), live LiDAR `ObstacleLayer` (`/scan`), and `InflationLayer`.
* **`planner_server`**: `nav2_navfn_planner::NavfnPlanner` (A* global path finder).
* **`controller_server`**: `dwb_core::DWBLocalPlanner` (`10 Hz`, progress checker `0.2 m / 10 s`, goal tolerance `0.15 m / 0.2 rad`).
* **`velocity_smoother`**: Smooths raw `cmd_vel_nav` into **`/cmd_vel_smoothed`**.
* **`collision_monitor`**: Subscribes to **`cmd_vel_smoothed`**, checks `/scan` against `PolygonStop` (`[[0.35, 0.25], [0.35, -0.25], [-0.35, -0.25], [-0.35, 0.25]]`), and outputs stamped velocities to **`/velocity_controller/cmd_vel`**.
* **`docking_server`**: Configured with `opennav_docking::SimpleChargingDock` so Nav2 Jazzy's lifecycle manager completes startup cleanly (even when no physical dock is used).
* **`lifecycle_manager_localization` & `lifecycle_manager_navigation`**: Both set `bond_timeout: 0.0` to prevent spurious 4000 ms startup timeouts on the Raspberry Pi 5.
* **`enable_stamped_cmd_vel: true`**: Enabled across `bt_navigator`, `controller_server`, `behavior_server`, `velocity_smoother`, `collision_monitor`, and `docking_server` so Nav2 outputs `TwistStamped` matching `velocity_controller`.
