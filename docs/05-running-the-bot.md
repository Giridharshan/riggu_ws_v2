# 5. Commands to Run Riggu

All robot nodes run on the **Raspberry Pi 5** (`riggu@RIGGU`), while **RViz2** runs on your **Workstation / Laptop**.

---

## Step 0: Pre-Flight Hardware Check (Pi)

Verify ESP32 (`/dev/ttyACM0`), SLLidar (`/dev/ttyUSB0`), and BNO055 (`/dev/i2c-1`) are visible:
```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/i2c*
```
If serial permissions fail, run:
```bash
sudo chmod 666 /dev/ttyACM* /dev/ttyUSB*
```

---

## Step 1: Start Base Hardware & Odometry (Pi — Terminal 1)

Always keep this running in **Terminal 1** for both SLAM and Navigation:
```bash
cd ~/riggu_ws_v2
source install/setup.bash
ros2 launch drivelink_bringup drivelink.launch.py
```
Wait until you see:
* `bno055_pi_node`: `BNO055 confirmed in NDOF mode (0x0C)` and publishing `/bno055/imu` at `50.0 Hz`.
* `joint_state_broadcaster` and `velocity_controller` activated.

---

## Step 2A: SLAM Mapping Mode (Pi — Terminal 2)

To create a new map of a room:
```bash
cd ~/riggu_ws_v2
source install/setup.bash
ros2 launch drivelink_bringup slam.launch.py
```
Drive Riggu slowly around the room using your teleop / joystick while watching the map build in RViz2.

### Save the Map (Pi — Terminal 3)
Once the map looks complete (keep `slam.launch.py` running while saving):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/riggu_ws_v2/maps/my_new_map
```
This creates `my_new_map.yaml` and `my_new_map.pgm`.

---

## Step 2B: Autonomous Navigation Mode (Pi — Terminal 2)

*(Stop `slam.launch.py` first so `slam_toolbox` and `sllidar` aren't launched twice.)*

Launch Nav2 using your saved map:
```bash
cd ~/riggu_ws_v2
source install/setup.bash
ros2 launch drivelink_bringup navigation.launch.py map:=$HOME/riggu_ws_v2/maps/my_new_map.yaml
```
AMCL will print `AMCL cannot publish a pose or update the transform. Please set the initial pose...` until you set the pose in RViz2.

---

## Step 3: Visualize & Command in RViz2 (Laptop — Terminal 3)

Make sure Laptop and Pi clocks are synced, then run on your **laptop**:
```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Localize & Navigate:
1. **Set Starting Position:** Click **`2D Pose Estimate`** in the top toolbar $\rightarrow$ click on the map where Riggu is physically sitting $\rightarrow$ drag the green arrow in the direction Riggu is facing.
   * AMCL warnings stop, the `map -> odom` transform becomes active, and live `/scan` red dots snap onto the map walls.
2. **Send Autonomous Goal:** Click **`2D Goal Pose`** $\rightarrow$ click any open floor space $\rightarrow$ drag the arrow to the desired final orientation.

*(If opening plain `rviz2` without the Nav2 layout, manually click **Add** for **Map** (`/map`, Durability: `Transient Local`), **LaserScan** (`/scan`), **TF**, and **Path** (`/plan`).)*

---

## Useful Test & Diagnostic Commands

### Drive Straight Test (No Nav2 Needed)
Drives forward at `0.15 m/s` for 3 seconds (~`0.45 m`), then auto-brakes when `timeout 3` stops publishing:
```bash
timeout 3 ros2 topic pub -r 10 /velocity_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.15}, angular: {z: 0.0}}}"
```

### Nav2 `DriveOnHeading` Test (0.5 m Forward)
```bash
ros2 action send_goal /drive_on_heading nav2_msgs/action/DriveOnHeading "{target: {x: 0.5, y: 0.0, z: 0.0}, speed: 0.15, time_allowance: {sec: 10, nanosec: 0}}"
```

### Live Health Checks
```bash
# Check wheel encoders (values must change when wheels spin)
ros2 topic echo /joint_states --field position

# Check fused EKF odometry position
ros2 topic echo /odometry/filtered --field pose.pose.position

# Check TF chain (map -> odom -> base_link -> laser)
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link

# Verify TwistStamped matching & QoS on velocity topics
ros2 topic info -v /cmd_vel_nav
ros2 topic info -v /cmd_vel_smoothed
ros2 topic info -v /velocity_controller/cmd_vel

# Inspect recent launch logs
grep -i "error\|warn" ~/.ros/log/latest/launch.log
```
