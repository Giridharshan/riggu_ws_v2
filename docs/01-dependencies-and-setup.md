# 1. Dependencies & Setup

This document lists all software, ROS 2 packages, system configuration, and build steps needed for **Riggu** on both the **Raspberry Pi 5** and the **Workstation (Laptop)**.

---

## 1. Platform & Environment

| Machine | OS | ROS 2 Distro | Role |
| :--- | :--- | :--- | :--- |
| **Raspberry Pi 5** | Ubuntu 24.04 LTS (`aarch64`, `6.8.0-raspi`) | ROS 2 **Jazzy Jalisco** | Runs `ros2_control`, BNO055 I2C IMU, EKF, LiDAR, SLAM, Nav2 |
| **Workstation / Laptop** | Ubuntu 24.04 LTS | ROS 2 **Jazzy Jalisco** | Runs `rviz2`, teleop, remote SSH monitoring |
| **Motor Controller** | ESP-IDF `5.5.2` (FreeRTOS 1000 Hz) | — | Runs `firmware/drivelink_v2` on `esp32-s3-devkitc-1` |

---

## 2. Raspberry Pi 5 — ROS 2 & System Dependencies

### ROS 2 Packages (`apt`)
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-imu-sensor-broadcaster \
  ros-jazzy-forward-command-controller \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-ros2controlcli \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-tf2-tools
```

### System & Hardware Utilities
* **Testing / Virtual Serial Loopback (for `drivelink_hardware` mock tests):**
  ```bash
  sudo apt install -y socat
  ```
* **I2C & Serial Access (for BNO055 on `/dev/i2c-1`, ESP32 on `/dev/ttyACM0`, LiDAR on `/dev/ttyUSB0`):**
  ```bash
  sudo apt install -y i2c-tools python3-smbus
  sudo usermod -aG dialout,i2c $USER
  ```
  Ensure `/boot/firmware/config.txt` enables ARM I2C (`dtparam=i2c_arm=on`) and `i2c-dev` is loaded (`sudo modprobe i2c-dev`).

---

## 3. Workstation (Laptop) — Visualization Dependencies

Because the Raspberry Pi runs headless without a display server, **RViz2** runs on your workstation over the same Wi-Fi network:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rviz2 \
  ros-jazzy-nav2-rviz-plugins \
  ros-jazzy-nav2-bringup
```

---

## 4. ESP32-S3 Firmware Dependencies (`firmware/drivelink_v2`)

Choose **one** of the following toolchains on the machine you flash from:
* **ESP-IDF `5.5.2`** (target `esp32s3`) — see the [Espressif ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/get-started/).
* **PlatformIO** (VS Code extension or `pio` CLI) using `espressif32` platform with `framework = espidf` (configured in `firmware/drivelink_v2/platformio.ini`).

---

## 5. Building the ROS 2 Workspace (`riggu_ws_v2`)

Make sure `firmware/` contains an empty `COLCON_IGNORE` file (`touch firmware/COLCON_IGNORE`) so `colcon` ignores ESP-IDF's `CMakeLists.txt`:

```bash
cd ~/riggu_ws_v2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To rebuild a single package quickly after config/URDF edits:
```bash
cd ~/riggu_ws_v2
colcon build --symlink-install --packages-select drivelink_bringup
source install/setup.bash
```

To run `drivelink_hardware` unit/integration tests:
```bash
colcon test --packages-select drivelink_hardware
colcon test-result --verbose
```

---

## 6. Network & Clock Synchronization (Pi <-> Laptop)

1. Both machines must be on the **same Wi-Fi subnet** and share the same `ROS_DOMAIN_ID` (default `0`).
2. **Synchronize clocks** (vital so TF2/RViz2/Nav2 do not drop laser scans due to timestamp skew):
   - Connect both machines to the internet so NTP syncs automatically, **or**
   - Sync the Pi's clock directly from your laptop over SSH:
     ```bash
     ssh riggu@RIGGU "sudo date -s '$(date -u +"%Y-%m-%d %H:%M:%S")'"
     ```
