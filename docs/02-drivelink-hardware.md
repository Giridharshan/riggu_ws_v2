# 2. `drivelink_hardware` — `ros2_control` Hardware Interface

`src/drivelink_hardware` is the C++ host-side **`ros2_control` plugin package** that bridges ROS 2 controllers (`controller_manager`) with the ESP32-S3 `drivelink_v2` firmware over a framed binary UART protocol.

---

## 1. Exported Plugins (`drivelink_hardware.xml`)

| Plugin Name | Base Class | Purpose | Status on Riggu |
| :--- | :--- | :--- | :--- |
| `drivelink_hardware/DriveLinkSystem` | `hardware_interface::SystemInterface` | Configures wheel motors on the ESP32, switches control modes, streams setpoints, and reads wheel encoder position/velocity telemetry. | **Active** |
| `drivelink_hardware/DriveLinkImuSensor` | `hardware_interface::SensorInterface` | Configures BNO055 IMU streaming over ESP32 UART and exports 10 semantic IMU state interfaces (`orientation.*`, `angular_velocity.*`, `linear_acceleration.*`). | **Commented out** in Riggu's URDF because the BNO055 runs on Pi I2C (`/dev/i2c-1`). |

---

## 2. Internal Architecture

### Shared Serial Ownership (`DriveLinkCommRegistry`)
Because `DriveLinkSystem` and `DriveLinkImuSensor` are separate `ros2_control` plugins targeting the same physical USB-UART port (`/dev/ttyACM0`), `DriveLinkCommRegistry` ensures **one shared `DriveLinkComm` instance** opens the serial port once and keeps it open until the last plugin detaches.

### Real-Time Safe Transport (`DriveLinkComm`)
Source files: `src/drivelink_comm_io.cpp`, `src/drivelink_comm_rx.cpp`
* Runs a dedicated `poll()`-based reader thread (50 ms timeout) so serial reads and CRC parsing **never block** `ros2_control`'s real-time `read()` / `write()` loop.
* `read()` copies the latest thread-safe telemetry snapshot using `try_lock`.
* `write()` detects changed command values (within per-mode epsilon) and transmits packed binary setpoint frames.
* Supports `inject_test_fd(int)` for loopback testing without physical hardware.

### Lifecycle & Mode Switching (`drivelink_system_lifecycle.cpp`, `drivelink_system_runtime.cpp`)
* `on_init()`: Validates joint names, URDF hardware params (`uart_port`, `uart_baud`, `telemetry_period_ms`), GPIO assignments, PID gains, and exported state/command interfaces.
* `on_configure()`: Opens `/dev/ttyACM0`, registers telemetry/response/time-sync callbacks, sends `CREATE_MOTOR` (`0x21`) and telemetry configuration commands to the ESP32, and replies to firmware time-sync requests (`0x20` -> `0x60`).
* `on_activate()` / `on_deactivate()`: Sends `SET_MODE` to activate motors (waiting for `ACK`) and safely stops motors + resets command values to `NaN` on deactivation or error.
* `prepare_command_mode_switch()` / `perform_command_mode_switch()`: Validates and switches per-joint between `velocity`, `position`, and `effort` modes.

---

## 3. Binary Wire Protocol Summary

Full specification is archived in [`archive/ros2_control_plan/01_firmware_api_reference.md`](../archive/ros2_control_plan/01_firmware_api_reference.md).

### Frame Format
```text
[ 0xAA ] [ 0xAA ] [ 0xAA ] [ MSG_TYPE ] [ LEN ] [ PAYLOAD (0..128 bytes) ] [ CRC_LO ] [ CRC_HI ]
```
* **CRC-16/CCITT** (`crc16.hpp`): Polynomial `0x1021`, `init = 0xFFFF`, computed over `[MSG_TYPE, LEN, PAYLOAD...]`, transmitted little-endian.

### Active Message IDs
| ID | Direction | Purpose |
| :--- | :--- | :--- |
| `0x20` / `0x60` | ESP -> Host / Host -> ESP | Time sync request / response |
| `0x21` / `0x61` | Host -> ESP / ESP -> Host | Motor manager configure command (`CREATE_MOTOR`, `SET_MODE`, etc.) / ACK-NACK response |
| `0x80` | Host -> ESP | Motor setpoint stream (latest-wins) |
| `0x81` | ESP -> Host | Motor position/velocity telemetry stream |
| `0x22` / `0x42` / `0x82` / `0xC2` | Bidirectional | IMU configure, response, telemetry stream, async event |

---

## 4. Built-in Tests & Mock Simulator

`CMakeLists.txt` builds four standalone test binaries under `test/`:
* `test_crc16`: Verifies CRC-16/CCITT reference vectors.
* `test_comm`: `socketpair` loopback test for valid framing, CRC mismatch rejection, and setpoint parsing.
* `test_plugin_list`: Verifies `pluginlib` exports `DriveLinkSystem` and `DriveLinkImuSensor`.
* `test_system_lifecycle`: Verifies clean error handling on invalid serial port paths.

`tools/mock_firmware.py` emulates the ESP32 protocol over a `socat` virtual PTY pair so `drivelink.launch.py` can be tested without real hardware.
