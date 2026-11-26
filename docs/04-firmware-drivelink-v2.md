# 4. `firmware/drivelink_v2` — ESP32-S3 ESP-IDF Firmware

`firmware/drivelink_v2` is the C++ embedded motor-control and telemetry firmware for **ESP32-S3** (`esp32-s3-devkitc-1`, 16 MB QIO flash, 240 MHz CPU, 1000 Hz FreeRTOS tick), built on **ESP-IDF 5.5.2** (and also buildable with **PlatformIO** via `platformio.ini`).

> An empty `firmware/COLCON_IGNORE` file sits above `drivelink_v2` so ROS 2 `colcon build` ignores this ESP-IDF project.

---

## 1. Startup Flow (`main/main.cpp`)

`app_main()` initializes four subsystems in sequence and checks `isValid()` on each:
1. **`uart_comm::UartComm`** — Starts UART0 RX/TX tasks and pattern-detection framing (`3x 0xAA`).
2. **`motor_manager::MotorManager`** — Registers command/setpoint handlers (`max_motors = 4`, internal 10 ms / 100 Hz control & telemetry update task, `use_mock_feedback = false`).
3. **`time_sync::TimeSync`** — Synchronizes ESP32 timestamps with the Raspberry Pi (`sync_interval_ms = 5000`, `response_timeout_ms = 250`, RTT compensation enabled).
4. **`imu_bmo_055::ImuBno055Manager`** — Optional ESP-side BNO055 manager (`default_operation_mode = 0x0C`).

---

## 2. Firmware Components (`firmware/drivelink_v2/components/`)

| Component | Purpose |
| :--- | :--- |
| **`uart_comm`** | Binary UART transport with `0xAA 0xAA 0xAA` sync pattern, CRC-16/CCITT verification, reliable FIFO command queue (`0x01..0x3F`, `0x40..0x7F`, `0xC0..0xFE`), and latest-wins stream channels (`0x80..0xBF`). |
| **`motor_hal`** | Low-level DC motor PWM + DIR driver (`esp_motor::Motor`) backed by pooled ESP32-S3 MCPWM timers (`25 kHz` PWM frequency, `1 MHz` timer resolution). |
| **`rotary_encoder_hal`** | Quadrature encoder decoding using ESP32-S3 hardware PCNT units (`1000 ns` glitch filter, `±32767` overflow accumulation, 50 ms default rate window). |
| **`pid`** | Hardware-independent discrete PID + feedforward controller (`10 ms` default / `100 Hz`, output limits `-1.0..1.0`). |
| **`motor_controller`** | Combines `motor_hal`, `rotary_encoder_hal`, and `pid` for each motor instance (`open_loop`, `velocity`, or `position` mode). |
| **`motor_manager`** | Handles dynamic `CREATE_MOTOR`, `DELETE_MOTOR`, `CONFIGURE_MOTOR`, `SET_MODE`, `SET_PID_PARAMS`, `SET_SETPOINT`, `STOP_MOTOR`, `RESET_MOTOR`, telemetry masks, and stream packets (`0x21`/`0x61`/`0x80`/`0x81`). |
| **`imu_bmo_055`** | Bosch BNO055 driver & telemetry streamer (`0x22`/`0x42`/`0x82`/`0xC2`; supports mock fallback if sensor is absent on ESP I2C). |
| **`time_sync`** | Periodic clock-sync request/response protocol (`0x20`/`0x60`) aligning firmware `timestamp_us` with host Unix time. |

---

## 3. Hardware Defaults (`sdkconfig.defaults` & `sdkconfig.esp32-s3-devkitc-1`)

* **Console Logs:** Routed to `USB_SERIAL_JTAG` (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, UART console disabled).
* **DriveLink UART Link (`uart_comm`):** `UART0` on **TX `GPIO 43`**, **RX `GPIO 44`**, `115200` baud, max payload `128` bytes.
* **Motor & Encoder Pins:** Passed dynamically at runtime from `robot.urdf.xacro` via `CREATE_MOTOR` (`Left: PWM 11, DIR 12, ENC 4/5`; `Right: PWM 15, DIR 16, ENC 7/6`). Default firmware `counts_per_rev` is `1440` (overridden by URDF `counts_per_rev`).
* **ESP-Side I2C IMU Defaults (unused on Riggu):** I2C Port `0`, **SDA `GPIO 8`**, **SCL `GPIO 9`**, `100 kHz`, address `0x28`, mode `0x0C`. *(Riggu connects the BNO055 to Raspberry Pi `/dev/i2c-1` instead.)*

---

## 4. Building & Flashing

> Always stop `drivelink.launch.py` on the Pi before flashing so `/dev/ttyACM0` is free.

### Using ESP-IDF CLI
```bash
source ~/esp/esp-idf/export.sh
cd ~/riggu_ws_v2/firmware/drivelink_v2
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

### Using PlatformIO (VS Code or CLI)
```bash
cd ~/riggu_ws_v2/firmware/drivelink_v2
pio run -t upload --upload-port /dev/ttyACM0
pio device monitor -b 115200
```
