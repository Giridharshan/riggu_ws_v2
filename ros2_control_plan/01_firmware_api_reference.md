# DriveLink Firmware API Reference (For Host ros2_control Integration)

This document is the firmware-side protocol ground truth for host implementation. All field layouts and message IDs are derived from source and active `sdkconfig` values.

## 1. Transport Framing (uart_comm)

### 1.1 Frame format (all traffic classes)

`uart_comm` frame layout:

| Byte Offset | Field | Type | Size | Notes |
|---:|---|---|---:|---|
| 0 | START0 | `uint8_t` | 1 | Constant `0xAA` |
| 1 | START1 | `uint8_t` | 1 | Constant `0xAA` |
| 2 | START2 | `uint8_t` | 1 | Constant `0xAA` |
| 3 | MSG_TYPE | `uint8_t` | 1 | Command/response/stream/event class |
| 4 | LEN | `uint8_t` | 1 | Payload length in bytes |
| 5..(4+LEN) | PAYLOAD | `uint8_t[]` | LEN | 0..`CONFIG_UART_COMM_MAX_PAYLOAD` |
| 5+LEN | CRC_LO | `uint8_t` | 1 | CRC16 low byte |
| 6+LEN | CRC_HI | `uint8_t` | 1 | CRC16 high byte |

Sources:
- constants/start/header/crc lengths: `components/uart_comm/priv_include/frame.hpp:12`, `components/uart_comm/priv_include/frame.hpp:13`, `components/uart_comm/priv_include/frame.hpp:14`, `components/uart_comm/priv_include/frame.hpp:15`
- encoding writes start/header/CRC bytes: `components/uart_comm/src/frame.cpp:18`, `components/uart_comm/src/frame.cpp:23`, `components/uart_comm/src/frame.cpp:24`, `components/uart_comm/src/frame.cpp:33`, `components/uart_comm/src/frame.cpp:34`
- active max payload config: `sdkconfig:2369`

### 1.2 Message class ranges

| Class | Numeric range | Intended direction |
|---|---|---|
| Command | `0x01..0x3F` | Usually host -> ESP (except time_sync request) |
| Response | `0x40..0x7F` | Usually ESP -> host (except time_sync response) |
| Stream channel | `0x80..0xBF` | Bidirectional streaming payloads |
| Event | `0xC0..0xFE` | Async events (currently sparse use) |

Sources: `components/uart_comm/priv_include/frame.hpp:20`, `components/uart_comm/priv_include/frame.hpp:22`, `components/uart_comm/priv_include/frame.hpp:24`, `components/uart_comm/priv_include/frame.hpp:26`

### 1.3 CRC definition

Firmware uses `crc16Ccitt()` over `[MSG_TYPE, LEN, PAYLOAD...]`, then appends little-endian CRC bytes (`CRC_LO`, `CRC_HI`).

- Named variant in source comments: CRC-16/CCITT
- Polynomial in source comments: `0x1021`
- Implementation path: `esp_rom_crc16_be()` with init/final inversion wrapper

Implementation excerpt behavior:
1. `crc = 0xFFFF`
2. `crc = esp_rom_crc16_be(crc, data, len)`
3. return `crc`

Sources:
- API comment: `components/uart_comm/priv_include/frame.hpp:35`
- compute and append: `components/uart_comm/src/frame.cpp:8`, `components/uart_comm/src/frame.cpp:11`, `components/uart_comm/src/frame.cpp:12`, `components/uart_comm/src/frame.cpp:31`, `components/uart_comm/src/frame.cpp:32`

⚠ VERIFY: if you re-implement CRC manually, match firmware behavior with test vectors against `esp_rom_crc16_be` wrapper, not only by variant name.

### 1.4 Parser and error behavior relevant to host

- Host must tolerate dropped frames after CRC mismatch or malformed lengths (`Parser` discards bad frame and continues): `components/uart_comm/src/parser.cpp:112`, `components/uart_comm/src/parser.cpp:129`, `components/uart_comm/src/parser.cpp:130`
- RX frame parse timeout is 100 ms for header/body reads: `components/uart_comm/src/parser.cpp:13`
- `sendCommand()` returns `ESP_ERR_NO_MEM` if command queue full (depth from Kconfig): `components/uart_comm/src/uart_comm.cpp:384`, `components/uart_comm/Kconfig:57`
- Stream channels are latest-wins and can overwrite unsent prior stream payloads: `components/uart_comm/src/uart_comm.cpp:395`

## 2. Active Message IDs (Current Build)

Message IDs are compile-time configurable. Current active values from `sdkconfig`:

| Purpose | Key | Value | Direction |
|---|---|---|---|
| Motor manager configure command | `CONFIG_MOTOR_MANAGER_CONFIGURE_MSG_TYPE` | `0x21` | host -> ESP |
| Motor manager configure response | `CONFIG_MOTOR_MANAGER_CONFIGURE_RESPONSE_MSG_TYPE` | `0x61` | ESP -> host |
| Motor manager setpoint stream | `CONFIG_MOTOR_MANAGER_SETPOINT_STREAM_CHANNEL` | `0x80` | host -> ESP |
| Motor manager telemetry stream | `CONFIG_MOTOR_MANAGER_TELEMETRY_STREAM_CHANNEL` | `0x81` | ESP -> host |
| IMU configure command | `CONFIG_IMU_BMO_055_CONFIGURE_MSG_TYPE` | `0x22` | host -> ESP |
| IMU configure response | `CONFIG_IMU_BMO_055_CONFIGURE_RESPONSE_MSG_TYPE` | `0x42` | ESP -> host |
| IMU event message | `CONFIG_IMU_BMO_055_EVENT_MSG_TYPE` | `0xC2` | ESP -> host |
| IMU telemetry stream | `CONFIG_IMU_BMO_055_STREAM_CHANNEL` | `0x82` | ESP -> host |
| Time sync request | `CONFIG_TIME_SYNC_REQUEST_MSG_TYPE` | `0x20` | ESP -> host |
| Time sync response | `CONFIG_TIME_SYNC_RESPONSE_MSG_TYPE` | `0x60` | host -> ESP |

Sources: `sdkconfig:2327`, `sdkconfig:2328`, `sdkconfig:2329`, `sdkconfig:2330`, `sdkconfig:2288`, `sdkconfig:2289`, `sdkconfig:2290`, `sdkconfig:2291`, `sdkconfig:2358`, `sdkconfig:2359`

## 3. motor_manager Protocol (`0x21`, `0x61`, `0x80`, `0x81`)

### 3.1 Configure command payload (host -> ESP, msg type `0x21`)

Common header:

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 |
| 1 | `subcommand` | `uint8_t` (`ConfigureSubcommand`) | 1 |
| 2 | `motor_id` | `uint8_t` | 1 |

Version is currently `0x01`: `components/motor_manager/include/motor_manager_types.hpp:48`.
Subcommand enum values: `components/motor_manager/include/motor_manager_types.hpp:24`.

#### 3.1.1 Subcommands and bodies

`ConfigureSubcommand` values:
- `0x01` CreateMotor
- `0x02` DeleteMotor
- `0x03` ConfigureMotor
- `0x04` SetMode
- `0x05` SetPidParams
- `0x06` SetSetpoint
- `0x07` StopMotor
- `0x08` ResetMotor
- `0x09` SetTelemetryGlobal
- `0x0A` SetTelemetryMotor

Source: `components/motor_manager/include/motor_manager_types.hpp:24`

Create/Configure body (after byte 2), little-endian for floats/u32 due raw `memcpy` parse:

| Relative Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `pwm_gpio` | `uint8_t` | 1 |
| 1 | `dir_gpio` | `uint8_t` | 1 |
| 2 | `enc_a` | `uint8_t` | 1 |
| 3 | `enc_b` | `uint8_t` | 1 |
| 4 | `counts_per_rev` | `float32` | 4 |
| 8 | `gear_ratio` | `float32` | 4 |
| 12 | `output_min` | `float32` | 4 |
| 16 | `output_max` | `float32` | 4 |
| 20 | `vel_kp` | `float32` | 4 |
| 24 | `vel_ki` | `float32` | 4 |
| 28 | `vel_kd` | `float32` | 4 |
| 32 | `pos_kp` | `float32` | 4 |
| 36 | `pos_ki` | `float32` | 4 |
| 40 | `pos_kd` | `float32` | 4 |
| 44 | `velocity_feedforward` | `float32` | 4 |
| 48 | `position_feedforward` | `float32` | 4 |
| 52 | `reserved0` | `float32` | 4 |
| 56 | `reserved1` | `float32` | 4 |
| 60 | `sample_time_ms` | `uint32_t` | 4 |

Total subcommand payload bytes after common header: 64.
Total full configure payload length (including header): 67.

Sources:
- parse constants and required length math: `components/motor_manager/src/motor_manager_uart.cpp:13`, `components/motor_manager/src/motor_manager_uart.cpp:15`, `components/motor_manager/src/motor_manager_uart.cpp:136`
- field extraction order: `components/motor_manager/src/motor_manager_uart.cpp:143`

SetMode body:

| Relative Offset | Field | Type | Size | Values |
|---:|---|---|---:|---|
| 0 | `mode` | `uint8_t` | 1 | `0=open_loop`, `1=velocity`, `2=position` |

Source: `components/motor_manager/src/motor_manager_uart.cpp:194`, `components/motor_manager/src/motor_manager_uart.cpp:43`

SetPidParams body:

| Relative Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `pid_kind` | `uint8_t` | 1 (`0=velocity`, `1=position`) |
| 1 | `kp` | `float32` | 4 |
| 5 | `ki` | `float32` | 4 |
| 9 | `kd` | `float32` | 4 |
| 13 | `sample_time_ms` | `uint32_t` | 4 |

Sources: `components/motor_manager/src/motor_manager_uart.cpp:208`, `components/motor_manager/src/motor_manager_uart.cpp:214`, `components/motor_manager/include/motor_manager_types.hpp:37`

SetSetpoint body:

| Relative Offset | Field | Type | Size | Values |
|---:|---|---|---:|---|
| 0 | `setpoint_kind` | `uint8_t` | 1 | `0=duty`, `1=velocity_rad_s`, `2=position_rad` |
| 1 | `value` | `float32` | 4 | little-endian |

Sources: `components/motor_manager/src/motor_manager_uart.cpp:231`, `components/motor_manager/include/motor_manager_types.hpp:42`

StopMotor and ResetMotor bodies: no extra bytes beyond 3-byte common header.
Source: `components/motor_manager/src/motor_manager_uart.cpp:247`, `components/motor_manager/src/motor_manager_uart.cpp:251`

SetTelemetryGlobal body:

| Relative Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `enabled` | `uint8_t` | 1 (`0=off`, nonzero=on) |
| 1 | `field_mask` | `uint8_t` | 1 (`bit0=angle`, `bit1=velocity`) |

Sources: `components/motor_manager/src/motor_manager_uart.cpp:255`, `components/motor_manager/src/motor_manager_uart.cpp:260`, `components/motor_manager/include/motor_manager_types.hpp:57`

SetTelemetryMotor body:

| Relative Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `enabled` | `uint8_t` | 1 |
| 1 | `field_mask` | `uint8_t` | 1 |
| 2 | `update_field_mask` | `uint8_t` | 1 (`0=keep previous`, nonzero=replace`) |

Sources: `components/motor_manager/src/motor_manager_uart.cpp:267`, `components/motor_manager/src/motor_manager_uart.cpp:273`

### 3.2 Configure response payload (ESP -> host, msg type `0x61`)

Fixed 5-byte payload:

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 |
| 1 | `subcommand_echo` | `uint8_t` | 1 |
| 2 | `motor_id` | `uint8_t` | 1 |
| 3 | `result_code` | `uint8_t` (`ResultCode`) | 1 |
| 4 | `reserved` | `uint8_t` | 1 |

Sources: `components/motor_manager/src/motor_manager_uart.cpp:289`, `components/motor_manager/src/motor_manager_uart.cpp:291`

ResultCode values:
- `0x00 Ok`
- `0x01 InvalidArgument`
- `0x02 NotFound`
- `0x03 AlreadyExists`
- `0x04 CapacityReached`
- `0x05 InvalidState`
- `0x06 NotSupported`
- `0x07 InternalError`

Source: `components/motor_manager/include/motor_manager_types.hpp:13`

### 3.3 Setpoint stream payload (host -> ESP, channel `0x80`)

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 |
| 1 | `entry_count` | `uint8_t` | 1 |
| 2.. | repeated entries | see below | `entry_count * 6` |

Per entry (`kSetpointEntrySize=6`):

| Relative Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `motor_id` | `uint8_t` | 1 |
| 1 | `setpoint_kind` | `uint8_t` | 1 |
| 2 | `value` | `float32` LE | 4 |

Sources: `components/motor_manager/include/motor_manager_types.hpp:54`, `components/motor_manager/src/motor_manager_uart.cpp:79`, `components/motor_manager/src/motor_manager_uart.cpp:86`

### 3.4 Telemetry stream payload (ESP -> host, channel `0x81`)

Header:

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 |
| 1 | `entry_count` | `uint8_t` | 1 |
| 2 | `timestamp_us` | `uint64_t` LE | 8 |

Per entry:

| Relative Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `motor_id` | `uint8_t` | 1 |
| 1 | `field_mask` | `uint8_t` | 1 |
| 2.. | optional `angle_rad` | `float32` LE | 4 if bit0 |
| ... | optional `angular_velocity_rad_s` | `float32` LE | 4 if bit1 |

Field mask bits:
- bit0 (`0x01`): angle
- bit1 (`0x02`): velocity

Sources:
- header sizes: `components/motor_manager/include/motor_manager_types.hpp:62`, `components/motor_manager/include/motor_manager_types.hpp:63`
- timestamp + entry encoding: `components/motor_manager/src/motor_manager.cpp:549`, `components/motor_manager/src/motor_manager.cpp:554`
- per-entry field selection and ordering: `components/motor_manager/src/motor_manager.cpp:563`, `components/motor_manager/src/motor_manager.cpp:580`, `components/motor_manager/src/motor_manager.cpp:585`

## 4. IMU Protocol (`0x22`, `0x42`, `0xC2`, `0x82`)

### 4.1 Configure command payload (host -> ESP, msg type `0x22`)

Common header:

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 (`0x01`) |
| 1 | `subcommand` | `uint8_t` (`ConfigureSubcommand`) | 1 |
| 2.. | subcommand payload | varies | varies |

Sources: `components/imu_bmo_055/include/imu_bmo_055_protocol.hpp:31`, `components/imu_bmo_055/src/imu_bmo_055.cpp:507`

Subcommand values:
- `0x01 SetStreamEnabled`
- `0x02 SetStreamPeriodMs`
- `0x03 SetStreamFieldMask`
- `0x04 SetOperationMode`
- `0x05 ReadCalibration`
- `0x06 WriteCalibration`
- `0x07 ApplyCalibration`

Source: `components/imu_bmo_055/include/imu_bmo_055_protocol.hpp:9`

Bodies:
- `SetStreamEnabled`: byte2 `enabled` (`0`/nonzero)
- `SetStreamPeriodMs`: bytes2..3 `uint16` LE
- `SetStreamFieldMask`: bytes2..3 `uint16` LE
- `SetOperationMode`: byte2 mode (`0x00..0x0C`)
- `ReadCalibration`: no extra bytes
- `WriteCalibration`: 41-byte calibration blob from byte2 onward
- `ApplyCalibration`: no extra bytes

Sources: `components/imu_bmo_055/src/imu_bmo_055.cpp:514`, `components/imu_bmo_055/src/imu_bmo_055.cpp:520`, `components/imu_bmo_055/src/imu_bmo_055.cpp:532`, `components/imu_bmo_055/src/imu_bmo_055.cpp:544`, `components/imu_bmo_055/src/imu_bmo_055.cpp:550`, `components/imu_bmo_055/src/imu_bmo_055.cpp:562`, `components/imu_bmo_055/src/imu_bmo_055.cpp:570`

Calibration blob wire layout (`41` bytes):

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `has_sic` | `uint8_t` | 1 |
| 1 | `accel_offset_x` | `int16` LE | 2 |
| 3 | `accel_offset_y` | `int16` LE | 2 |
| 5 | `accel_offset_z` | `int16` LE | 2 |
| 7 | `accel_radius` | `int16` LE | 2 |
| 9 | `gyro_offset_x` | `int16` LE | 2 |
| 11 | `gyro_offset_y` | `int16` LE | 2 |
| 13 | `gyro_offset_z` | `int16` LE | 2 |
| 15 | `mag_offset_x` | `int16` LE | 2 |
| 17 | `mag_offset_y` | `int16` LE | 2 |
| 19 | `mag_offset_z` | `int16` LE | 2 |
| 21 | `mag_radius` | `int16` LE | 2 |
| 23 | `sic[0]` | `int16` LE | 2 |
| ... | `sic[1..8]` | `int16` LE | 16 |

Sources: `components/imu_bmo_055/src/imu_bmo_055.cpp:23`, `components/imu_bmo_055/src/imu_bmo_055.cpp:66`, `components/imu_bmo_055/src/imu_bmo_055.cpp:96`

### 4.2 Configure response payload (ESP -> host, msg type `0x42`)

Common response header (3 bytes):

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 |
| 1 | `subcommand_echo` | `uint8_t` | 1 |
| 2 | `result_code` | `uint8_t` | 1 |

Optional extra payload:
- only `ReadCalibration` on success appends 41-byte calibration blob.

Sources: `components/imu_bmo_055/include/imu_bmo_055_protocol.hpp:50`, `components/imu_bmo_055/src/imu_bmo_055.cpp:577`, `components/imu_bmo_055/src/imu_bmo_055.cpp:559`

ResultCode values:
- `0x00 Ok`
- `0x01 InvalidArgument`
- `0x02 InvalidState`
- `0x03 NotSupported`
- `0x04 Busy`
- `0x05 InternalError`

Source: `components/imu_bmo_055/include/imu_bmo_055_types.hpp:9`

### 4.3 IMU telemetry stream payload (ESP -> host, channel `0x82`)

Header:

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 |
| 1 | `field_mask` | `uint16` LE | 2 |
| 3 | `timestamp_us` | `uint64` LE | 8 |

Then fields are appended strictly in this order when corresponding field bit is set:
1. Euler (`h,r,p`) as `int16` each
2. Quaternion (`w,x,y,z`) as `int16` each
3. LinearAccel (`x,y,z`) as `int16` each
4. Gravity (`x,y,z`) as `int16` each
5. RawAccel (`x,y,z`) as `int16` each
6. RawGyro (`x,y,z`) as `int16` each
7. RawMag (`x,y,z`) as `int16` each
8. Temperature (`int8`)
9. CalibrationStatus (`sys,gyro,accel,mag` each `uint8`)

Sources: `components/imu_bmo_055/include/imu_bmo_055_protocol.hpp:49`, `components/imu_bmo_055/src/imu_bmo_055.cpp:126`, `components/imu_bmo_055/src/imu_bmo_055.cpp:139`, `components/imu_bmo_055/src/imu_bmo_055.cpp:145`, `components/imu_bmo_055/src/imu_bmo_055.cpp:149`, `components/imu_bmo_055/src/imu_bmo_055.cpp:155`, `components/imu_bmo_055/src/imu_bmo_055.cpp:173`, `components/imu_bmo_055/src/imu_bmo_055.cpp:177`, `components/imu_bmo_055/src/imu_bmo_055.cpp:184`

Stream field bit assignments:
- bit0 Euler
- bit1 Quaternion
- bit2 LinearAccel
- bit3 Gravity
- bit4 RawAccel
- bit5 RawGyro
- bit6 RawMag
- bit7 Temperature
- bit8 CalibrationStatus

Source: `components/imu_bmo_055/include/imu_bmo_055_protocol.hpp:19`

### 4.4 IMU event message (`0xC2`)

Event type exists as config and constant, but current manager source does not emit event payloads.

Sources: `components/imu_bmo_055/include/imu_bmo_055_protocol.hpp:35`, `components/imu_bmo_055/src/imu_bmo_055.cpp`

## 5. time_sync Protocol (`0x20`, `0x60`)

Important: direction is atypical.
- ESP sends request (`0x20`) to host.
- Host responds (`0x60`) to ESP.

Sources: `components/time_sync/Kconfig:3`, `components/time_sync/Kconfig:11`, `components/time_sync/time_sync.cpp:375`

### 5.1 Request payload (ESP -> host, msg type `0x20`)

Encoded size: 12 bytes.

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 (`1`) |
| 1 | `flags` | `uint8_t` | 1 (`0` currently) |
| 2 | `request_id` | `uint16` LE | 2 |
| 4 | `client_unix_send_us` | `int64` LE | 8 |

Sources: `components/time_sync/time_sync_protocol.hpp:26`, `components/time_sync/time_sync_protocol.cpp:44`, `components/time_sync/time_sync_protocol.cpp:45`, `components/time_sync/time_sync_protocol.cpp:46`, `components/time_sync/time_sync_protocol.cpp:47`

### 5.2 Response payload (host -> ESP, msg type `0x60`)

Encoded size: 30 bytes.

| Offset | Field | Type | Size |
|---:|---|---|---:|
| 0 | `version` | `uint8_t` | 1 |
| 1 | `status` | `uint8_t` | 1 (`0=success`) |
| 2 | `request_id` | `uint16` LE | 2 |
| 4 | `echoed_client_unix_send_us` | `int64` LE | 8 |
| 12 | `server_unix_rx_us` | `int64` LE | 8 |
| 20 | `server_unix_tx_us` | `int64` LE | 8 |
| 28 | `reserved0` | `uint8_t` | 1 (0) |
| 29 | `reserved1` | `uint8_t` | 1 (0) |

Sources: `components/time_sync/time_sync_protocol.hpp:27`, `components/time_sync/time_sync_protocol.cpp:72`, `components/time_sync/time_sync_protocol.cpp:73`, `components/time_sync/time_sync_protocol.cpp:74`, `components/time_sync/time_sync_protocol.cpp:75`, `components/time_sync/time_sync_protocol.cpp:76`, `components/time_sync/time_sync_protocol.cpp:78`, `components/time_sync/time_sync_protocol.cpp:79`

## 6. Required Host Initialization Sequences

### 6.1 Motor initialization sequence (per motor_id)

The firmware supports create+configure split and mode/setpoint operations. Recommended sequence:

1. Send `CreateMotor` (`subcommand=0x01`) with full motor config payload.
2. Wait for configure response (`0x61`) `result_code=0x00`.
3. Optionally send `SetPidParams` for velocity and position (`subcommand=0x05`) to tune at runtime.
4. Send `SetTelemetryGlobal` and/or `SetTelemetryMotor` (`0x09`/`0x0A`) to desired field mask.
5. Send `SetMode` (`0x04`) with mode byte (`0/1/2`).
6. Start setpoint updates using either:
   - `SetSetpoint` configure command (`0x06`) for command/response style, or
   - setpoint stream channel `0x80` for high-rate multi-motor updates.

Rationale from code paths:
- mode mapping and command parsing: `components/motor_manager/src/motor_manager_uart.cpp:194`
- setpoint stream parser: `components/motor_manager/src/motor_manager_uart.cpp:62`
- telemetry controls: `components/motor_manager/src/motor_manager_uart.cpp:255`, `components/motor_manager/src/motor_manager_uart.cpp:267`

### 6.2 IMU stream initialization sequence

1. Send IMU `SetStreamEnabled(0)` (`0x22`, subcommand `0x01`, value `0`).
2. Send optional `SetOperationMode` (`0x04`), valid `0x00..0x0C`.
3. Send `SetStreamFieldMask` (`0x03`) with desired field bits.
4. Send `SetStreamPeriodMs` (`0x02`) with desired period; firmware clamps to configured min/max.
5. Send `SetStreamEnabled(1)`.
6. Parse telemetry stream frames on channel `0x82`.

Sources: `components/imu_bmo_055/src/imu_bmo_055.cpp:514`, `components/imu_bmo_055/src/imu_bmo_055.cpp:544`, `components/imu_bmo_055/src/imu_bmo_055.cpp:532`, `components/imu_bmo_055/src/imu_bmo_055.cpp:520`, `components/imu_bmo_055/src/imu_bmo_055.cpp:358`, `components/imu_bmo_055/src/imu_bmo_055.cpp:622`

## 7. Timing Constraints and Throughput Limits

### 7.1 Explicit configured limits

- UART baud: `115200` (`sdkconfig:2368`)
- UART max payload: `128` (`sdkconfig:2369`)
- UART command TX queue depth: configurable (default entry in Kconfig): `components/uart_comm/Kconfig:57`
- UART RX delivery queue depth: `components/uart_comm/Kconfig:65`
- IMU stream period clamp: min `5 ms`, max `1000 ms` (`sdkconfig:2301`, `sdkconfig:2302`)

### 7.2 Implied operational limits

- No explicit "max command rate" guard in motor_manager/imu manager; overflow appears as queue full drops (`xQueueSend` failure logs) at component queue boundaries.
- For motor telemetry, update/send cadence is manager update task period (`mgr_cfg.update_task.period_ms` in main example is 10 ms).

Sources: `components/motor_manager/src/motor_manager.cpp:508`, `main/main.cpp:24`

⚠ VERIFY: measure practical sustained command rate on target hardware/baud; firmware source defines queue capacities but not formal throughput guarantees.

## 8. Known Quirks, Edge Cases, and Error Responses

1. `time_sync` request/response direction is opposite typical host-initiated design.
- Source: `components/time_sync/time_sync.cpp:375`

2. IMU event msg type (`0xC2`) is defined but not actively emitted in current manager source.
- Source: `components/imu_bmo_055/include/imu_bmo_055_protocol.hpp:35`

3. IMU stream carries raw sensor integer fields from Bosch driver structs; no conversion to ROS SI units in firmware.
- Source: `components/imu_bmo_055/src/bno055_device.cpp:164`, `components/imu_bmo_055/src/imu_bmo_055.cpp:145`

4. motor_manager create/configure parser expects two reserved float extension fields in payload. Omitting these causes `InvalidArgument`.
- Source: `components/motor_manager/src/motor_manager_uart.cpp:166`

5. UART parser junk discard computation uses `junk_len = pos - 1`; this may be intentional but is unusual.
- Source: `components/uart_comm/src/parser.cpp:82`
- ⚠ VERIFY: if packet loss appears near frame boundaries, inspect this line first.

6. Configure responses for motor_manager are always 5 bytes and include trailing reserved byte.
- Source: `components/motor_manager/src/motor_manager_uart.cpp:291`

7. Time sync non-zero status in response is treated as invalid response and sync aborts.
- Source: `components/time_sync/time_sync.cpp:401`

## 9. Byte Order Summary

- UART frame CRC bytes: little-endian (`CRC_LO`, `CRC_HI`) by explicit write order.
- `time_sync` protocol integers: explicit little-endian encode/decode helpers.
- `motor_manager` and IMU float/int payloads: encoded/decoded by raw byte copy from Xtensa little-endian memory layout (host must treat as little-endian).

Sources: `components/uart_comm/src/frame.cpp:33`, `components/time_sync/time_sync_protocol.cpp:2`, `components/motor_manager/src/motor_manager_uart.cpp:22`, `components/imu_bmo_055/src/imu_bmo_055.cpp:30`
