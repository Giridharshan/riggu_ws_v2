# Communication Layer Design (Host Side)

Goal: define a robust host-side transport/protocol library that speaks DriveLink UART firmware protocol and can be shared by multiple ros2_control hardware plugins.

Reference firmware protocol source is in `01_firmware_api_reference.md`.

## 1. Scope and Non-Goals

### 1.1 In scope

- UART transport open/close/reconnect.
- Frame encode/decode (`0xAA 0xAA 0xAA`, type, len, payload, CRC16).
- Command/response correlation for configure-style messages.
- Stream demux for setpoint, telemetry, IMU stream channels.
- Thread-safe API for ros2_control plugins.
- Telemetry/state cache for realtime `read()` access.

### 1.2 Out of scope

- ros2_control lifecycle code itself (covered in Phase 4).
- Controller configuration YAML.
- Any firmware changes.

## 2. High-Level Architecture

Proposed library module name: `drivelink_comm`.

Primary objects:

1. `SerialPort`
- Owns OS serial handle and read/write syscalls.
- Provides blocking read with timeout and serialized write.

2. `DriveLinkFrameCodec`
- `encode(msg_type, payload)`
- incremental parse state machine + CRC verification
- reports parser stats/counters

3. `DriveLinkSession`
- Owns RX thread and parser.
- Registers callbacks per `msg_type` and per stream channel.
- Manages command-response promises by `(msg_type, request_key)` policy.

4. `DriveLinkStateStore`
- Latest motor telemetry by `motor_id`.
- Latest IMU sample (converted + raw fields).
- Timestamped health status and error counters.

5. `DriveLinkClient`
- Public API consumed by ros2_control hardware plugins.
- Higher-level methods (`create_motor`, `set_mode`, `set_setpoint`, `set_imu_stream`, etc.).

## 3. API Surface (C++ Sketch)

```cpp
struct CommConfig {
  std::string port;
  int baudrate;               // default 115200
  std::chrono::milliseconds read_timeout;
  std::chrono::milliseconds cmd_timeout;
  int reconnect_backoff_ms;
};

class DriveLinkClient {
public:
  bool start(const CommConfig& cfg);
  void stop();
  bool is_connected() const;

  // Motor manager (0x21/0x61)
  ResultCode create_or_configure_motor(uint8_t motor_id, const MotorConfig& cfg, bool create);
  ResultCode delete_motor(uint8_t motor_id);
  ResultCode set_mode(uint8_t motor_id, ControlMode mode);
  ResultCode set_pid(uint8_t motor_id, PidKind kind, const PidParams& p);
  ResultCode set_setpoint(uint8_t motor_id, SetpointKind kind, float value);
  ResultCode stop_motor(uint8_t motor_id);
  ResultCode reset_motor(uint8_t motor_id);
  ResultCode set_telemetry_global(bool enabled, uint8_t field_mask);
  ResultCode set_telemetry_motor(uint8_t motor_id, bool enabled, uint8_t field_mask, bool update_mask);

  // Setpoint stream (0x80)
  bool publish_setpoint_batch(const std::vector<SetpointEntry>& entries);

  // IMU manager (0x22/0x42 + 0x82)
  ImuResult set_imu_stream_enabled(bool enabled);
  ImuResult set_imu_stream_period_ms(uint16_t period_ms);
  ImuResult set_imu_stream_field_mask(uint16_t field_mask);
  ImuResult set_imu_operation_mode(uint8_t mode);
  ImuResult read_imu_calibration(CalibrationBlob& out);
  ImuResult write_imu_calibration(const CalibrationBlob& in);
  ImuResult apply_imu_calibration();

  // Time sync (0x20/0x60)
  void set_time_sync_handler(std::function<void(const TimeSyncRequest&)> handler);

  // Data access for realtime
  bool get_latest_motor_state(uint8_t motor_id, MotorState& out) const;
  bool get_latest_imu_state(ImuState& out) const;
};
```

## 4. Threading Model

Threads:

1. RX thread (blocking serial read)
- reads bytes from UART
- feeds parser
- dispatches complete frames to handler path

2. Optional TX worker thread
- if needed for rate limiting / batching; otherwise sync writes guarded by mutex

3. ros2_control realtime thread(s)
- call into `DriveLinkClient` read-only getters with non-blocking semantics
- never perform blocking serial operations directly

Synchronization:

- `write_mutex` for wire writes.
- lock-free ring buffer or double-buffer for latest IMU sample.
- per-motor atomic or mutex-protected state structs.
- bounded queue for outgoing commands if asynchronous API is desired.

## 5. Frame Handling and Validation

### 5.1 Encode path

- Build header: start bytes + type + payload length.
- Append payload.
- Compute CRC over `[type, len, payload]` and append low/high bytes.

### 5.2 Parse path

- Seek start marker `AA AA AA`.
- Read header and payload length.
- Enforce max payload (128 from current firmware config).
- Verify CRC; increment error counters on mismatch.
- Dispatch by `msg_type` class.

### 5.3 Parser quirks compatibility

Firmware parser has unusual junk-discard arithmetic (`pos - 1`) noted in Phase 1; host parser should be conservative and robust to boundary noise.

## 6. Command/Response Correlation

Motor manager responses (`0x61`) include:
- version
- subcommand echo
- motor_id
- result

Correlation key:
- `(subcommand, motor_id)` plus outstanding call context.

IMU configure responses (`0x42`) include:
- version
- subcommand echo
- result

Correlation key:
- `(subcommand)` since no motor ID.

Timeout policy:
- default 100-300 ms for configure commands (tunable).
- return transport timeout distinct from firmware `ResultCode`.

## 7. Stream Demux and State Cache

### 7.1 Motor telemetry channel (`0x81`)

On receipt:
- parse global timestamp and entries
- update per-motor `{angle_rad, angular_velocity_rad_s, source_timestamp_us, host_rx_time}`

### 7.2 IMU telemetry channel (`0x82`)

On receipt:
- parse selected fields by bitmask in canonical order
- store raw fields and converted SI fields

Unit conversion policy:
- keep both raw + converted values in cache
- apply conversion only once on ingest

⚠ VERIFY required constants against selected BNO055 operation mode and expected units.

### 7.3 Setpoint stream (`0x80`)

Publish path only from host; no cache needed except optional last-command mirror for diagnostics.

## 8. Reconnect and Fault Strategy

Connection states:
- `DISCONNECTED`
- `CONNECTING`
- `ACTIVE`
- `DEGRADED`

Transitions:

1. `start()` attempts open -> `ACTIVE` on success.
2. read/write failure -> `DEGRADED`.
3. retry loop with exponential backoff -> `ACTIVE` on success or `DISCONNECTED` after max retries.
4. lifecycle `on_deactivate` / `on_cleanup` should always force `DISCONNECTED`.

Behavior during disconnect:
- command APIs fail fast with transport error.
- getters continue returning last known sample + staleness metadata.

## 9. Time Sync Handling

Firmware initiates request (`0x20`), host must send response (`0x60`).

Design:
- RX handler detects `0x20` and invokes time provider callback.
- Default callback computes server RX/TX timestamps from host steady/system clock pair.
- Build and transmit response within bounded latency.

⚠ VERIFY clock domain mapping policy (UNIX microseconds expected by firmware).

## 10. Diagnostics and Introspection

Expose counters:
- total RX/TX frames
- CRC failures
- length violations
- unknown `msg_type`
- command timeouts
- reconnect count

Expose health snapshot:
- connection state
- last RX timestamp
- per-channel freshness age

These diagnostics should be consumable by ros2_control hardware components for status publishing and log throttling.

## 11. Test Plan for Comm Layer

1. Codec unit tests
- known-good frame roundtrip
- CRC mismatch detection
- malformed lengths

2. Parser fuzz tests
- random byte noise with valid-frame insertion

3. Integration tests with pseudo-terminal loopback
- scripted firmware-like responses for `0x21/0x61`, `0x22/0x42`, streams

4. Hardware-in-loop smoke tests
- connect to ESP board
- create motor, set mode, stream telemetry, IMU stream receive

## 12. Deliverables for Implementation Agent

1. `drivelink_comm` static/shared library.
2. Public headers for `DriveLinkClient`, protocol structs, error enums.
3. Unit tests (codec/parser/correlation).
4. Sample executable for manual UART interaction.
5. Clear ABI boundary for ros2_control plugins (Phase 4).
