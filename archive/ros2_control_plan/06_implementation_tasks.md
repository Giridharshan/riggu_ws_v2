# Implementation Task Backlog

This backlog is ordered for one implementation agent working without firmware repo access.

## 1. Milestone Plan

### M0 - Environment and Distro Lock

1. Confirm target ROS 2 distro (`Humble` or `Jazzy/Rolling`).
2. Select API style accordingly (old `HardwareInfo` init + export vectors vs newer interface-description API).
3. Scaffold package and baseline build.

Exit criteria:
- package builds with empty plugin stubs
- pluginlib can discover both plugin classes

### M1 - Comm Layer Core

1. Implement frame codec and parser.
2. Implement serial open/read/write with timeout.
3. Implement session RX thread and dispatch.
4. Implement command-response waiter abstraction.
5. Add state cache for motor telemetry + IMU.

Exit criteria:
- unit tests pass for codec/parser/timeout paths
- simulated responder integration works for `0x21/0x61`, `0x22/0x42`, streams

### M2 - System Hardware Plugin (motors)

1. Implement `DriveLinkSystemHardware` lifecycle.
2. Parse wheel->motor mappings and validate interfaces.
3. Wire `read()` from telemetry cache.
4. Wire `write()` to setpoint stream.
5. Implement safe deactivate/stop behavior.

Exit criteria:
- loads/configures/activates under controller manager
- can drive wheels via `diff_drive_controller`

### M3 - IMU Sensor Plugin

1. Implement `DriveLinkImuSensorHardware` lifecycle.
2. Configure IMU stream in `on_configure`.
3. Read IMU cache and expose semantic interface names.
4. Validate with `imu_sensor_broadcaster`.

Exit criteria:
- `imu_sensor_broadcaster` activates
- `sensor_msgs/Imu` publishes valid data

### M4 - Shared Ownership and Robustness

1. Implement shared client registry across both plugins.
2. Add reconnect logic and stale data diagnostics.
3. Implement mode-switch callbacks in system plugin.
4. Harden error mapping from firmware result codes.

Exit criteria:
- no duplicate serial open when both plugins active
- controlled behavior on serial disconnect/reconnect

### M5 - Packaging, Tests, and Docs

1. Complete launch/config assets.
2. Add plugin load and integration tests.
3. Write operator README and troubleshooting guide.

Exit criteria:
- `colcon test` green
- launch flow reproducible on clean machine

## 2. Detailed Task List

## Task Group A - Scaffold and Compatibility

1. Create package `drivelink_ros2_control` with headers/sources/config/launch/test folders.
2. Add dependencies and plugin XML wiring in CMake/package.xml.
3. Add compile-time guard macros if distro-specific code paths are needed.

Acceptance:
- build succeeds
- `pluginlib` load test passes for both plugin classes

## Task Group B - Protocol Types and Codec

1. Define strongly typed enums/structs mirroring firmware payloads.
2. Implement serializer/deserializer helpers with little-endian handling.
3. Implement CRC function matching firmware behavior.
4. Build parser state machine with bounded memory use.

Acceptance:
- golden frame tests for motor configure, response, telemetry, IMU stream
- malformed frame tests detect and recover

## Task Group C - Serial Session and Dispatch

1. Implement serial abstraction (`open`, `close`, `read_exact`, `write_all`).
2. Start RX thread and parse frames continuously.
3. Dispatch callbacks by message type/channel.
4. Add command timeout and cancellation logic.

Acceptance:
- simulated loopback test handles concurrent streams + command responses
- no deadlocks under timeout stress

## Task Group D - Motor Manager API Wrapper

1. Implement wrappers for all motor configure subcommands.
2. Implement setpoint stream batch publisher.
3. Implement telemetry parser to update per-motor state cache.
4. Implement firmware result-code mapping.

Acceptance:
- can create/configure motor and receive `Ok`
- telemetry cache updates with monotonic timestamps

## Task Group E - IMU API Wrapper

1. Implement IMU configure wrappers (`set_period`, `set_mask`, `set_mode`, calibration calls).
2. Parse IMU telemetry bitmask payload with optional fields.
3. Add raw+converted value cache.
4. Implement optional time-sync request handler.

Acceptance:
- IMU stream parsing verified against captured frames
- conversion outputs match expected ranges

## Task Group F - Shared Client Registry

1. Implement ref-counted singleton/factory for `DriveLinkClient` per serial key.
2. Ensure thread-safe acquire/release.
3. Prevent duplicate opens and double-stop.

Acceptance:
- both plugins can run simultaneously with one serial session
- plugin unload/load cycles do not leak threads

## Task Group G - System Plugin Implementation

1. Implement lifecycle callbacks.
2. Validate URDF interfaces and params in `on_init`.
3. Configure firmware motors in `on_configure`.
4. Activate/deactivate behavior with safe stop semantics.
5. Implement `read`/`write` non-blocking cache interactions.
6. Implement `prepare_command_mode_switch` and `perform_command_mode_switch`.

Acceptance:
- `diff_drive_controller` can claim interfaces and command motion
- mode switches do not crash and fail cleanly on invalid requests

## Task Group H - IMU Sensor Plugin Implementation

1. Implement lifecycle callbacks and IMU setup.
2. Expose exact semantic IMU state interface names.
3. Implement `read` cache copy and stale handling.

Acceptance:
- `imu_sensor_broadcaster` interface claim succeeds
- published IMU topic updates at configured rate

## Task Group I - Diagnostics and Error Handling

1. Add structured logs for lifecycle and protocol faults.
2. Add diagnostics snapshot API/counters.
3. Define stale-data thresholds and warning/error transitions.

Acceptance:
- faults are visible with actionable logs
- stale transport condition reflected in diagnostics

## Task Group J - Launch and Controller Config

1. Add controller YAML for manager + broadcasters + diff drive.
2. Add launch file to start full stack.
3. Add sample URDF/xacro with both plugins.

Acceptance:
- one-command launch works on test machine
- controller spawn order stable

## Task Group K - Tests

1. Unit tests: codec/parser/protocol mapping.
2. Plugin load tests via `pluginlib`.
3. Integration tests with pseudo-terminal firmware simulator.
4. Optional HIL scripts for real board smoke test.

Acceptance:
- all tests pass in CI/local
- HIL checklist validated manually

## 3. Risk Register and Mitigations

1. Distro API mismatch.
Mitigation: lock distro before coding, use compile guards only if unavoidable.

2. Serial contention between plugins.
Mitigation: mandatory shared client registry and ownership checks.

3. Realtime jitter from blocking I/O.
Mitigation: all serial operations in comm thread; `read`/`write` cache-only.

4. IMU unit conversion ambiguity.
Mitigation: preserve raw values, parameterize scales, validate on known motion patterns.

5. Firmware command sequencing edge cases.
Mitigation: strict state machine and retry/backoff in configure paths.

## 4. Suggested Work Breakdown (Single Agent)

1. Days 1-2: A+B+C
2. Days 3-4: D+G baseline diff-drive run
3. Day 5: E+H IMU integration
4. Day 6: F+I robustness
5. Day 7: J+K and documentation

(Adjust based on actual complexity and hardware access.)

## 5. Definition of Done

1. Both plugins load and pass lifecycle transitions.
2. `diff_drive_controller` controls wheels through firmware.
3. `imu_sensor_broadcaster` publishes valid IMU topic.
4. Serial reconnect and fault handling are tested.
5. Unit + integration tests are passing.
6. Launch/config/docs enable another engineer to reproduce setup end-to-end.
