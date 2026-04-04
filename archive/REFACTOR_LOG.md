# Refactor Log

## Package Overview

ROS 2 hardware package implementing a pluginlib-exported hardware_interface backend for DriveLink hardware, with transport/protocol handling, sensor/system interfaces, tests, and a mock firmware tool.

## File Inventory

```text
./CMakeLists.txt
./REFACTOR_LOG.md
./drivelink_hardware.xml
./include/drivelink_hardware/crc16.hpp
./include/drivelink_hardware/drivelink_comm.hpp
./include/drivelink_hardware/drivelink_comm_registry.hpp
./include/drivelink_hardware/drivelink_imu_sensor.hpp
./include/drivelink_hardware/drivelink_messages.hpp
./include/drivelink_hardware/drivelink_system.hpp
./package.xml
./src/drivelink_comm.cpp
./src/drivelink_comm_registry.cpp
./src/drivelink_imu_sensor.cpp
./src/drivelink_system.cpp
./test/test_comm.cpp
./test/test_crc16.cpp
./test/test_plugin_list.cpp
./test/test_system_lifecycle.cpp
./tools/mock_firmware.py
```

## Analysis Notes

### File: CMakeLists.txt
- What it does: Defines build targets for the shared hardware plugin library, standalone test executables, install/export rules, and pluginlib descriptor export.
- Lines of code: 102
- Classes/functions present: CMake target declarations (`add_library`, `add_executable`), dependency wiring (`find_package`, `ament_target_dependencies`), install/export blocks.
- Problems found:
	- Test targets are always built and installed; they are not wrapped in `BUILD_TESTING`, which is atypical for ROS 2 package hygiene.
	- Large monolithic source files are compiled directly; no intermediate modular targets to enforce separation.
	- Comment references "task 1" and may be stale project-management context in build config.
- Suggested split/rename (>~200 lines): N/A (file is 102 lines).

### File: REFACTOR_LOG.md
- What it does: Maintains ongoing analysis, planning, implementation checkpoints, and rationale for this refactor.
- Lines of code: 13 (initial scaffold at creation time)
- Classes/functions present: N/A (documentation file).
- Problems found: None; structure matches requested sections.
- Suggested split/rename (>~200 lines): N/A.

### File: drivelink_hardware.xml
- What it does: Declares pluginlib classes for system and IMU sensor hardware interfaces.
- Lines of code: 14
- Classes/functions present: plugin descriptors for `drivelink_hardware::DriveLinkSystem` and `drivelink_hardware::DriveLinkImuSensor`.
- Problems found:
	- Descriptor coverage appears correct for both plugin types; no immediate structural issue found.
	- No versioning or compatibility note in description strings (optional improvement).
- Suggested split/rename (>~200 lines): N/A.

### File: package.xml
- What it does: Defines package metadata and dependencies for build/runtime/testing.
- Lines of code: 30
- Classes/functions present: N/A (manifest file).
- Problems found:
	- `controller_manager` is listed as a hard dependency while CMake marks it optional (`QUIET`), creating potential metadata/build-policy mismatch.
	- Lint test dependencies are present but no lint gating appears in CMake (possible inconsistency).
- Suggested split/rename (>~200 lines): N/A.

### File: include/drivelink_hardware/crc16.hpp
- What it does: Provides constexpr CRC-16 table generation and runtime CRC computation helper.
- Lines of code: 48
- Classes/functions present: `detail::crc16_table_entry`, `detail::make_crc16_table`, `crc16`.
- Problems found:
	- Public function and helper APIs lack Doxygen-style documentation.
	- No explicit test vector comments near implementation to tie behavior to wire protocol variant.
- Suggested split/rename (>~200 lines): N/A.

### File: include/drivelink_hardware/drivelink_comm_registry.hpp
- What it does: Declares a process-local singleton-like registry to share one `DriveLinkComm` object with reference counting.
- Lines of code: 28
- Classes/functions present: `DriveLinkCommRegistry`, static methods `acquire` and `release`.
- Problems found:
	- Public class and methods do not have Doxygen-style docstrings.
	- Global mutable static state implies cross-plugin-instance coupling; should be clearly justified and documented for thread/process safety expectations.
- Suggested split/rename (>~200 lines): N/A.

### File: include/drivelink_hardware/drivelink_comm.hpp
- What it does: Declares serial communication transport, frame parser, protocol send helpers, and callback dispatch for motor/IMU/time-sync traffic.
- Lines of code: 114
- Classes/functions present: `DriveLinkComm` with open/close/send APIs, parser state machine helpers, callback setters, and telemetry handlers.
- Problems found:
	- Public class and all public methods lack Doxygen-style documentation.
	- Multiple responsibilities combined in one class (serial fd lifecycle, protocol encode/decode, callback fanout, read-thread management), making the corresponding implementation a likely god-class.
	- Default serial port and baudrate are hardcoded in constructor defaults; should align with parameter flow from hardware info.
- Suggested split/rename (>~200 lines): Header size is fine, but implementation should be split into focused translation units (I/O, parser, command encoder).

### File: include/drivelink_hardware/drivelink_imu_sensor.hpp
- What it does: Declares IMU `hardware_interface::SensorInterface` plugin lifecycle, interface export, telemetry parsing, and state publishing fields.
- Lines of code: 90
- Classes/functions present: `DriveLinkImuSensor`, nested `ImuSnapshot`, lifecycle hooks (`on_init/on_configure/on_activate/on_deactivate/on_cleanup/on_shutdown/on_error`), `read`, parameter parsing helpers.
- Problems found:
	- Public class and methods lack Doxygen-style docs.
	- Uses `std::unordered_map` in method signatures but header does not include `<unordered_map>` explicitly (relies on transitive include).
	- Hardcoded defaults (`uart_baud_`, stream period) should be clearly documented as fallback behavior for missing hardware parameters.
- Suggested split/rename (>~200 lines): Header size acceptable; corresponding source should be reviewed for decomposition because it is >250 lines.

### File: include/drivelink_hardware/drivelink_messages.hpp
- What it does: Defines wire protocol message enums and packed payload/frame structures shared across comm/system/sensor components.
- Lines of code: 333
- Classes/functions present: Message enums (`MessageType`, `MotorConfigureSubcommand`, etc.), many packed structs for command/response/telemetry payloads, static asserts for ABI size.
- Problems found:
	- File exceeds the ~200 line threshold and combines heterogeneous concerns (motor, IMU, time sync, frame envelope).
	- No Doxygen docs on protocol enums/structs, making firmware/host contract harder to audit.
	- `kDriveLinkStartByte` is declared at global scope outside namespace, leaking symbol globally.
	- Uses GCC/Clang-specific `__attribute__((packed))`; acceptable here but should be centralized/abstracted if portability is desired.
- Suggested split/rename (>~200 lines): Split into `protocol/frame.hpp`, `protocol/motor_messages.hpp`, `protocol/imu_messages.hpp`, and `protocol/time_sync_messages.hpp`.

### File: include/drivelink_hardware/drivelink_system.hpp
- What it does: Declares primary `hardware_interface::SystemInterface` implementation for motor joint control, lifecycle handling, command/state export, and comm callbacks.
- Lines of code: 110
- Classes/functions present: `DriveLinkSystem`, nested `JointContext`, full lifecycle hooks (`on_init/on_configure/on_activate/on_deactivate/on_cleanup/on_shutdown/on_error`), read/write, command mode switch methods, parameter/helper methods.
- Problems found:
	- Public class and methods lack Doxygen-style docs.
	- Header relies on `std::chrono::milliseconds` in method signature but does not include `<chrono>` explicitly.
	- `JointContext` aggregates parsing config + runtime state + command tracking (mixed concerns, likely mirrored by large source file complexity).
- Suggested split/rename (>~200 lines): Header acceptable size; source should be decomposed because it is 645 lines.

### File: src/drivelink_comm_registry.cpp
- What it does: Implements static shared-communication registry acquisition and release logic with simple ref-counting.
- Lines of code: 59
- Classes/functions present: `DriveLinkCommRegistry::acquire`, `DriveLinkCommRegistry::release`, static member definitions.
- Problems found:
	- Uses `std::cerr` logging instead of ROS logging macros.
	- Shared singleton state with ref-count can hide lifecycle ownership bugs across multiple hardware plugin instances.
	- Missing Doxygen docs for implemented public methods.
- Suggested split/rename (>~200 lines): N/A.

### File: src/drivelink_comm.cpp
- What it does: Implements serial port setup, frame TX/RX, background reader thread, parser state machine, protocol message encode/decode, and callback dispatch.
- Lines of code: 695
- Classes/functions present: `DriveLinkComm` ctor/dtor/open/close/send APIs, parser (`process_rx_byte`), dispatcher (`dispatch_frame`), helpers for motor/IMU/time-sync commands and telemetry parsing.
- Problems found:
	- Significant god-class and mixed concerns in one file (serial config, transport retries, parser state machine, protocol-level handlers).
	- Extensive `std::cerr` usage instead of `RCLCPP_*` logging.
	- File exceeds modularity target by a large margin (>250 lines).
	- `close()` can block up to 2s waiting for reader thread; acceptable for lifecycle transitions but should not be used in real-time paths.
	- No Doxygen docs for public API.
- Suggested split/rename (>~200 lines): Split into `drivelink_serial_transport.cpp`, `drivelink_frame_parser.cpp`, and `drivelink_command_encoder.cpp` (or equivalent).

### File: src/drivelink_imu_sensor.cpp
- What it does: Implements IMU sensor plugin lifecycle, parameter parsing from `hardware_parameters`, state interface export, telemetry decode, and shared comm lifecycle.
- Lines of code: 374
- Classes/functions present: `DriveLinkImuSensor` lifecycle methods, `read`, parameter helper methods, `parse_imu_telemetry`, `release_comm`, pluginlib export macro.
- Problems found:
	- File exceeds modularity target (>250 lines) and mixes lifecycle control with telemetry decoding details.
	- Public methods lack Doxygen docs.
	- Contains hardcoded telemetry parsing constants inline (`kQuatScale` etc.) instead of centralized constants.
	- `read()` uses `try_lock` (good non-blocking behavior) but silently drops updates under lock contention with no optional diagnostics.
	- Pluginlib export macro is present and appears correct.
- Suggested split/rename (>~200 lines): Separate telemetry parsing helpers into dedicated source/header pair.

### File: src/drivelink_system.cpp
- What it does: Implements motor system plugin lifecycle, hardware parameter extraction, joint configuration, command/state interface handling, command mode switching, and comm callback integration.
- Lines of code: 645
- Classes/functions present: `DriveLinkSystem` lifecycle methods, `read`/`write`, mode-switch methods, parameter parsers, telemetry/response callbacks, shutdown helpers, pluginlib export macro.
- Problems found:
	- Very large god-file with mixed concerns (URDF param validation, runtime state machine, command dispatch, response waiting logic).
	- Public methods lack Doxygen docs.
	- Inline "VERIFY" comments and magic values (`500 ms`, `1e-6`, `1e-4`) should move to named constants.
	- `read()` is non-blocking (good), but currently no-op except lock/unlock; all state updates happen asynchronously in callback path.
	- `write()` performs serial sends synchronously; this may introduce jitter depending on UART/backpressure behavior.
	- Lifecycle compliance: all major hooks are implemented; parameters are loaded through `info.hardware_parameters` in `on_init`.
	- Pluginlib export macro is present and appears correct.
- Suggested split/rename (>~200 lines): Split into lifecycle/config module, command-mode/write module, and telemetry/response module.

### File: test/test_crc16.cpp
- What it does: Standalone executable test validating CRC16 implementation against a bitwise reference algorithm and basic vectors.
- Lines of code: 63
- Classes/functions present: helper functions `crc16_reference_bitwise`, `print_result`, `main`.
- Problems found:
	- Uses custom print-based assertions instead of gtest despite package including test dependencies.
	- No negative/fuzz boundary tests beyond minimal vectors.
- Suggested split/rename (>~200 lines): N/A.

### File: test/test_comm.cpp
- What it does: Integration-style executable test using `socketpair` to validate frame parsing, CRC rejection, setpoint send path, and time-sync response behavior.
- Lines of code: 189
- Classes/functions present: `make_frame`, `write_all`, `approx_equal`, `print_case`, `main`.
- Problems found:
	- Uses sleep-based timing (`sleep_for(100ms)`), which can be flaky under load.
	- Uses print-based pass/fail instead of gtest assertions.
	- Duplicates frame build logic from production path in test helper (risk of mirrored bugs).
- Suggested split/rename (>~200 lines): N/A.

### File: test/test_plugin_list.cpp
- What it does: Validates pluginlib discovery of system and sensor classes.
- Lines of code: 52
- Classes/functions present: helper `contains`, `main`.
- Problems found:
	- Uses print-based checks rather than gtest.
	- Duplicates acceptable class-name matching in ad-hoc form (could centralize expected plugin IDs/constants).
- Suggested split/rename (>~200 lines): N/A.

### File: test/test_system_lifecycle.cpp
- What it does: Exercises minimal system init/configure lifecycle path and expects configure failure without a real comm backend.
- Lines of code: 62
- Classes/functions present: `main`.
- Problems found:
	- Uses print-based checks rather than gtest assertions.
	- Relies on deprecated initialization API path pragmas.
	- Missing explicit `<iostream>` include while using `std::cout`.
- Suggested split/rename (>~200 lines): N/A.

### File: tools/mock_firmware.py
- What it does: Emulates firmware-side serial protocol behavior for motor, IMU, and time-sync traffic to support local integration testing.
- Lines of code: 285
- Classes/functions present: `crc16`, `make_frame`, `configure_serial`, class `MockFirmware` with send/parse/spin handlers, `main`.
- Problems found:
	- File exceeds modularity target (>250 lines) with parser, model state, and runtime loop all in one class/file.
	- Uses print logging only; acceptable for tool script, but no log-level controls.
	- Protocol constants are duplicated relative to C++ headers (risk of drift).
- Suggested split/rename (>~200 lines): Split into `mock_protocol.py` and `mock_runtime.py` modules.

### Cross-Cutting ROS2 Hardware Contract Checks
- `hardware_interface` inheritance:
	- `DriveLinkSystem` extends `hardware_interface::SystemInterface`.
	- `DriveLinkImuSensor` extends `hardware_interface::SensorInterface`.
- Lifecycle coverage:
	- Both plugins implement `on_init`, `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown`, and `on_error`.
- Interface export correctness (initial assessment):
	- `DriveLinkSystem` exports command/state interfaces for all declared joints.
	- `DriveLinkImuSensor` exports 10 IMU semantic state interfaces.
- `read()` / `write()` blocking behavior:
	- `read()` in both plugins uses `try_lock` and is non-blocking.
	- `write()` is synchronous on serial sends and may incur non-deterministic latency under transport backpressure.
- Parameter sourcing:
	- Core parameters are loaded from `info.hardware_parameters` in `on_init` for both plugins.
- pluginlib registration:
	- `PLUGINLIB_EXPORT_CLASS` present for both system and sensor plugins.
- Direct `rclcpp::Node` usage:
	- No direct node ownership/creation detected in hardware plugins; logging uses `get_logger()`.

## Refactoring Plan

### New Files To Create
- `include/drivelink_hardware/constants.hpp`: Shared constants for protocol/logging/runtime thresholds and fallback values.
- `include/drivelink_hardware/protocol/frame.hpp`: Frame envelope/start-byte definitions.
- `include/drivelink_hardware/protocol/motor_messages.hpp`: Motor protocol enums and packed payload structs.
- `include/drivelink_hardware/protocol/imu_messages.hpp`: IMU protocol enums and packed payload structs.
- `include/drivelink_hardware/protocol/time_sync_messages.hpp`: Time sync payload structs.
- `src/drivelink_comm_io.cpp`: DriveLinkComm open/close/send and TX-side helpers.
- `src/drivelink_comm_rx.cpp`: DriveLinkComm reader-loop/parser/dispatch RX behavior.
- `src/drivelink_system_lifecycle.cpp`: DriveLinkSystem lifecycle/init/export methods.
- `src/drivelink_system_runtime.cpp`: DriveLinkSystem read/write/mode switch/runtime callback logic.
- `src/drivelink_imu_sensor_lifecycle.cpp`: IMU lifecycle/init/export/read methods.
- `src/drivelink_imu_sensor_parser.cpp`: IMU payload parsing and decode helpers.

### Files To Split
- `include/drivelink_hardware/drivelink_messages.hpp`:
	- Move frame definitions to `protocol/frame.hpp`.
	- Move motor enums/structs to `protocol/motor_messages.hpp`.
	- Move IMU enums/structs to `protocol/imu_messages.hpp`.
	- Move time sync structs to `protocol/time_sync_messages.hpp`.
	- Keep `drivelink_messages.hpp` as umbrella include for backwards compatibility.
- `src/drivelink_comm.cpp`:
	- TX/open/close API and command encoders -> `src/drivelink_comm_io.cpp`.
	- RX loop/parser/dispatch/time-sync handling -> `src/drivelink_comm_rx.cpp`.
- `src/drivelink_system.cpp`:
	- Lifecycle/config/export methods -> `src/drivelink_system_lifecycle.cpp`.
	- Read/write/mode-switch/telemetry-response helpers -> `src/drivelink_system_runtime.cpp`.
- `src/drivelink_imu_sensor.cpp`:
	- Lifecycle/export/read/init -> `src/drivelink_imu_sensor_lifecycle.cpp`.
	- Telemetry decode/parser helper -> `src/drivelink_imu_sensor_parser.cpp`.

### Files To Rename
- No public filename rename planned for API stability.
- Legacy monolithic source files are replaced by split source files (old files removed after wiring CMake).

### Ordered Implementation Checklist
- [x] Add `constants.hpp` and shared protocol sub-headers; convert `drivelink_messages.hpp` to umbrella include.
- [x] Add Doxygen docs for every public class and public method in key headers (`crc16.hpp`, `drivelink_comm.hpp`, `drivelink_comm_registry.hpp`, `drivelink_system.hpp`, `drivelink_imu_sensor.hpp`).
- [x] Split `DriveLinkComm` implementation into TX/I/O and RX/parser files; replace `std::cerr` with ROS logging.
- [x] Split `DriveLinkSystem` implementation into lifecycle and runtime files; centralize magic values via constants.
- [x] Split `DriveLinkImuSensor` implementation into lifecycle and parser files; centralize parse constants.
- [x] Update `CMakeLists.txt` source lists and testing sections for new file layout and ROS2-idiomatic `BUILD_TESTING` gating.
- [x] Verify `package.xml` and optional dependency alignment (`controller_manager`) with CMake behavior.
- [x] Validate plugin descriptor accuracy (`drivelink_hardware.xml`) against exported classes.
- [x] Build and run package tests; fix any introduced issues.
- [x] Write final summary in this log with changed files, architecture decisions, and manual hardware validation steps.

### API/Interface Changes For Callers
- No expected runtime API changes to plugin class names or pluginlib identifiers.
- Internal header layout changes add protocol sub-headers while retaining `drivelink_messages.hpp` compatibility include.
- Build graph changes only affect maintainers (source split and test gating); consumer-facing plugin contracts remain unchanged.

### Plan Coverage Check
- In-code documentation: covered by checklist item 2.
- Modularity/no file >~250 lines: covered by checklist items 1, 3, 4, 5.
- ROS2 lifecycle compliance: preserved and audited in checklist items 4 and 5.
- Parameter handling via `hardware_parameters`: preserved during lifecycle split (items 4 and 5).
- Thread safety in `read()`/`write()`: reviewed and tightened with constants/logging updates in items 3-5.
- CMake/package consistency: covered by items 6 and 7.
- pluginlib XML descriptor accuracy: covered by item 8.

## Implementation Progress

- Completed checklist item 1: Introduced `include/drivelink_hardware/constants.hpp` and split protocol types into `include/drivelink_hardware/protocol/*.hpp`. `drivelink_messages.hpp` now acts as a compatibility umbrella include to preserve existing include paths while reducing monolithic protocol file size.
- Completed checklist item 2: Added Doxygen-style documentation to public APIs in `crc16.hpp`, `drivelink_comm_registry.hpp`, `drivelink_comm.hpp`, `drivelink_system.hpp`, and `drivelink_imu_sensor.hpp`, including `@brief`, `@param`, and `@return` metadata where applicable.
- Completed checklist item 3: Replaced monolithic `src/drivelink_comm.cpp` with `src/drivelink_comm_io.cpp` and `src/drivelink_comm_rx.cpp`. All previous `std::cerr` logging paths in comm/registry code now use `RCLCPP_ERROR/WARN/DEBUG`.
- Completed checklist item 4: Replaced `src/drivelink_system.cpp` with `src/drivelink_system_lifecycle.cpp` and `src/drivelink_system_runtime.cpp`. Moved mode-switch timeout and setpoint epsilon literals into `constants.hpp`.
- Completed checklist item 5: Replaced `src/drivelink_imu_sensor.cpp` with `src/drivelink_imu_sensor_lifecycle.cpp` and `src/drivelink_imu_sensor_parser.cpp`. Moved quaternion scale literal into `constants.hpp`.
- Completed checklist item 6: Updated `CMakeLists.txt` to build split source files and gate test executable creation/installation with `BUILD_TESTING`.
- Completed checklist item 7: Aligned CMake dependency policy to match `package.xml` by making `controller_manager` required and exported unconditionally.
- Completed checklist item 8: Confirmed `drivelink_hardware.xml` class IDs/types still match plugin exports in split source files.
- Completed checklist item 9: Verified with `colcon build --packages-select drivelink_hardware` (success). `colcon test` reported no registered tests; standalone test executables were run manually and all reported PASS.

## Decisions & Rationale

- Kept `drivelink_messages.hpp` as a compatibility umbrella to avoid breaking existing include sites while enabling protocol modularization.
- Introduced `constants.hpp` for shared timing/epsilon/default values to reduce magic numbers and keep runtime behavior tuning centralized.
- Preserved pluginlib class names and XML identifiers to avoid downstream launch/config regressions.
- Switched comm and registry diagnostics to ROS logging for consistent runtime observability through ROS2 tooling.
- Maintained non-blocking `read()` semantics (try-lock pattern) and preserved write-path behavior to avoid changing control-loop semantics during structural refactor.

## Summary

### Files Created / Deleted / Modified
- Created:
	- `REFACTOR_LOG.md`
	- `include/drivelink_hardware/constants.hpp`
	- `include/drivelink_hardware/protocol/frame.hpp`
	- `include/drivelink_hardware/protocol/motor_messages.hpp`
	- `include/drivelink_hardware/protocol/imu_messages.hpp`
	- `include/drivelink_hardware/protocol/time_sync_messages.hpp`
	- `src/drivelink_comm_io.cpp`
	- `src/drivelink_comm_rx.cpp`
	- `src/drivelink_system_lifecycle.cpp`
	- `src/drivelink_system_runtime.cpp`
	- `src/drivelink_imu_sensor_lifecycle.cpp`
	- `src/drivelink_imu_sensor_parser.cpp`
- Deleted:
	- `src/drivelink_comm.cpp`
	- `src/drivelink_system.cpp`
	- `src/drivelink_imu_sensor.cpp`
- Modified:
	- `CMakeLists.txt`
	- `include/drivelink_hardware/crc16.hpp`
	- `include/drivelink_hardware/drivelink_comm.hpp`
	- `include/drivelink_hardware/drivelink_comm_registry.hpp`
	- `include/drivelink_hardware/drivelink_imu_sensor.hpp`
	- `include/drivelink_hardware/drivelink_messages.hpp`
	- `include/drivelink_hardware/drivelink_system.hpp`
	- `src/drivelink_comm_registry.cpp`

### Key Architectural Decisions
- Split large implementation files by responsibility (I/O vs RX parser; lifecycle vs runtime paths) while preserving existing plugin public contracts.
- Introduced a dedicated constants layer to replace magic numbers in control and parser paths.
- Preserved `drivelink_messages.hpp` as an umbrella compatibility header while physically modularizing protocol definitions.
- Standardized low-level transport/registry diagnostics on ROS logging macros for consistent runtime introspection.

### Manual Hardware Validation Remaining
- Validate end-to-end behavior on real UART hardware (`/dev/ttyACM*` or target UART bridge) under sustained command load to confirm no control jitter regressions.
- Verify activation/deactivation and shutdown safety behavior on real motors (including stop-command effect and firmware ACK timing).
- Confirm IMU scaling factors (`imu_gyro_scale`, `imu_accel_scale`) against known reference motion/calibration data.
- Optionally register standalone test executables with CTest/colcon for automated CI visibility (currently manually runnable binaries).
