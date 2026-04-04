# DriveLink ros2_control Handoff Plan

This folder is a complete planning handoff for implementing a host-side ros2_control integration for DriveLink ESP-IDF firmware.

Audience: implementation agent/engineer with no direct access to this firmware repository.

## 1. Reading Order

1. `01_firmware_api_reference.md`
- Ground truth for wire protocol, message IDs, payload layouts, and firmware constraints.

2. `02_ros2_control_framework_notes.md`
- ros2_control API contracts, lifecycle/mode-switch semantics, controller expectations, pluginlib/package patterns, and distro compatibility notes.

3. `03_comm_layer_design.md`
- transport/protocol client architecture and thread model.

4. `04_hardware_interface_design.md`
- plugin-level design for `SystemInterface` + `SensorInterface` and shared transport ownership.

5. `05_package_and_build_structure.md`
- package tree, CMake/package.xml/plugin XML build wiring.

6. `06_implementation_tasks.md`
- actionable phased backlog with acceptance criteria.

## 2. Core Decisions Already Made

1. Use one shared UART protocol client across both hardware plugins.
2. Implement two plugins:
- system plugin for wheel/motor control
- sensor plugin for IMU state interfaces
3. Keep serial I/O off realtime control loop threads.
4. Treat distro lock (Humble vs Jazzy/Rolling) as a first-class gating decision.

## 3. Critical Firmware Facts to Preserve

1. Frame format and CRC behavior must match firmware exactly.
2. Current message IDs are from active `sdkconfig` and should not be assumed defaults.
3. Time sync direction is ESP request -> host response.
4. IMU stream uses bitmask-driven optional fields and may include raw units requiring host conversion.

## 4. Critical ros2_control Facts to Preserve

1. `prepare_command_mode_switch` is non-realtime; `perform_command_mode_switch` participates in realtime switch path and must be fast.
2. `diff_drive_controller` expects wheel `velocity` command interfaces and position/velocity feedback depending on config.
3. IMU semantic interface names must match expected canonical strings exactly.
4. Plugin XML + CMake export wiring is required for discovery by pluginlib.

## 5. Immediate Next Actions for Implementation Agent

1. Lock target ROS 2 distro and API style.
2. Scaffold package from `05_package_and_build_structure.md`.
3. Build comm layer per `03_comm_layer_design.md` and validate with simulator.
4. Implement system plugin then IMU plugin per `04_hardware_interface_design.md`.
5. Execute backlog in `06_implementation_tasks.md` and satisfy Definition of Done.

## 6. Open Verification Items

Items marked `⚠ VERIFY` in phase documents are intentional checkpoints that must be resolved during implementation/testing.

## 7. Non-Goals Reminder

This handoff intentionally avoids firmware modification. It is host-side integration planning and execution guidance only.
