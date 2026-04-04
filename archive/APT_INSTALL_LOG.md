# APT Install Log

Date: 2026-04-04

Packages installed in this workspace environment:

- ros-jazzy-controller-manager
- ros-jazzy-joint-state-broadcaster
- ros-jazzy-imu-sensor-broadcaster
- ros-jazzy-forward-command-controller
- ros-jazzy-ros2controlcli
- socat

Reason:

- Required runtime components for ros2_control launch/integration validation.
- `socat` required to create virtual PTY pairs for UART loopback with `tools/mock_firmware.py`.

Notes:

- `ros-jazzy-joint-state-broadcaster` was already present but retained as an explicit dependency in the install command.

## Additional install event

Date: 2026-04-04 (later session)

- Installed: `socat`, `libwrap0`
- Verified already present: `ros-jazzy-controller-manager`, `ros-jazzy-ros2controlcli`

Reason:

- Environment reset removed `socat`; reinstalled to run PTY loopback integration checks.
