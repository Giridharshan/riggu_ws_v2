#pragma once

#include <chrono>
#include <cstdint>

namespace drivelink_hardware
{
namespace constants
{

inline constexpr int kDefaultUartBaud = 115200;
inline constexpr int kDefaultTelemetryPeriodMs = 100;
inline constexpr int kDefaultImuStreamPeriodMs = 10;

inline constexpr size_t kWriteRetryLimit = 3;
inline constexpr int kReaderPollTimeoutMs = 50;
inline constexpr std::chrono::seconds kReaderShutdownTimeout{2};
inline constexpr std::chrono::milliseconds kModeSwitchResponseTimeout{500};

inline constexpr double kVelocitySetpointEpsilon = 1e-6;
inline constexpr double kPositionSetpointEpsilon = 1e-6;
inline constexpr double kEffortSetpointEpsilon = 1e-4;

inline constexpr double kImuQuaternionScale = 1.0 / 16384.0;

}  // namespace constants
}  // namespace drivelink_hardware
