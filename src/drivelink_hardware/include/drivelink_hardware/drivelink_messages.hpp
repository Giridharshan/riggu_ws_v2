#pragma once

#include <cstdint>

#include "drivelink_hardware/protocol/frame.hpp"
#include "drivelink_hardware/protocol/imu_messages.hpp"
#include "drivelink_hardware/protocol/motor_messages.hpp"
#include "drivelink_hardware/protocol/time_sync_messages.hpp"

inline constexpr uint8_t kDriveLinkStartByte = drivelink_hardware::kDriveLinkStartByte;

namespace drivelink_hardware
{

static_assert(sizeof(float) == 4, "float must be 4 bytes");

}  // namespace drivelink_hardware
