#pragma once

#include <cstdint>

namespace drivelink_hardware
{

inline constexpr uint8_t kDriveLinkStartByte = static_cast<uint8_t>(0xAA);

struct __attribute__((packed)) FrameEnvelopeHeader
{
  uint8_t start0;
  uint8_t start1;
  uint8_t start2;
  uint8_t msg_type;
  uint8_t payload_len;
};
static_assert(sizeof(FrameEnvelopeHeader) == 5, "FrameEnvelopeHeader size mismatch");

}  // namespace drivelink_hardware
