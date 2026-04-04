#pragma once

#include <cstdint>

namespace drivelink_hardware
{

struct __attribute__((packed)) TimeSyncRequestPayload
{
  uint8_t version;
  uint8_t flags;
  uint16_t request_id;
  int64_t client_unix_send_us;
};
static_assert(sizeof(TimeSyncRequestPayload) == 12, "TimeSyncRequestPayload size mismatch");

struct __attribute__((packed)) TimeSyncResponsePayload
{
  uint8_t version;
  uint8_t status;
  uint16_t request_id;
  int64_t echoed_client_unix_send_us;
  int64_t server_unix_rx_us;
  int64_t server_unix_tx_us;
  uint8_t reserved0;
  uint8_t reserved1;
};
static_assert(sizeof(TimeSyncResponsePayload) == 30, "TimeSyncResponsePayload size mismatch");

}  // namespace drivelink_hardware
