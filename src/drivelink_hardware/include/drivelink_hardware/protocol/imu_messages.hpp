#pragma once

#include <cstdint>

namespace drivelink_hardware
{

enum class ImuConfigureSubcommand : uint8_t
{
  kSetStreamEnabled = 0x01,
  kSetStreamPeriodMs = 0x02,
  kSetStreamFieldMask = 0x03,
  kSetOperationMode = 0x04,
  kReadCalibration = 0x05,
  kWriteCalibration = 0x06,
  kApplyCalibration = 0x07,
};

enum class ImuResultCode : uint8_t
{
  kOk = 0x00,
  kInvalidArgument = 0x01,
  kInvalidState = 0x02,
  kNotSupported = 0x03,
  kBusy = 0x04,
  kInternalError = 0x05,
};

enum class ImuStreamFieldMask : uint16_t
{
  kEuler = 1U << 0,
  kQuaternion = 1U << 1,
  kLinearAccel = 1U << 2,
  kGravity = 1U << 3,
  kRawAccel = 1U << 4,
  kRawGyro = 1U << 5,
  kRawMag = 1U << 6,
  kTemperature = 1U << 7,
  kCalibrationStatus = 1U << 8,
};

struct __attribute__((packed)) ImuConfigureHeader
{
  uint8_t version;
  uint8_t subcommand;
};
static_assert(sizeof(ImuConfigureHeader) == 2, "ImuConfigureHeader size mismatch");

struct __attribute__((packed)) ImuSetStreamEnabledBody
{
  uint8_t enabled;
};
static_assert(sizeof(ImuSetStreamEnabledBody) == 1, "ImuSetStreamEnabledBody size mismatch");

struct __attribute__((packed)) ImuSetStreamPeriodBody
{
  uint16_t period_ms;
};
static_assert(sizeof(ImuSetStreamPeriodBody) == 2, "ImuSetStreamPeriodBody size mismatch");

struct __attribute__((packed)) ImuSetStreamFieldMaskBody
{
  uint16_t field_mask;
};
static_assert(sizeof(ImuSetStreamFieldMaskBody) == 2, "ImuSetStreamFieldMaskBody size mismatch");

struct __attribute__((packed)) ImuSetOperationModeBody
{
  uint8_t mode;
};
static_assert(sizeof(ImuSetOperationModeBody) == 1, "ImuSetOperationModeBody size mismatch");

struct __attribute__((packed)) ImuCalibrationBlob
{
  uint8_t has_sic;
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
  int16_t accel_radius;
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;
  int16_t mag_offset_x;
  int16_t mag_offset_y;
  int16_t mag_offset_z;
  int16_t mag_radius;
  int16_t sic[9];
};
static_assert(sizeof(ImuCalibrationBlob) == 41, "ImuCalibrationBlob size mismatch");

struct __attribute__((packed)) ImuConfigureResponse
{
  uint8_t version;
  uint8_t subcommand_echo;
  uint8_t result_code;
};
static_assert(sizeof(ImuConfigureResponse) == 3, "ImuConfigureResponse size mismatch");

struct __attribute__((packed)) ImuTelemetryHeader
{
  uint8_t version;
  uint16_t field_mask;
  uint64_t timestamp_us;
};
static_assert(sizeof(ImuTelemetryHeader) == 11, "ImuTelemetryHeader size mismatch");

}  // namespace drivelink_hardware
