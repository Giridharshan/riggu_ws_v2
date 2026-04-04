#pragma once

#include <cstdint>

namespace drivelink_hardware
{

enum class MessageType : uint8_t
{
  kTimeSyncRequest = 0x20,
  kTimeSyncResponse = 0x60,
  kMotorConfigureCommand = 0x21,
  kMotorConfigureResponse = 0x61,
  kImuConfigureCommand = 0x22,
  kImuConfigureResponse = 0x42,
  kMotorSetpointStream = 0x80,
  kMotorTelemetryStream = 0x81,
  kImuTelemetryStream = 0x82,
  kImuEvent = 0xC2,
};

enum class MotorConfigureSubcommand : uint8_t
{
  kCreateMotor = 0x01,
  kDeleteMotor = 0x02,
  kConfigureMotor = 0x03,
  kSetMode = 0x04,
  kSetPidParams = 0x05,
  kSetSetpoint = 0x06,
  kStopMotor = 0x07,
  kResetMotor = 0x08,
  kSetTelemetryGlobal = 0x09,
  kSetTelemetryMotor = 0x0A,
};

enum class MotorControlMode : uint8_t
{
  kOpenLoop = 0,
  kVelocity = 1,
  kPosition = 2,
};

enum class MotorPidKind : uint8_t
{
  kVelocity = 0,
  kPosition = 1,
};

enum class MotorSetpointKind : uint8_t
{
  kDuty = 0,
  kVelocityRadS = 1,
  kPositionRad = 2,
};

enum class MotorResultCode : uint8_t
{
  kOk = 0x00,
  kInvalidArgument = 0x01,
  kNotFound = 0x02,
  kAlreadyExists = 0x03,
  kCapacityReached = 0x04,
  kInvalidState = 0x05,
  kNotSupported = 0x06,
  kInternalError = 0x07,
};

enum class MotorTelemetryFieldMask : uint8_t
{
  kAngle = 0x01,
  kVelocity = 0x02,
};

struct __attribute__((packed)) MotorConfigureHeader
{
  uint8_t version;
  uint8_t subcommand;
  uint8_t motor_id;
};
static_assert(sizeof(MotorConfigureHeader) == 3, "MotorConfigureHeader size mismatch");

struct __attribute__((packed)) MotorCreateOrConfigureBody
{
  uint8_t pwm_gpio;
  uint8_t dir_gpio;
  uint8_t enc_a;
  uint8_t enc_b;
  float counts_per_rev;
  float gear_ratio;
  float output_min;
  float output_max;
  float vel_kp;
  float vel_ki;
  float vel_kd;
  float pos_kp;
  float pos_ki;
  float pos_kd;
  float velocity_feedforward;
  float position_feedforward;
  float reserved0;
  float reserved1;
  uint32_t sample_time_ms;
};
static_assert(sizeof(MotorCreateOrConfigureBody) == 64, "MotorCreateOrConfigureBody size mismatch");

struct __attribute__((packed)) MotorSetModeBody
{
  uint8_t mode;
};
static_assert(sizeof(MotorSetModeBody) == 1, "MotorSetModeBody size mismatch");

struct __attribute__((packed)) MotorSetPidParamsBody
{
  uint8_t pid_kind;
  float kp;
  float ki;
  float kd;
  uint32_t sample_time_ms;
};
static_assert(sizeof(MotorSetPidParamsBody) == 17, "MotorSetPidParamsBody size mismatch");

struct __attribute__((packed)) MotorSetpointBody
{
  uint8_t setpoint_kind;
  float value;
};
static_assert(sizeof(MotorSetpointBody) == 5, "MotorSetpointBody size mismatch");

struct __attribute__((packed)) MotorSetTelemetryGlobalBody
{
  uint8_t enabled;
  uint8_t field_mask;
};
static_assert(sizeof(MotorSetTelemetryGlobalBody) == 2, "MotorSetTelemetryGlobalBody size mismatch");

struct __attribute__((packed)) MotorSetTelemetryMotorBody
{
  uint8_t enabled;
  uint8_t field_mask;
  uint8_t update_field_mask;
};
static_assert(sizeof(MotorSetTelemetryMotorBody) == 3, "MotorSetTelemetryMotorBody size mismatch");

struct __attribute__((packed)) MotorConfigureResponse
{
  uint8_t version;
  uint8_t subcommand_echo;
  uint8_t motor_id;
  uint8_t result_code;
  uint8_t reserved;
};
static_assert(sizeof(MotorConfigureResponse) == 5, "MotorConfigureResponse size mismatch");

struct __attribute__((packed)) MotorSetpointEntry
{
  uint8_t motor_id;
  uint8_t setpoint_kind;
  float value;
};
static_assert(sizeof(MotorSetpointEntry) == 6, "MotorSetpointEntry size mismatch");

struct __attribute__((packed)) MotorSetpointCmd
{
  uint8_t version;
  uint8_t entry_count;
  MotorSetpointEntry entry;
};
static_assert(sizeof(MotorSetpointCmd) == 8, "MotorSetpointCmd size mismatch");

struct __attribute__((packed)) MotorTelemetryHeader
{
  uint8_t version;
  uint8_t entry_count;
  uint64_t timestamp_us;
};
static_assert(sizeof(MotorTelemetryHeader) == 10, "MotorTelemetryHeader size mismatch");

struct __attribute__((packed)) MotorTelemetryEntryHeader
{
  uint8_t motor_id;
  uint8_t field_mask;
};
static_assert(sizeof(MotorTelemetryEntryHeader) == 2, "MotorTelemetryEntryHeader size mismatch");

struct __attribute__((packed)) MotorStateTelemetry
{
  uint8_t version;
  uint8_t entry_count;
  uint64_t timestamp_us;
  uint8_t motor_id;
  uint8_t field_mask;
  float angle_rad;
  float angular_velocity_rad_s;
};
static_assert(sizeof(MotorStateTelemetry) == 20, "MotorStateTelemetry size mismatch");

}  // namespace drivelink_hardware
