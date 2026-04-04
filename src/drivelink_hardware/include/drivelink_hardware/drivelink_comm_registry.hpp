#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "drivelink_hardware/drivelink_comm.hpp"

namespace drivelink_hardware
{

/**
 * @brief Process-local registry that shares one DriveLinkComm instance per UART endpoint.
 *
 * This helper ensures system and sensor plugins in the same process reuse the same serial
 * transport when they are configured with identical `port` and `baudrate` parameters.
 */
class DriveLinkCommRegistry
{
public:
  /**
   * @brief Acquire a shared communication object for the given UART endpoint.
   * @param port UART device path.
   * @param baudrate UART baud rate.
   * @return Shared DriveLinkComm instance, or nullptr on endpoint mismatch.
   */
  static std::shared_ptr<DriveLinkComm> acquire(const std::string & port, int baudrate);

  /**
   * @brief Release a previously acquired shared communication object.
   * @param comm Shared DriveLinkComm instance obtained from acquire().
   * @return None.
   */
  static void release(const std::shared_ptr<DriveLinkComm> & comm);

private:
  static std::mutex mutex_;
  static std::weak_ptr<DriveLinkComm> shared_comm_;
  static std::string shared_port_;
  static int shared_baudrate_;
  static size_t ref_count_;
};

}  // namespace drivelink_hardware
