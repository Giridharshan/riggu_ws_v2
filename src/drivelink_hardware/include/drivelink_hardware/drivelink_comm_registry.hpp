#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "drivelink_hardware/drivelink_comm.hpp"

namespace drivelink_hardware
{

class DriveLinkCommRegistry
{
public:
  // Process-local shared comm object for one UART endpoint. The first acquire creates the
  // instance, each acquire increments ref-count, and the final release closes and destroys it.
  static std::shared_ptr<DriveLinkComm> acquire(const std::string & port, int baudrate);
  static void release(const std::shared_ptr<DriveLinkComm> & comm);

private:
  static std::mutex mutex_;
  static std::weak_ptr<DriveLinkComm> shared_comm_;
  static std::string shared_port_;
  static int shared_baudrate_;
  static size_t ref_count_;
};

}  // namespace drivelink_hardware
