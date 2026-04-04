#include "drivelink_hardware/drivelink_comm_registry.hpp"

#include <iostream>

namespace drivelink_hardware
{

std::mutex DriveLinkCommRegistry::mutex_;
std::weak_ptr<DriveLinkComm> DriveLinkCommRegistry::shared_comm_;
std::string DriveLinkCommRegistry::shared_port_;
int DriveLinkCommRegistry::shared_baudrate_ = 0;
size_t DriveLinkCommRegistry::ref_count_ = 0;

std::shared_ptr<DriveLinkComm> DriveLinkCommRegistry::acquire(const std::string & port, int baudrate)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto existing = shared_comm_.lock();
  if (existing) {
    if (shared_port_ != port || shared_baudrate_ != baudrate) {
      std::cerr << "DriveLinkCommRegistry: parameter mismatch for shared UART instance. "
                << "existing(" << shared_port_ << ", " << shared_baudrate_ << ") requested(" << port
                << ", " << baudrate << ")" << std::endl;
      return nullptr;
    }
    ++ref_count_;
    return existing;
  }

  auto created = std::make_shared<DriveLinkComm>(port, baudrate);
  shared_comm_ = created;
  shared_port_ = port;
  shared_baudrate_ = baudrate;
  ref_count_ = 1;
  return created;
}

void DriveLinkCommRegistry::release(const std::shared_ptr<DriveLinkComm> & comm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (ref_count_ == 0) {
    return;
  }

  --ref_count_;
  if (ref_count_ > 0) {
    return;
  }

  auto existing = shared_comm_.lock();
  if (existing && existing == comm) {
    existing->close();
  }
  shared_comm_.reset();
  shared_port_.clear();
  shared_baudrate_ = 0;
}

}  // namespace drivelink_hardware
