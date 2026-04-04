#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "drivelink_hardware/crc16.hpp"

namespace
{
uint16_t crc16_reference_bitwise(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (size_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return static_cast<uint16_t>(crc);
}

void print_result(const std::string & label, bool pass)
{
  std::cout << label << ": " << (pass ? "PASS" : "FAIL") << std::endl;
}
}  // namespace

int main()
{
  bool all_pass = true;

  const std::vector<uint8_t> seq1 = {
    0x21, 0x05, 0x01, 0x06, 0x00, 0x00, 0x80, 0x3f
  };

  const auto seq1_crc_table = drivelink_hardware::crc16(seq1.data(), seq1.size());
  const auto seq1_crc_ref = crc16_reference_bitwise(seq1.data(), seq1.size());
  const bool seq1_ok = (seq1_crc_table == seq1_crc_ref);
  print_result("CRC known-sequence self-check", seq1_ok);
  std::cout << "Reference vector bytes: 21 05 01 06 00 00 80 3f, crc=0x"
            << std::hex << std::setw(4) << std::setfill('0') << seq1_crc_table << std::dec
            << std::endl;
  all_pass = all_pass && seq1_ok;

  const std::array<uint8_t, 0> empty{};
  const bool empty_ok = (drivelink_hardware::crc16(empty.data(), empty.size()) == 0xFFFFU);
  print_result("CRC empty payload", empty_ok);
  all_pass = all_pass && empty_ok;

  const uint8_t one_byte[] = {0xAA};
  const bool one_byte_ok =
    (drivelink_hardware::crc16(one_byte, 1) == crc16_reference_bitwise(one_byte, 1));
  print_result("CRC single-byte", one_byte_ok);
  all_pass = all_pass && one_byte_ok;

  std::cout << (all_pass ? "PASS" : "FAIL") << std::endl;
  return all_pass ? 0 : 1;
}
