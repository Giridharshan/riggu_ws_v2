#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace drivelink_hardware
{
namespace detail
{
constexpr uint16_t kCrc16Polynomial = 0x1021;

constexpr uint16_t crc16_table_entry(uint16_t index)
{
  uint16_t crc = static_cast<uint16_t>(index << 8);
  for (size_t bit = 0; bit < 8; ++bit) {
    const bool msb_set = (crc & 0x8000U) != 0U;
    crc = static_cast<uint16_t>(crc << 1);
    if (msb_set) {
      crc = static_cast<uint16_t>(crc ^ kCrc16Polynomial);
    }
  }
  return crc;
}

constexpr std::array<uint16_t, 256> make_crc16_table()
{
  std::array<uint16_t, 256> table{};
  for (size_t i = 0; i < table.size(); ++i) {
    table[i] = crc16_table_entry(static_cast<uint16_t>(i));
  }
  return table;
}

constexpr auto kCrc16Table = make_crc16_table();
}  // namespace detail

inline uint16_t crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    const uint8_t index = static_cast<uint8_t>((crc >> 8) ^ data[i]);
    crc = static_cast<uint16_t>((crc << 8) ^ detail::kCrc16Table[index]);
  }
  return static_cast<uint16_t>(crc);
}

}  // namespace drivelink_hardware
