#ifndef BOT_SERIAL__COMMON_HPP_
#define BOT_SERIAL__COMMON_HPP_

#include <vector>
#include <cstdint>

namespace bot_serial
{
    // 计算 CRC16 校验码（声明）
    uint16_t calculate_crc16(const std::vector<uint8_t>& data);
}  // namespace bot_serial

#endif  // BOT_SERIAL__COMMON_HPP_
