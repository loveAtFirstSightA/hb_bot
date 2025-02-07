#include "bot_serial/common.hpp"  // 包含声明的头文件

namespace bot_serial
{
// 计算 CRC16 校验码（定义）
uint16_t calculate_crc16(const std::vector<uint8_t>& data) 
{
    // 初始化 CRC 校验码为 0xFFFF（常见的初始值）
    uint16_t crc = 0xFFFF;

    // 遍历输入的数据，每次取出一个字节
    for (uint8_t byte : data) 
    {
        // CRC 校验码与当前字节进行异或操作
        crc ^= byte;

        // 对每一个字节进行 8 次移位运算（共 8 位）
        for (int i = 0; i < 8; ++i) 
        {
            // 如果 CRC 校验码的最低位是 1，执行 XOR 操作
            if (crc & 0x0001) 
            {
                crc >>= 1;          // 将 CRC 校验码右移一位
                crc ^= 0xA001;      // 将 CRC 校验码与常数 0xA001 进行异或操作
            } 
            else 
            {
                crc >>= 1;          // 如果最低位是 0，只进行右移操作
            }
        }
    }
    
    // 返回最终计算出的 CRC 校验码
    return crc;
}

}
