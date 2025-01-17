#ifndef BOT_SERIAL__CHASSIS_DEVICE_HPP_
#define BOT_SERIAL__CHASSIS_DEVICE_HPP_

#include <sstream>  // 确保包含这个头文件
#include <iomanip>  // 用于设置输出格式
#include "bot_serial/device_unit.hpp"
#include "spdlog/spdlog.h"
#include <fmt/ranges.h>

namespace bot_serial
{
class ChassisDevice : public DeviceUnit
{
public:
    void process_data(const std::string &data) override;
    void enable_device() override;
    void disable_device() override;
    bool get_motor_status();

private:
    std::string buffer_; // 用于保存未处理的数据

    std::vector<std::string> extract_frames(); // 从缓冲区提取完整帧
    void parse_frame(const std::string &frame);
    uint16_t calculate_crc16(const std::vector<uint8_t> &data);
    void process_frame_data(uint8_t frame_type, const std::vector<uint8_t> &data);

    bool motor_status_{false};
    int16_t left_motor_speed_;
    int16_t right_motor_speed_;
    int32_t left_motor_encoder_;
    int32_t right_motor_encoder_;


};
}  // namespace bot_serial

#endif  // BOT_SERIAL__CHASSIS_DEVICE_HPP_
