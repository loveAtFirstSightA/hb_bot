#ifndef BOT_SERIAL__CHASSIS_DEVICE_HPP_
#define BOT_SERIAL__CHASSIS_DEVICE_HPP_

#include <chrono>
#include <sstream>  // 确保包含这个头文件
#include <iomanip>  // 用于设置输出格式
#include "bot_serial/device_unit.hpp"
#include "spdlog/spdlog.h"
#include <fmt/ranges.h>
#include <vector>
#include <string>
#include <cstdint>
#include "bot_serial/common.hpp"

namespace bot_serial
{
class ChassisDevice : public DeviceUnit
{
public:
    void process_data(const std::string &data) override;
    std::chrono::high_resolution_clock::time_point get_last_heartbeat_stamp();
    bool get_motor_status();
    int16_t get_left_motor_speed();
    int16_t get_right_motor_speed();
    int32_t get_left_motor_encoder();
    int32_t get_right_motor_encoder();

private:
    std::string buffer_; // 用于保存未处理的数据

    std::vector<std::string> extract_frames(); // 从缓冲区提取完整帧
    void parse_frame(const std::string &frame);
    void process_frame_data(const uint8_t frame_type, const uint8_t data_length, const std::vector<uint8_t> &data);

    std::chrono::high_resolution_clock::time_point last_heartbeat_timestamp_;
    bool motor_status_{false};
    int16_t left_motor_speed_;
    int16_t right_motor_speed_;
    int32_t left_motor_encoder_;
    int32_t right_motor_encoder_;
    


};
}  // namespace bot_serial

#endif  // BOT_SERIAL__CHASSIS_DEVICE_HPP_
