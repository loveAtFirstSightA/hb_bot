#include "bot_serial/chassis_device.hpp"

namespace bot_serial 
{

// 处理接收到的数据
void ChassisDevice::process_data(const std::string &data) 
{
    // 将新接收的数据追加到缓冲区
    buffer_ += data;

    // 提取完整帧
    auto frames = extract_frames();

    // 逐帧处理
    for (const auto &frame : frames) 
    {
        // // 打印完整帧内容（十六进制显示）
        // std::ostringstream hex_stream;
        // hex_stream << "Complete frame: ";
        // for (unsigned char c : frame) 
        // {
        //     hex_stream << std::hex << std::uppercase << std::setw(2) 
        //                << std::setfill('0') << static_cast<int>(c) << " ";
        // }
        // spdlog::info("{}", hex_stream.str());

        // 解析帧内容
        parse_frame(frame);
    }
}

// 从缓冲区中提取完整帧
std::vector<std::string> ChassisDevice::extract_frames() 
{
    std::vector<std::string> frames;
    size_t pos = 0;

    while (true) 
    {
        // 查找帧头 (AA 55)
        size_t start = buffer_.find("\xAA\x55", pos);
        if (start == std::string::npos) 
        {
            // 如果找不到帧头，停止解析
            break;
        }

        // 检查是否包含最小帧长度 (帧头 + 帧类型 + 长度 + CRC)
        if (buffer_.size() < start + 6) 
        {
            pos = start;
            break;
        }

        // 获取数据长度
        uint8_t data_length = static_cast<uint8_t>(buffer_[start + 3]);

        // 计算完整帧的总长度
        size_t frame_length = 4 + data_length + 2; // 帧头2字节 + 类型1字节 + 长度1字节 + 数据长度 + CRC2字节

        // 检查缓冲区是否包含完整帧
        if (buffer_.size() < start + frame_length) 
        {
            pos = start;
            break;
        }

        // 提取完整帧
        std::string frame = buffer_.substr(start, frame_length);
        frames.push_back(frame);

        // 更新搜索位置
        pos = start + frame_length;
    }

    // 清理已处理的数据
    if (pos > 0) 
    {
        buffer_ = buffer_.substr(pos);
    }

    return frames;
}

// 解析帧内容
void ChassisDevice::parse_frame(const std::string &frame) 
{
    // 检查帧长度是否足够
    if (frame.size() < 6) 
    {
        spdlog::warn("Frame too short to parse: {}", frame.size());
        return;
    }

    // 验证帧头是否正确
    if (static_cast<uint8_t>(frame[0]) != 0xAA || static_cast<uint8_t>(frame[1]) != 0x55) 
    {
        spdlog::warn("Invalid frame header. Expected AA 55, got {:02X} {:02X}", 
                     static_cast<uint8_t>(frame[0]), static_cast<uint8_t>(frame[1]));
        return;
    }

    // 提取帧类型和数据长度
    uint8_t frame_type = static_cast<uint8_t>(frame[2]);
    uint8_t data_length = static_cast<uint8_t>(frame[3]);

    // 检查帧长度是否与数据长度匹配
    size_t expected_size = 4 + data_length + 2; // 帧头 + 帧类型 + 数据长度 + 数据 + CRC
    if (frame.size() < expected_size) 
    {
        spdlog::warn("Frame size mismatch. Expected at least {} bytes, but got {}", expected_size, frame.size());
        return;
    }

    // 提取有效数据
    std::vector<uint8_t> data(frame.begin() + 4, frame.begin() + 4 + data_length);

    // 提取接收到的 CRC
    uint16_t received_crc = (static_cast<uint8_t>(frame[4 + data_length]) << 8) |
                            static_cast<uint8_t>(frame[5 + data_length]);

    // 计算 CRC 数据类型和数据长度和数据
    std::vector<uint8_t> frame_for_crc(frame.begin() + 2, frame.begin() + 4 + data_length); // 从帧类型开始，不包含 CRC 本身 
    uint16_t calculated_crc = calculate_crc16(frame_for_crc);

    // 验证 CRC 是否匹配
    if (calculated_crc != received_crc) 
    {
        spdlog::error("CRC check failed! Received: 0x{:04X}, Calculated: 0x{:04X}", received_crc, calculated_crc);
        return;
    }

    // spdlog::info("CRC check passed. Frame Type: 0x{:02X}, Data Length: {}", frame_type, data_length);

    // 根据帧类型处理数据
    process_frame_data(frame_type, data_length, data);
}

// 根据帧类型处理帧数据
void ChassisDevice::process_frame_data(const uint8_t frame_type, const uint8_t data_length, const std::vector<uint8_t> &data)
{
    switch (frame_type) 
    {
        // 心跳帧
        case 0xA0:
        {
            if (data.size() == data_length) 
            {
                uint8_t heartbeat_status = data[0];
                // spdlog::info("Heartbeat frame received, status: 0x{:02X}", heartbeat_status);
                if (heartbeat_status == 0 || heartbeat_status == 1)
                {
                    // 记录心跳发生的时间戳
                    this->last_heartbeat_timestamp_ = std::chrono::high_resolution_clock::now();
                }
            }
            else 
            {
                spdlog::warn("Invalid data length for Heartbeat frame");
            }
        } break;
        // 电机状态
        case 0x01:
        {
            if (data.size() == data_length)
            {
                if (data[0] == 0)
                {
                    this->motor_status_ = false;
                    // spdlog::info("Motor status is disable");
                }
                else if (data[0] == 1)
                {
                    this->motor_status_ = true;
                    // spdlog::info("Motor status is enable");
                }
                else
                {
                    spdlog::warn("Motor status is invalid");
                }
            }
            else
            {
                spdlog::warn("Invalid data length for Motor status frame");
            }
        } break;
        // 电机轮速编码器数据
        case 0x21:
        {
            if (data.size() == data_length)
            {
                // spdlog::info("encoder, data efficient {}", data[0]);
                this->left_motor_speed_ = data[1];
                this->right_motor_speed_ = data[2];
                this->left_motor_encoder_ = data[3];
                this->right_motor_encoder_ = data[4];
            }
            else
            {
                spdlog::warn("Invalid data length for Motor value speed and encoder frame");
            }
        } break;
        
        case 0x22:  // battery
        {
            spdlog::info("battery status");
        } break;
        
        case 0x23:  // imu data
        {

        } break;

        case 0xB1: // 示例帧类型
            spdlog::info("Frame Type 0xB1 received. Data: {}", fmt::join(data, " "));
            break;

        default:
            // spdlog::warn("Unknown frame type: 0x{:02X}", frame_type);
            break;
    }
}

std::chrono::high_resolution_clock::time_point ChassisDevice::get_last_heartbeat_stamp()
{
    return this->last_heartbeat_timestamp_;
}

bool ChassisDevice::get_motor_status()
{
    return this->motor_status_;
}

int16_t ChassisDevice::get_left_motor_speed()
{
    return this->left_motor_speed_;
}

int16_t ChassisDevice::get_right_motor_speed()
{
    return this->right_motor_speed_;
}

int32_t ChassisDevice::get_left_motor_encoder()
{
    return left_motor_encoder_;
}

int32_t ChassisDevice::get_right_motor_encoder()
{
    return this->right_motor_encoder_;
}





} // namespace bot_serial
