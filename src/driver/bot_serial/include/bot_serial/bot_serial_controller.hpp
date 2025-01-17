#ifndef BOT_SERIAL__BOT_SERIAL_CONTROLLER_HPP_
#define BOT_SERIAL__BOT_SERIAL_CONTROLLER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "bot_serial/serial_port_manager.hpp"
#include "bot_serial/chassis_device.hpp"
#include "bot_serial/action_unit_device.hpp"
#include "sensor_msgs/msg/imu.hpp"      // IMU消息类型
#include "nav_msgs/msg/odometry.hpp"    // 里程计消息类型

namespace bot_serial
{
class BotSerialController : public rclcpp::Node
{
public:
    BotSerialController();
    ~BotSerialController();

private:
    void setup_devices();

    std::shared_ptr<SerialPortManager> chassis_manager_;
    std::shared_ptr<SerialPortManager> action_unit_manager_;
    std::shared_ptr<DeviceUnit> chassis_device_;
    std::shared_ptr<DeviceUnit> action_unit_device_;

    // Device parameters
    std::string chassis_port_;
    int chassis_baud_rate_;
    bool chassis_enabled_;

    std::string action_unit_port_;
    int action_unit_baud_rate_;
    bool action_unit_enabled_;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    // 定时器
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    // 数据（这里只是示例，实际数据应来自设备）
    sensor_msgs::msg::Imu imu_data_;
    nav_msgs::msg::Odometry odom_data_;

    void publish_imu_data();
    void publish_odom_data();

    void control_motor_status(bool status);


};
}  // namespace bot_serial

#endif  // BOT_SERIAL__BOT_SERIAL_CONTROLLER_HPP_
