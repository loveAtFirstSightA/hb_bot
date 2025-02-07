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
#include "tf2/utils.hpp"
#include "bot_serial/common.hpp"
#include "std_msgs/msg/string.hpp"

namespace bot_serial
{
class BotSerialController : public rclcpp::Node
{
public:
    BotSerialController();
    ~BotSerialController();

private:
    void setup_devices();
    void vel_subscriber_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void control_cell_subscriber_callback(const std_msgs::msg::String::SharedPtr msg);

    std::shared_ptr<SerialPortManager> chassis_manager_;
    std::shared_ptr<SerialPortManager> action_unit_manager_;
    std::shared_ptr<DeviceUnit> chassis_device_;
    std::shared_ptr<DeviceUnit> action_unit_device_;
    std::shared_ptr<ChassisDevice> chassis_;
    std::shared_ptr<ActionUnitDevice> action_;
    
    // Device parameters
    std::string chassis_port_;
    int chassis_baud_rate_;
    bool chassis_enabled_;

    std::string action_unit_port_;
    int action_unit_baud_rate_;
    bool action_unit_enabled_;

    // calculate odometry
    double wheel_diameter_;
    int encoder_pulse_;
    double wheel_base_;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_cell_subscriber_;

    // 定时器
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    // 数据（这里只是示例，实际数据应来自设备）
    sensor_msgs::msg::Imu imu_data_;
    nav_msgs::msg::Odometry odom_data_;

    void publish_imu_data();
    void publish_odom_data();

    bool is_first_run_;
    int last_right_encoder_;
    int last_left_encoder_;
    double x_position_;
    double y_position_;
    double theta_;
    double v_;
    double w_;
    void calculate_odometry(double& x, double& y, double& theta, double& v, double& w);

    // 使能电机
    void enable_motor();
    // 失能电机
    void disable_motor();
    // 关闭工控机电源
    void turn_off_calculate_power();
    // 控制灯带为红色 test pass
    void control_led_info_red();
    // 控制灯带为绿色 test pass
    void control_led_info_green();
    // 使能急停和防撞条
    void enable_emergency_switch();
    // 失能急停和防撞条
    void disable_emergency_switch();
    // 清空编码器
    void clear_motor_encoder_value();
    // 开始充电
    void start_recharge();
    // 关闭充电
    void close_recharge();
    // 发布电机转速
    void control_motor_speed(const double v, const double w);

    // test pass
    void open_cell_one();
    void close_cell_one();
    void open_cell_two();
    void close_cell_two();
    void open_cell_three();
    void close_cell_three();
    void open_cell_four();
    void close_cell_four();




};
}  // namespace bot_serial

#endif  // BOT_SERIAL__BOT_SERIAL_CONTROLLER_HPP_
