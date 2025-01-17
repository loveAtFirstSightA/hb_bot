#include "bot_serial/bot_serial_controller.hpp"

namespace bot_serial
{

BotSerialController::BotSerialController() : Node("bot_serial_controller")
{
    spdlog::info("Initializing BotSerialController...");

    // Declare and get parameters for chassis
    this->declare_parameter<std::string>("chassis_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("chassis_baud_rate", 115200);
    this->declare_parameter<bool>("chassis_enabled", true);

    this->get_parameter("chassis_port", chassis_port_);
    this->get_parameter("chassis_baud_rate", chassis_baud_rate_);
    this->get_parameter("chassis_enabled", chassis_enabled_);

    // Declare and get parameters for action unit
    this->declare_parameter<std::string>("action_unit_port", "/dev/ttyUSB1");
    this->declare_parameter<int>("action_unit_baud_rate", 115200);
    this->declare_parameter<bool>("action_unit_enabled", true);

    this->get_parameter("action_unit_port", action_unit_port_);
    this->get_parameter("action_unit_baud_rate", action_unit_baud_rate_);
    this->get_parameter("action_unit_enabled", action_unit_enabled_);

    // Set up devices based on parameters
    setup_devices();

    // Initialize publishers for IMU and Odometry data
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Create timers for publishing data
    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // 100ms interval for IMU data
        std::bind(&BotSerialController::publish_imu_data, this));

    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50ms interval for Odometry data
        std::bind(&BotSerialController::publish_odom_data, this));

}

BotSerialController::~BotSerialController()
{
    spdlog::info("Shutting down BotSerialController...");
}

void BotSerialController::setup_devices()
{
    // Set up chassis device if enabled
    if (chassis_enabled_) {
        spdlog::info("Setting up chassis device on port '{}' with baud rate {}.", chassis_port_, chassis_baud_rate_);
        chassis_device_ = std::make_shared<ChassisDevice>();
        chassis_manager_ = std::make_shared<SerialPortManager>(chassis_port_, chassis_baud_rate_);
        chassis_manager_->start(chassis_device_);
    } else {
        spdlog::info("Chassis device is disabled.");
    }

    // Set up action unit device if enabled
    if (action_unit_enabled_) {
        spdlog::info("Setting up action unit device on port '{}' with baud rate {}.", action_unit_port_, action_unit_baud_rate_);
        action_unit_device_ = std::make_shared<ActionUnitDevice>();
        action_unit_manager_ = std::make_shared<SerialPortManager>(action_unit_port_, action_unit_baud_rate_);
        action_unit_manager_->start(action_unit_device_);
    } else {
        spdlog::info("Action unit device is disabled.");
    }
}

void BotSerialController::publish_imu_data()
{
    // Create and populate IMU message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "base_link";
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 9.81;

    // Publish IMU data
    imu_publisher_->publish(imu_msg);
    // spdlog::info("Published IMU data.");
}

void BotSerialController::publish_odom_data()
{
    // Create and populate Odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 1.0;
    odom_msg.pose.pose.position.y = 1.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    odom_msg.twist.twist.linear.x = 0.1;
    odom_msg.twist.twist.angular.z = 0.01;

    // Publish Odometry data
    odom_publisher_->publish(odom_msg);
    // spdlog::info("Published Odometry data.");
}

void BotSerialController::control_motor_status(bool status)
{
    if (!status) 
    {
        std::vector<uint8_t> data = {0xaa, 0x55, 0x01, 0x01, 0x00, 0x90, 0x21};
        chassis_manager_->write_data(data);
    }
    else
    {
        std::vector<uint8_t> data = {0xaa, 0x55, 0x01, 0x01, 0x01, 0x50, 0xE0};
        chassis_manager_->write_data(data);
    }
}

}  // namespace bot_serial
