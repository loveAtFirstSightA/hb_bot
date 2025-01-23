#include "bot_serial/bot_serial_controller.hpp"

namespace bot_serial
{

BotSerialController::BotSerialController() : Node("bot_serial_controller"),
    is_first_run_(true), last_right_encoder_(0), last_left_encoder_(0),
    x_position_(0.0), y_position_(0.0), theta_(0.0)
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

    // calculate odometry
    this->declare_parameter<double>("wheel_diameter", 0.17);
    this->declare_parameter<int>("encoder_pulse", 16384);
    this->declare_parameter<double>("wheel_base", 0.4885);

    this->get_parameter("wheel_diameter", wheel_diameter_);
    this->get_parameter("encoder_pulse", encoder_pulse_);
    this->get_parameter("wheel_base", wheel_base_);


    // Set up devices based on parameters
    setup_devices();

    // Initialize publishers for IMU and Odometry data
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // 处理速度指令
    vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&BotSerialController::vel_subscriber_callback, this, std::placeholders::_1));

    // Create timers for publishing data
    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // 100ms interval for IMU data
        std::bind(&BotSerialController::publish_imu_data, this));

    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50ms interval for Odometry data
        std::bind(&BotSerialController::publish_odom_data, this));
    
    chassis_ = std::dynamic_pointer_cast<ChassisDevice>(chassis_device_);
    action_ = std::dynamic_pointer_cast<ActionUnitDevice>(action_unit_device_);

    // TODO
    timer_info_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&BotSerialController::timer_info_callback, this));

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

void BotSerialController::vel_subscriber_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double v, w;
    v = msg->linear.x;
    w = msg->angular.z;
    control_motor_speed(v, w);
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

void BotSerialController::calculate_odometry(double& x, double& y, double& theta, double& v, double& w)
{
    double wheel_diameter = this->wheel_diameter_;   // 轮子的直径
    int encoder_pulse = this->encoder_pulse_;         // 编码器的脉冲数
    double wheel_base = this->wheel_base_;            // 轮距，即左右轮之间的距离

    // 如果是第一次运行，只记录初值，不进行计算
    if (this->is_first_run_) {
        // 获取首次运行的左右轮编码器值
        this->last_right_encoder_ = chassis_->get_right_motor_encoder();
        this->last_left_encoder_ = chassis_->get_left_motor_encoder();
        
        // 设置为不是第一次运行
        this->is_first_run_ = false;
        
        return;  // 跳过计算
    }

    // 获取当前周期的左右轮的编码器数值
    int right_encoder = chassis_->get_right_motor_encoder();
    int left_encoder = chassis_->get_left_motor_encoder();

    // 获取左右电机的转速（单位：转每分钟，rpm）
    double right_motor_speed = chassis_->get_right_motor_speed(); // 获取右轮电机转速
    double left_motor_speed = chassis_->get_left_motor_speed();  // 获取左轮电机转速

    // 计算每次编码器脉冲对应的轮子行进的距离
    double wheel_circumference = M_PI * wheel_diameter;  // 轮子的周长
    double distance_per_pulse = wheel_circumference / encoder_pulse;  // 每个脉冲对应的行进距离

    // 计算每个周期内的编码器脉冲差值（即两轮的运动增量）
    int right_delta = right_encoder - this->last_right_encoder_;
    int left_delta = left_encoder - this->last_left_encoder_;

    // 更新上次编码器数值，用于下一周期计算
    this->last_right_encoder_ = right_encoder;
    this->last_left_encoder_ = left_encoder;

    // 计算周期内左右轮的行进距离
    double right_distance = right_delta * distance_per_pulse;
    double left_distance = left_delta * distance_per_pulse;

    // 计算机器人前进的距离（左右轮平均距离）
    double linear_distance = (right_distance + left_distance) / 2.0;

    // 计算机器人的角速度（通过两轮之间的差速来计算）
    double angular_distance = (right_distance - left_distance) / wheel_base;

    // 更新机器人的位置（假设我们有一个全局的机器人位置信息变量 x_position_，y_position_，theta_）
    this->x_position_ += linear_distance * cos(this->theta_);
    this->y_position_ += linear_distance * sin(this->theta_);
    this->theta_ += angular_distance;

    // 更新机器人姿态
    this->theta_ = std::fmod(this->theta_, 2 * M_PI);  // 保证角度在 -π 到 π 之间

    // TODO 根据左右电机的转速计算中心的线速度和角速度
    double right_linear_velocity = right_motor_speed * wheel_circumference / 60.0;  // 单位：m/s
    double left_linear_velocity = left_motor_speed * wheel_circumference / 60.0;    // 单位：m/s

    // 计算机器人中心的线速度v和角速度w
    this->v_ = (right_linear_velocity + left_linear_velocity) / 2.0; // 线速度
    this->w_ = (right_linear_velocity - left_linear_velocity) / wheel_base; // 角速度

    // 你可以根据v_和w_进行位置更新，或用于其他控制逻辑
    v = this->v_;  // 通过引用将更新后的线速度传递出去
    w = this->w_;  // 通过引用将更新后的角速度传递出去

    // 返回更新后的位置、角度
    x = this->x_position_;
    y = this->y_position_;
    theta = this->theta_;
}

void BotSerialController::publish_odom_data()
{
    // Create and populate Odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Declare variables for position and velocity
    double x = 0.0, y = 0.0, theta = 0.0;
    double v = 0.0, w = 0.0;

    // Call calculate_odometry to update position and velocity
    calculate_odometry(x, y, theta, v, w);

    // Populate Odometry message with updated position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    // Calculate orientation from theta (angle) and set it in the message
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, theta);  // Roll, pitch, yaw (theta)
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();

    // Initialize covariance to zero
    for (size_t i = 0; i < odom_msg.pose.covariance.size(); i++) {
        odom_msg.pose.covariance[i] = 0.0f;
    }

    // Populate Odometry message with velocity data
    odom_msg.twist.twist.linear.x = v;  // Linear velocity in the x direction
    odom_msg.twist.twist.angular.z = w; // Angular velocity around z axis

    // Initialize twist covariance to zero
    for (size_t i = 0; i < odom_msg.twist.covariance.size(); i++) {
        odom_msg.twist.covariance[i] = 0.0f;
    }

    // Publish Odometry data
    odom_publisher_->publish(odom_msg);
    // spdlog::info("Published Odometry data.");
}

void BotSerialController::enable_motor()
{
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x01;
    uint8_t data_length = 0x01;
    uint8_t data = 0x01;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("enable motor, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::disable_motor()
{
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x01;
    uint8_t data_length = 0x01;
    uint8_t data = 0x00;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("disable motor, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::control_led_info_red()
{
    // 数据定义：1：亮红灯，充电中，0：亮绿灯，充电完成，正常状态 
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x08;
    uint8_t data_length = 0x01;
    uint8_t data = 0x01;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("control led info red, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::control_led_info_green()
{
    // 数据定义：1：亮红灯，充电中，0：亮绿灯，充电完成，正常状态 
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x08;
    uint8_t data_length = 0x01;
    uint8_t data = 0x00;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("control led info green, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::enable_emergency_switch()
{
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x07;
    uint8_t data_length = 0x01;
    uint8_t data = 0x01;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("enable emergency switch, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::disable_emergency_switch()
{
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x07;
    uint8_t data_length = 0x01;
    uint8_t data = 0x00;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("disable emergency switch, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::clear_motor_encoder_value()
{
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x06;
    uint8_t data_length = 0x01;
    uint8_t data = 0x00;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("clear motor encoder value, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::start_recharge()
{
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x04;
    uint8_t data_length = 0x01;
    uint8_t data = 0x01;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("start recharge, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::close_recharge()
{
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x04;
    uint8_t data_length = 0x01;
    uint8_t data = 0x00;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("close recharge, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

void BotSerialController::turn_off_calculate_power()
{
    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x25;
    uint8_t data_length = 0x01;
    uint8_t data = 0x01;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("turn off calculate power, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}

// TODO
void BotSerialController::control_motor_speed(const double v, const double w)
{
    double wheel_diameter = this->wheel_diameter_;  // 轮子的直径
    double wheel_base = this->wheel_base_;           // 轮距，即左右轮之间的距离
    double wheel_radius = wheel_diameter / 2.0;      // 轮子的半径

    // 计算左右轮的线速度 (单位：rad/s)
    double left_speed_rad_per_sec = (v - (w * wheel_base / 2.0)) / wheel_radius;
    double right_speed_rad_per_sec = (v + (w * wheel_base / 2.0)) / wheel_radius;

    // 将 rad/s 转换为 r/min
    double left_speed_rpm = (left_speed_rad_per_sec * 60.0) / (2.0 * M_PI);
    double right_speed_rpm = (right_speed_rad_per_sec * 60.0) / (2.0 * M_PI);

    // 打印计算结果
    spdlog::info("Left wheel speed: {:.2f} r/min", left_speed_rpm);  // 输出左轮的转速（单位：r/min）
    spdlog::info("Right wheel speed: {:.2f} r/min", right_speed_rpm);  // 输出右轮的转速（单位：r/min）

    // crc 校验 数据类型 数据长度 数据
    std::vector<uint8_t> frame;
    uint8_t heading1 = 0xaa;
    uint8_t heading2 = 0x55;
    uint8_t type = 0x02;
    uint8_t data_length = 0x05;

    // D1-D2:  左轮电机转速，单位：r/min 
    // D3-D4:  右轮电机转速，单位：r/min 
    // int16_t left_motor_spd; 
    // int16_t right_mode_spd; 
    uint8_t data = 0x03;

    // CRC 校验数据部分
    std::vector<uint8_t> crc_data = {type, data_length, data};

    // 计算 CRC 校验码
    uint16_t crc = calculate_crc16(crc_data);

    // 组装数据：添加头部信息
    frame.push_back(heading1);
    frame.push_back(heading2);
    frame.push_back(type);
    frame.push_back(data_length);
    frame.push_back(data);

    // 在数据末尾添加新的 CRC 校验码，按高字节在前，低字节在后的顺序
    frame.push_back((crc >> 8) & 0xFF); // CRC 高字节
    frame.push_back(crc & 0xFF);        // CRC 低字节

    // 打印整个 frame 中的数据
    std::ostringstream oss;
    for (auto byte : frame) {
        oss << "0x" << std::hex << std::uppercase << (int)byte << " ";
    }
    // 使用 spdlog 打印
    spdlog::info("control motor speed, frame: {}", oss.str());

    // 写入数据，包括新的 CRC 校验码
    chassis_manager_->write_data(frame);
}


void BotSerialController::open_cell_one()
{
    std::vector<uint8_t> data = {0xaa, 0x55, 0x84, 0x06, 0x00, 0x08, 0x00, 0x04, 0x16, 0x5E};
    spdlog::info("open_cell_one");
    action_unit_manager_->write_data(data);
}

void BotSerialController::close_cell_one()
{
    std::vector<uint8_t> data = {0xaa, 0x55, 0x84, 0x06, 0x00, 0x08, 0x00, 0x07, 0x56, 0x5F};
    spdlog::info("close_cell_one");
    action_unit_manager_->write_data(data);
}

void BotSerialController::open_cell_two()
{
    std::vector<uint8_t> data = {0xaa, 0x55, 0x84, 0x06, 0x00, 0x09, 0x00, 0x04, 0x47, 0x9E};
    spdlog::info("open_cell_two");
    action_unit_manager_->write_data(data);
}

void BotSerialController::close_cell_two()
{
    std::vector<uint8_t> data = {0xaa, 0x55, 0x84, 0x06, 0x00, 0x09, 0x00, 0x07, 0x07, 0x9F};
    spdlog::info("close_cell_two");
    action_unit_manager_->write_data(data);
}

void BotSerialController::open_cell_three()
{
    std::vector<uint8_t> data = {0xaa, 0x55, 0x84, 0x06, 0x00, 0x0A, 0x00, 0x04, 0xB7, 0x9E};
    spdlog::info("open_cell_three");
    action_unit_manager_->write_data(data);
}

void BotSerialController::close_cell_three()
{
    std::vector<uint8_t> data = {0xaa, 0x55, 0x84, 0x06, 0x00, 0x0A, 0x00, 0x07, 0xF7, 0x9F};
    spdlog::info("close_cell_three");
    action_unit_manager_->write_data(data);
}

void BotSerialController::open_cell_four()
{
    std::vector<uint8_t> data = {0xaa, 0x55, 0x84, 0x06, 0x00, 0x0B, 0x00, 0x04, 0xE6, 0x5E};
    spdlog::info("open_cell_four");
    action_unit_manager_->write_data(data);
}

void BotSerialController::close_cell_four()
{
    std::vector<uint8_t> data = {0xaa, 0x55, 0x84, 0x06, 0x00, 0x0B, 0x00, 0x07, 0xA6, 0x5F};
    spdlog::info("close_cell_four");
    action_unit_manager_->write_data(data);
}


}  // namespace bot_serial
