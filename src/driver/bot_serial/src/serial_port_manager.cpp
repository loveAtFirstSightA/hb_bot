#include "bot_serial/serial_port_manager.hpp"

namespace bot_serial
{

// 构造函数：初始化串口管理器的端口号和波特率
SerialPortManager::SerialPortManager(const std::string &port, int baud_rate)
    : port_(port), baud_rate_(baud_rate)
{
}

// 析构函数：调用停止方法以释放所有资源
SerialPortManager::~SerialPortManager()
{
    stop();
}

// 启动串口管理器
// 参数：`device` 设备单元的共享指针，用于处理串口数据
void SerialPortManager::start(std::shared_ptr<DeviceUnit> device)
{
    device_ = device;  // 保存设备单元对象
    running_ = true;   // 设置运行标志位为真

    try
    {
        // 设置串口参数
        serial_port_.setPort(port_);              // 设置端口号
        serial_port_.setBaudrate(baud_rate_);     // 设置波特率
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100); // 设置超时时间为100毫秒
        serial_port_.setTimeout(timeout);         // 应用超时时间
        serial_port_.open();                      // 打开串口

        if (serial_port_.isOpen())
        {
            // 如果串口打开成功，记录日志并启动读写线程
            spdlog::info("Serial port {} opened at baud rate {}", port_, baud_rate_);
            read_thread_ = std::thread(&SerialPortManager::read_thread, this);   // 启动读取线程
            write_thread_ = std::thread(&SerialPortManager::write_thread, this); // 启动写入线程
        }
        else
        {
            // 如果串口打开失败，记录错误日志
            spdlog::error("Failed to open serial port {}", port_);
        }
    }
    catch (const std::exception &e)
    {
        // 捕获打开串口时的异常，并记录日志
        spdlog::error("Exception while opening serial port {}, {}", port_, e.what());
    }
}

// 停止串口管理器：关闭线程并释放资源
void SerialPortManager::stop()
{
    running_ = false; // 设置运行标志位为假

    // 等待读取线程结束
    if (read_thread_.joinable())
    {
        read_thread_.join();
    }

    // 等待写入线程结束
    if (write_thread_.joinable())
    {
        write_thread_.join();
    }

    // 关闭串口
    if (serial_port_.isOpen())
    {
        serial_port_.close();
    }
}

// 串口读取线程函数
void SerialPortManager::read_thread()
{
    while (running_) // 当管理器处于运行状态时循环读取
    {
        try
        {
            // 检查串口中是否有数据可读
            if (serial_port_.available() > 0)
            {
                std::string data = serial_port_.read(64); // 从串口读取最多64字节数据
                // 将数据交给设备单元处理
                if (device_)
                {
                    device_->process_data(data); // 调用设备单元的处理函数
                    // spdlog::info("Device processed data: {}", data); // 可选：记录处理的数据
                }
            }
        }
        catch (const std::exception &e)
        {
            // 捕获读取数据时的异常，并记录错误日志
            spdlog::error("Error reading from serial port {}: {}", port_, e.what());
        }
    }
}

// 串口写入线程函数
void SerialPortManager::write_thread()
{
    while (running_) // 当管理器处于运行状态时循环写入
    {
        // 加锁并等待条件变量触发，表明有数据可写或停止运行
        std::unique_lock<std::mutex> lock(mutex_);
        write_cond_.wait(lock, [this]() { return !write_queue_.empty() || !running_; });

        // 如果写队列中有数据
        if (!write_queue_.empty())
        {
            // 从队列中取出一条数据
            std::vector<uint8_t> data = write_queue_.front();
            write_queue_.pop();
            lock.unlock(); // 解锁以避免长时间持有锁

            try
            {
                // 如果串口已打开，写入数据
                if (serial_port_.isOpen())
                {
                    serial_port_.write(data.data(), data.size());
                    // 将数据转换为十六进制字符串以便显示
                    std::ostringstream oss;
                    for (const auto &byte : data)
                    {
                        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                    }
                    // 记录写入数据的详细内容
                    spdlog::info("Data written to serial port {}: {} bytes, data: [{}]", port_, data.size(), oss.str());
                }
                else
                {
                    spdlog::error("Serial port {} is not open", port_);
                }
            }
            catch (const std::exception &e)
            {
                // 捕获写入数据时的异常，并记录错误日志
                spdlog::error("Error writing to serial port {}: {}", port_, e.what());
            }
        }
    }
}

// 写数据方法：将数据加入写队列并通知写线程
// 参数：`data` 要写入的字节数据
void SerialPortManager::write_data(const std::vector<uint8_t> &data)
{
    std::lock_guard<std::mutex> lock(mutex_); // 加锁保护写队列
    write_queue_.push(data);                  // 将数据加入队列
    write_cond_.notify_one();                 // 通知写线程数据已可用
}

}  // namespace bot_serial
