#include "bot_serial/serial_port_manager.hpp"

namespace bot_serial
{

SerialPortManager::SerialPortManager(const std::string &port, int baud_rate)
    : port_(port), baud_rate_(baud_rate)
{
}

SerialPortManager::~SerialPortManager()
{
    stop();
}

void SerialPortManager::start(std::shared_ptr<DeviceUnit> device)
{
    device_ = device;
    running_ = true;

    try
    {
        serial_port_.setPort(port_);
        serial_port_.setBaudrate(baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        serial_port_.setTimeout(timeout);
        serial_port_.open();

        if (serial_port_.isOpen())
        {
            spdlog::info("Serial port {} opened at baud rate {}", port_, baud_rate_);
            read_thread_ = std::thread(&SerialPortManager::read_thread, this);
            write_thread_ = std::thread(&SerialPortManager::write_thread, this);
        }
        else
        {
            spdlog::error("Failed to open serial port {}", port_);
        }
    }
    catch (const std::exception &e)
    {
        spdlog::error("Exception while opening serial port {}: {}", port_, e.what());
    }
}

void SerialPortManager::stop()
{
    running_ = false;
    if (read_thread_.joinable())
    {
        read_thread_.join();
    }
    if (write_thread_.joinable())
    {
        write_thread_.join();
    }
    if (serial_port_.isOpen())
    {
        serial_port_.close();
    }
}

void SerialPortManager::read_thread()
{
    while (running_)
    {
        try
        {
            if (serial_port_.available() > 0)
            {
                std::string data = serial_port_.read(64);
                // std::string data = serial_port_.readline();
                if (device_)
                {
                    device_->process_data(data);
                    // spdlog::info("Device processed data: {}", data);
                }
            }
        }
        catch (const std::exception &e)
        {
            spdlog::error("Error reading from serial port {}: {}", port_, e.what());
        }
    }
}

void SerialPortManager::write_thread()
{
    while (running_)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        write_cond_.wait(lock, [this]() { return !write_queue_.empty() || !running_; });

        if (!write_queue_.empty())
        {
            std::vector<uint8_t> data = write_queue_.front();
            write_queue_.pop();
            lock.unlock();  // Release the lock while writing

            try
            {
                if (serial_port_.isOpen())
                {
                    serial_port_.write(data.data(), data.size());
                    spdlog::info("Data written to serial port: {} bytes", data.size());
                }
                else
                {
                    spdlog::error("Serial port {} is not open", port_);
                }
            }
            catch (const std::exception &e)
            {
                spdlog::error("Error writing to serial port {}: {}", port_, e.what());
            }
        }
    }
}

// 使用示例
// std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
// serial_port_manager.write_data(data);

void SerialPortManager::write_data(const std::vector<uint8_t> &data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    write_queue_.push(data);
    write_cond_.notify_one();  // Notify the write thread that data is available
}

}  // namespace bot_serial
