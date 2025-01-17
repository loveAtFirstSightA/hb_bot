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
            thread_ = std::thread(&SerialPortManager::read_thread, this);
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
    if (thread_.joinable())
    {
        thread_.join();
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

}  // namespace bot_serial
