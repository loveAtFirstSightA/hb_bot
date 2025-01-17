#ifndef BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_
#define BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_

#include <thread>
#include <mutex>
#include <string>
#include <atomic>
#include "spdlog/spdlog.h"
#include "bot_serial/serial.h"
#include "bot_serial/device_unit.hpp"

namespace bot_serial
{
class SerialPortManager
{
public:
    SerialPortManager(const std::string &port, int baud_rate);
    ~SerialPortManager();

    void start(std::shared_ptr<DeviceUnit> device);
    void stop();

private:
    void read_thread();

    std::string port_;
    int baud_rate_;
    serial::Serial serial_port_;
    std::thread thread_;
    std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::shared_ptr<DeviceUnit> device_;

};
}  // namespace bot_serial

#endif  // BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_
