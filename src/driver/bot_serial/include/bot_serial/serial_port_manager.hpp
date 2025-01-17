#ifndef BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_
#define BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_

#include <thread>
#include <mutex>
#include <string>
#include <atomic>
#include <queue>
#include <condition_variable>
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

    // 写数据方法，写入队列
    void write_data(const std::vector<uint8_t> &data);

private:
    void read_thread();
    void write_thread();

    std::string port_;
    int baud_rate_;
    serial::Serial serial_port_;
    std::thread read_thread_;
    std::thread write_thread_;
    std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::shared_ptr<DeviceUnit> device_;

    // 写数据队列
    std::queue<std::vector<uint8_t>> write_queue_;
    std::condition_variable write_cond_;
};
}  // namespace bot_serial

#endif  // BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_
