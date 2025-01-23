#ifndef BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_
#define BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_

// 包含必要的头文件
#include <iomanip>                      // 用于格式化输出（例如设置输出为十六进制格式）
#include <thread>                       // 用于多线程操作，创建和管理线程
#include <mutex>                        // 用于线程间的互斥操作，防止多个线程同时访问共享资源
#include <string>                       // 用于字符串处理和操作
#include <atomic>                       // 提供原子操作，用于线程间的数据同步和状态管理
#include <queue>                        // 用于存储数据的队列，通常用于生产者-消费者模式
#include <condition_variable>           // 用于线程间的条件变量同步，配合互斥锁使用，用于线程间的等待和通知机制
#include "spdlog/spdlog.h"              // 用于日志记录的库，提供高效且灵活的日志功能
#include "bot_serial/serial.h"          // 自定义的串口操作封装类，提供串口的读写操作封装
#include "bot_serial/device_unit.hpp"   // 自定义设备单元类，用于处理串口数据和设备交互

namespace bot_serial  // 定义一个命名空间，用于串口相关功能
{
// 串口管理类，封装了串口的读写功能及线程管理
class SerialPortManager
{
public:
    // 构造函数：初始化串口参数，包括端口号和波特率
    SerialPortManager(const std::string &port, int baud_rate);

    // 析构函数：负责资源释放，例如线程和串口关闭
    ~SerialPortManager();

    // 启动串口管理器，与设备关联
    // 参数：`device` 设备单元的共享指针，用于设备交互
    void start(std::shared_ptr<DeviceUnit> device);

    // 停止串口管理器，停止读写线程并释放资源
    void stop();

    // 写数据方法，将数据写入队列
    // 参数：`data` 要写入的字节数据
    void write_data(const std::vector<uint8_t> &data);

private:
    // 串口读线程函数：从串口读取数据并交给设备单元处理
    void read_thread();

    // 串口写线程函数：从写队列中取出数据并写入串口
    void write_thread();

    // 串口参数
    std::string port_;          // 串口端口号
    int baud_rate_;             // 串口波特率
    serial::Serial serial_port_; // 串口对象，用于实际的读写操作

    // 多线程相关
    std::thread read_thread_;   // 串口读取线程
    std::thread write_thread_;  // 串口写入线程
    std::mutex mutex_;          // 互斥锁，保护共享资源
    std::atomic<bool> running_; // 原子标志位，指示管理器是否运行

    // 设备单元
    std::shared_ptr<DeviceUnit> device_; // 设备单元，用于处理读到的数据

    // 写数据队列
    std::queue<std::vector<uint8_t>> write_queue_; // 写数据队列，存放待写入的数据
    std::condition_variable write_cond_;          // 条件变量，用于写线程的同步
};
}  // namespace bot_serial

#endif  // BOT_SERIAL__SERIAL_PORT_MANAGER_HPP_
