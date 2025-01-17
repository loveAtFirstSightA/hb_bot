#ifndef BOT_SERIAL__DEVICE_UNIT_HPP_
#define BOT_SERIAL__DEVICE_UNIT_HPP_

#include <memory>
#include <string>

namespace bot_serial
{
class DeviceUnit
{
public:
    virtual ~DeviceUnit() = default;

    virtual void process_data(const std::string &data) = 0;
    virtual void enable_device() = 0;
    virtual void disable_device() = 0;
};
}  // namespace bot_serial

#endif  // BOT_SERIAL__DEVICE_UNIT_HPP_
