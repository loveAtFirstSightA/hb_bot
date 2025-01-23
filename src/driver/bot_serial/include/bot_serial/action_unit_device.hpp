#ifndef BOT_SERIAL__ACTION_UNIT_DEVICE_HPP_
#define BOT_SERIAL__ACTION_UNIT_DEVICE_HPP_

#include "bot_serial/device_unit.hpp"

namespace bot_serial
{
class ActionUnitDevice : public DeviceUnit
{
public:
    void process_data(const std::string &data) override;
    // void enable_device() override;
    // void disable_device() override;
};
}  // namespace bot_serial

#endif  // BOT_SERIAL__ACTION_UNIT_DEVICE_HPP_
