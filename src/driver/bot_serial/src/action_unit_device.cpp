#include "bot_serial/action_unit_device.hpp"
#include <spdlog/spdlog.h>

namespace bot_serial
{

void ActionUnitDevice::process_data(const std::string &data)
{
    spdlog::info("ActionUnitDevice received data: {}", data);
}

void ActionUnitDevice::enable_device()
{
    spdlog::info("ActionUnitDevice enabled.");
}

void ActionUnitDevice::disable_device()
{
    spdlog::info("ActionUnitDevice disabled.");
}

}  // namespace bot_serial
