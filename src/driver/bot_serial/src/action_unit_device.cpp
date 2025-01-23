#include "bot_serial/action_unit_device.hpp"
#include <spdlog/spdlog.h>

namespace bot_serial
{

void ActionUnitDevice::process_data(const std::string &data)
{
    spdlog::info("ActionUnitDevice received data: {}", data);
}


}  // namespace bot_serial
