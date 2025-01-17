#ifndef BOT_MQTT_HPP
#define BOT_MQTT_HPP

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

namespace bot_mqtt 
{
class BotMqtt : public rclcpp::Node 
{
public:
    BotMqtt();
    ~BotMqtt();

private:

};

}  // namespace bot_mqtt

#endif  // BOT_MQTT_HPP
