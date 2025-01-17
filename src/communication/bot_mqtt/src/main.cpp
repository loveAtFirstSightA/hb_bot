#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bot_mqtt/bot_mqtt.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bot_mqtt::BotMqtt>());
    rclcpp::shutdown();
    return 0;
}
