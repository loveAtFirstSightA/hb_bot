#ifndef BOT_SOCKET_HPP
#define BOT_SOCKET_HPP

#include "rclcpp/rclcpp.hpp"
#include "bot_socket/socket_server.hpp"

namespace bot_socket
{

class BotSocket : public rclcpp::Node
{
public:
    BotSocket();
    ~BotSocket();

private:

};

}  // namespace bot_socket

#endif  // BOT_SOCKET_HPP
