#ifndef UART_WRITER_UART_WRITER_NODE_HPP_
#define UART_WRITER_UART_WRITER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <car_msgs/msg/im_speed.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <string>

namespace uart_writer {

class UartWriterNode : public rclcpp::Node
{
public:
    UartWriterNode();
    ~UartWriterNode();

private:
    struct ZeroCommandTimer {
        bool should_send(int value, std::chrono::steady_clock::time_point now);

        bool running{false};
        std::chrono::steady_clock::time_point started_at;
    };

    void deadman_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void joy_callback(const car_msgs::msg::ImSpeed::SharedPtr msg);
    void write_serial(const std::string& message);

    int uart_fd_{-1};
    bool car_on_{false};
    ZeroCommandTimer drive_zero_timer_;
    ZeroCommandTimer steering_zero_timer_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
    rclcpp::Subscription<car_msgs::msg::ImSpeed>::SharedPtr movement_sub_;
};

} // namespace uart_writer

#endif // UART_WRITER_UART_WRITER_NODE_HPP_
