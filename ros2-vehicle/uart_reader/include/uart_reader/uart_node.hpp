#ifndef UART_READER_UART_NODE_HPP_
#define UART_READER_UART_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <string>

namespace uart_reader {

class UartNode : public rclcpp::Node
{
public:
    UartNode();
    ~UartNode();

private:
    void read_uart();
    void handle_line(const std::string& line);

    int uart_fd_{-1};

    std::string line_buffer_;

    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vesc_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dir_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acc_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gyro_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deadman_pub_;
};

} // namespace uart_reader

#endif // UART_READER_UART_NODE_HPP_