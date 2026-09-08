#ifndef DEADMAN_PUB_DEADMAN_NODE_HPP_
#define DEADMAN_PUB_DEADMAN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

namespace deadman_pub {

class DeadmanNode : public rclcpp::Node
{
public:
    DeadmanNode();

private:
    void serial_deadman_callback(const std_msgs::msg::String::ConstSharedPtr msg);
    void deadman_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void process_alive();

    double alive_timeout_{0.5};
    bool car_on_{false};

    std::chrono::steady_clock::time_point last_alive_time_;

    rclcpp::TimerBase::SharedPtr alive_timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alive_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_deadman_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
};

} // namespace deadman_pub

#endif // DEADMAN_PUB_DEADMAN_NODE_HPP_