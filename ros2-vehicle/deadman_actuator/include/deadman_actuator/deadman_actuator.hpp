#ifndef DEADMAN_ACTUATOR_DEADMAN_ACTUATOR_NODE_HPP_
#define DEADMAN_ACTUATOR_DEADMAN_ACTUATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <car_msgs/msg/im_speed.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <string>

namespace deadman_actuator {

class DeadmanActuatorNode : public rclcpp::Node
{
public:
    DeadmanActuatorNode();
    ~DeadmanActuatorNode();

private:
    void deadman_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void listener_callback(const car_msgs::msg::ImSpeed::SharedPtr msg);
    void process_alive();
    void read_uart();
    void write_serial(const std::string& message);

    int uart_fd_{-1};

    bool car_on_{false};

    std::chrono::steady_clock::time_point last_alive_time_;

    rclcpp::TimerBase::SharedPtr alive_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alive_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
    rclcpp::Subscription<car_msgs::msg::ImSpeed>::SharedPtr subscription_;
};

} // namespace deadman_actuator

#endif // DEADMAN_ACTUATOR_DEADMAN_ACTUATOR_NODE_HPP_
