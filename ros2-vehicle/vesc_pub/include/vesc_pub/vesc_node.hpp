#ifndef VESC_PUB_VESC_NODE_HPP_
#define VESC_PUB_VESC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <car_msgs/msg/vesc_data.hpp>

#include <string>

namespace vesc_pub {

class VesceNode : public rclcpp::Node
{
public:
    VesceNode();

private:
    void process(const std::string& data);
    void read_uart();

    int uart_fd_{-1};

    rclcpp::Publisher<car_msgs::msg::VescData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace vesc_pub

#endif // VESC_PUB_VESC_NODE_HPP_
