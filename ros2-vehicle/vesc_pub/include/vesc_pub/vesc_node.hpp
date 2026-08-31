#ifndef VESC_PUB_VESC_NODE_HPP_
#define VESC_PUB_VESC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <car_msgs/msg/vesc_data.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace vesc_pub {

class VesceNode : public rclcpp::Node
{
public:
    VesceNode();

private:
    void process(const std_msgs::msg::String::ConstSharedPtr msg);

    rclcpp::Publisher<car_msgs::msg::VescData>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

} // namespace vesc_pub

#endif // VESC_PUB_VESC_NODE_HPP_
