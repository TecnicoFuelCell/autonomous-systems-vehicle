#ifndef DIR_PUB_DIR_NODE_HPP_
#define DIR_PUB_DIR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <car_msgs/msg/dir.hpp>
#include <std_msgs/msg/string.hpp>

namespace dir_pub {

class DirNode : public rclcpp::Node
{
public:
    DirNode();

private:
    void process(const std_msgs::msg::String::ConstSharedPtr msg);

    rclcpp::Publisher<car_msgs::msg::Dir>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

} // namespace dir_pub

#endif // DIR_PUB_DIR_NODE_HPP_
