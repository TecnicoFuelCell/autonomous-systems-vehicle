#ifndef DIR_PUB_DIR_NODE_HPP_
#define DIR_PUB_DIR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <car_msgs/msg/dir.hpp>

namespace dir_pub {

class DirNode : public rclcpp::Node
{
public:
    DirNode();

private:
    void read_uart();

    int uart_fd_{-1};

    rclcpp::Publisher<car_msgs::msg::Dir>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace dir_pub

#endif // DIR_PUB_DIR_NODE_HPP_
