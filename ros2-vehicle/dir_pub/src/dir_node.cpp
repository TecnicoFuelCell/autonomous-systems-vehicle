#include "dir_pub/dir_node.hpp"

#include <string>
#include <functional>
#include <exception>

#include "dir_pub/uart_utils.hpp"

using namespace std::chrono_literals;

namespace dir_pub {

rclcpp::Time safe_now(rclcpp::Node* n) {
    rclcpp::Time t = n->get_clock()->now();
    if (t.nanoseconds() != 0) return t;
    static rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    return wall_clock.now();
}

DirNode::DirNode()
: Node("dir_node"), uart_fd_(-1)
{
    std::string dir_topic = this->declare_parameter<std::string>("dir_topic", "/dir_data");
    std::string frame_id = this->declare_parameter<std::string>("frame_id", "");

    uart_fd_ = uart_utils::discover_or_fallback(this->get_logger());
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UART device for reading Dir data.");
        rclcpp::shutdown();
        return;
    }

    publisher_ = this->create_publisher<car_msgs::msg::Dir>(dir_topic, 1);
    timer_ = this->create_wall_timer(5ms, std::bind(&DirNode::read_uart, this));
}

void DirNode::read_uart() {
    if (uart_fd_ < 0) return;

    static std::string line_buffer;
    char c;
    while (read(uart_fd_, &c, 1) == 1) {
        if (c == '\n') {
            std::string line = uart_utils::trim(line_buffer);
            if (line.find("Dir:") == 0) {
                try {
                    int value = std::stoi(uart_utils::trim(line.substr(4)));
                    auto msg = car_msgs::msg::Dir();
                    msg.header.stamp = safe_now(this);
                    msg.dir = value;

                    RCLCPP_DEBUG(this->get_logger(), "Parsed Dir: %d", value);
                    publisher_->publish(msg);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Dir parse exception: %s. Payload: %s", e.what(), line.c_str());
                }
            }
            line_buffer.clear();
        } else {
            line_buffer += c;
        }
    }
}

} // namespace dir_pub

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dir_pub::DirNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
