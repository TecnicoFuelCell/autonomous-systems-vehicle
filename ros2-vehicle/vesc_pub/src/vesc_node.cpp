#include "vesc_pub/vesc_node.hpp"

#include <cmath>
#include <vector>
#include <functional>
#include <exception>

#include "vesc_pub/uart_utils.hpp"

using namespace std::chrono_literals;

namespace vesc_pub {

rclcpp::Time safe_now(rclcpp::Node* n) {
    rclcpp::Time t = n->get_clock()->now();
    if (t.nanoseconds() != 0) return t;
    static rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    return wall_clock.now();
}

VesceNode::VesceNode()
: Node("vesc_node"), uart_fd_(-1)
{
    std::string vesc_topic = this->declare_parameter<std::string>("vesc_topic", "/vesc_data");
    std::string frame_id = this->declare_parameter<std::string>("frame_id", "");

    uart_fd_ = uart_utils::discover_or_fallback(this->get_logger());
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UART device for reading VESC data.");
        rclcpp::shutdown();
        return;
    }

    publisher_ = this->create_publisher<car_msgs::msg::VescData>(vesc_topic, 1);
    timer_ = this->create_wall_timer(5ms, std::bind(&VesceNode::read_uart, this));
}

void VesceNode::process(const std::string& data) {
    std::vector<std::string> parts = uart_utils::split(data, ',');
    if (parts.size() < 7) {
        RCLCPP_ERROR(this->get_logger(), "VESC parse error: Not enough parts in payload (%zu/7)", parts.size());
        return;
    }

    try {
        auto msg = car_msgs::msg::VescData();
        msg.header.stamp = safe_now(this);

        msg.tempmosfet = std::stof(parts[0]);
        msg.avgmotorcurrent = std::stof(parts[1]);
        msg.avginputcurrent = std::stof(parts[2]);
        msg.dutycyclenow = std::stof(parts[3]);
        msg.rpm = std::stof(parts[4]);
        msg.inpvoltage = std::stof(parts[5]);
        msg.watthours = std::stof(parts[6]);

        if (std::abs(msg.tempmosfet) < 1e-5 && std::abs(msg.avgmotorcurrent) < 1e-5 &&
            std::abs(msg.avginputcurrent) < 1e-5 && std::abs(msg.dutycyclenow) < 1e-5 &&
            std::abs(msg.rpm) < 1e-5 && std::abs(msg.inpvoltage) < 1e-5 &&
            std::abs(msg.watthours) < 1e-5) {
            RCLCPP_ERROR(this->get_logger(), "VESC ALERT: All values are exactly zero!");
        }

        RCLCPP_DEBUG(this->get_logger(), "Parsed VESC: rpm=%.2f, duty=%.2f, v_in=%.2f", msg.rpm, msg.dutycyclenow, msg.inpvoltage);
        publisher_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "VESC parse exception: %s. Payload: %s", e.what(), data.c_str());
    }
}

void VesceNode::read_uart() {
    if (uart_fd_ < 0) return;

    static std::string line_buffer;
    char c;
    while (read(uart_fd_, &c, 1) == 1) {
        if (c == '\n') {
            std::string line = uart_utils::trim(line_buffer);
            if (line.find("VESC:") == 0) {
                process(line.substr(5));
            }
            line_buffer.clear();
        } else {
            line_buffer += c;
        }
    }
}

} // namespace vesc_pub

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vesc_pub::VesceNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
