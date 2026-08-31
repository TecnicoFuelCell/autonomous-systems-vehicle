#include "dir_pub/dir_node.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <functional>
#include <exception>

using namespace std::chrono_literals;

namespace dir_pub {

namespace {
rclcpp::Time safe_now(rclcpp::Node* n) {
    rclcpp::Time t = n->get_clock()->now();
    if (t.nanoseconds() != 0) return t;
    static rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    return wall_clock.now();
}

std::string trim(const std::string& str) {
    std::string s = str;
    s.erase(s.begin(), std::find_if_not(s.begin(), s.end(),
        [](unsigned char ch) { return std::isspace(ch); }));
    s.erase(std::find_if_not(s.rbegin(), s.rend(),
        [](unsigned char ch) { return std::isspace(ch); }).base(), s.end());
    return s;
}
} // namespace

DirNode::DirNode()
: Node("dir_node")
{
    std::string dir_topic = this->declare_parameter<std::string>("dir_topic", "/dir_data");

    publisher_ = this->create_publisher<car_msgs::msg::Dir>(dir_topic, 1);
    subscription_ = this->create_subscription<std_msgs::msg::String>("/serial/dir", 10,
        std::bind(&DirNode::process, this, std::placeholders::_1));
}

void DirNode::process(const std_msgs::msg::String::ConstSharedPtr msg) {
    try {
        int value = std::stoi(trim(msg->data));
        auto out = car_msgs::msg::Dir();
        out.header.stamp = safe_now(this);
        out.dir = value;

        RCLCPP_DEBUG(this->get_logger(), "Parsed Dir: %d", value);
        publisher_->publish(out);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Dir parse exception: %s. Payload: %s", e.what(), msg->data.c_str());
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
