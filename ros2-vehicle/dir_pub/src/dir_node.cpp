#include "dir_pub/dir_node.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <string>
#include <functional>
#include <exception>

#include <rclcpp_components/register_node_macro.hpp>

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

// Sets this node's logger level from its log_level parameter (debug, info,
// warn, error, fatal). Overrides --ros-args --log-level for this node.
void apply_log_level(rclcpp::Node & node)
{
    std::string level = node.declare_parameter<std::string>("log_level", "info");
    std::transform(level.begin(), level.end(), level.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

    static const std::map<std::string, rclcpp::Logger::Level> levels = {
        {"debug", rclcpp::Logger::Level::Debug},
        {"info", rclcpp::Logger::Level::Info},
        {"warn", rclcpp::Logger::Level::Warn},
        {"error", rclcpp::Logger::Level::Error},
        {"fatal", rclcpp::Logger::Level::Fatal},
    };
    auto it = levels.find(level);
    if (it == levels.end()) {
        RCLCPP_WARN(node.get_logger(),
            "Unknown log_level '%s' (valid: debug, info, warn, error, fatal); using info", level.c_str());
        node.get_logger().set_level(rclcpp::Logger::Level::Info);
        return;
    }
    node.get_logger().set_level(it->second);
}
} // namespace

DirNode::DirNode(const rclcpp::NodeOptions & options)
: Node("dir_node", options)
{
    apply_log_level(*this);

    std::string dir_topic = this->declare_parameter<std::string>("dir_topic", "/dir_data");

    publisher_ = this->create_publisher<car_msgs::msg::Dir>(dir_topic, 1);
    std::string serial_topic =
        this->declare_parameter<std::string>("serial_dir_topic", "/vehicle_internal/serial/dir");
    subscription_ = this->create_subscription<std_msgs::msg::String>(serial_topic, 10,
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
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Dir parse exception: %s. Payload: %s", e.what(), msg->data.c_str());
    }
}

} // namespace dir_pub

RCLCPP_COMPONENTS_REGISTER_NODE(dir_pub::DirNode)
