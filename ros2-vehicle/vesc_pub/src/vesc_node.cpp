#include "vesc_pub/vesc_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <vector>
#include <functional>
#include <exception>
#include <sstream>

using namespace std::chrono_literals;

namespace vesc_pub {

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

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::istringstream stream(s);
    std::string token;
    while (std::getline(stream, token, delimiter)) {
        tokens.push_back(trim(token));
    }
    return tokens;
}
} // namespace

VesceNode::VesceNode()
: Node("vesc_node")
{
    std::string vesc_topic = this->declare_parameter<std::string>("vesc_topic", "/vesc_data");

    publisher_ = this->create_publisher<car_msgs::msg::VescData>(vesc_topic, 1);
    subscription_ = this->create_subscription<std_msgs::msg::String>("/serial/vesc", 10,
        std::bind(&VesceNode::process, this, std::placeholders::_1));
}

void VesceNode::process(const std_msgs::msg::String::ConstSharedPtr msg) {
    std::vector<std::string> parts = split(msg->data, ',');
    if (parts.size() < 7) {
        RCLCPP_ERROR(this->get_logger(), "VESC parse error: Not enough parts in payload (%zu/7)", parts.size());
        return;
    }

    try {
        auto out = car_msgs::msg::VescData();
        out.header.stamp = safe_now(this);

        out.tempmosfet = std::stof(parts[0]);
        out.avgmotorcurrent = std::stof(parts[1]);
        out.avginputcurrent = std::stof(parts[2]);
        out.dutycyclenow = std::stof(parts[3]);
        out.rpm = std::stof(parts[4]);
        out.inpvoltage = std::stof(parts[5]);
        out.watthours = std::stof(parts[6]);

        if (std::abs(out.tempmosfet) < 1e-5 && std::abs(out.avgmotorcurrent) < 1e-5 &&
            std::abs(out.avginputcurrent) < 1e-5 && std::abs(out.dutycyclenow) < 1e-5 &&
            std::abs(out.rpm) < 1e-5 && std::abs(out.inpvoltage) < 1e-5 &&
            std::abs(out.watthours) < 1e-5) {
            RCLCPP_ERROR(this->get_logger(), "VESC ALERT: All values are exactly zero!");
        }

        RCLCPP_DEBUG(this->get_logger(), "Parsed VESC: rpm=%.2f, duty=%.2f, v_in=%.2f", out.rpm, out.dutycyclenow, out.inpvoltage);
        publisher_->publish(out);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "VESC parse exception: %s. Payload: %s", e.what(), msg->data.c_str());
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
