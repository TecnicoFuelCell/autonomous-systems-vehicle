#include "deadman_pub/deadman_node.hpp"

#include <algorithm>
#include <cctype>
#include <string>

using namespace std::chrono_literals;

namespace deadman_pub {

namespace {
std::string trim(const std::string& str) {
    std::string s = str;
    s.erase(s.begin(), std::find_if_not(s.begin(), s.end(),
        [](unsigned char ch) { return std::isspace(ch); }));
    s.erase(std::find_if_not(s.rbegin(), s.rend(),
        [](unsigned char ch) { return std::isspace(ch); }).base(), s.end());
    return s;
}
} // namespace

DeadmanNode::DeadmanNode()
: Node("deadman_node")
{
    alive_timeout_ = this->declare_parameter<double>("alive_timeout", 0.5);

    alive_pub_ = this->create_publisher<std_msgs::msg::Bool>("/deadman/alive", 1);

    serial_deadman_sub_ = this->create_subscription<std_msgs::msg::String>("/serial/deadman", 10,
        std::bind(&DeadmanNode::serial_deadman_callback, this, std::placeholders::_1));

    // External re-arm path: a fresh /deadman/alive=true (from another source)
    // feeds the watchdog just like a serial ALIVE line.
    deadman_sub_ = this->create_subscription<std_msgs::msg::Bool>("/deadman/alive", 10,
        std::bind(&DeadmanNode::deadman_callback, this, std::placeholders::_1));

    // Start the deadman stale so the system stays disarmed until a real
    // ALIVE (/serial/deadman) or a fresh /deadman/alive=true arrives.
    last_alive_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);

    // Watchdog only ever DISARMS; arming is done solely by a real ALIVE line
    // or a deadman-true event.
    alive_timer_ = this->create_wall_timer(50ms, std::bind(&DeadmanNode::process_alive, this));

    RCLCPP_INFO(this->get_logger(), "Deadman Node initialized successfully.");
}

void DeadmanNode::serial_deadman_callback(const std_msgs::msg::String::ConstSharedPtr msg) {
    if (trim(msg->data).find("ALIVE") == 0) {
        last_alive_time_ = std::chrono::steady_clock::now();
        car_on_ = true;

        auto m = std_msgs::msg::Bool();
        m.data = true;
        alive_pub_->publish(m);

        RCLCPP_DEBUG(this->get_logger(), "Received ALIVE signal");
    }
}

void DeadmanNode::deadman_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    car_on_ = msg->data;
    // Feed the watchdog so a fresh deadman=true keeps the system armed.
    // A false simply disarms and is left to go stale, so it sticks until the next true.
    if (msg->data) {
        last_alive_time_ = std::chrono::steady_clock::now();
    }
    RCLCPP_DEBUG(this->get_logger(), "Deadman status updated: %s", car_on_ ? "ALIVE" : "DEAD");
}

void DeadmanNode::process_alive() {
    const auto current_time = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = current_time - last_alive_time_;
    const double elapsed_seconds = elapsed.count();

    // Watchdog only ever DISARMS. Re-arming is done exclusively by a real ALIVE
    // (/serial/deadman) or a fresh /deadman/alive=true.
    if (car_on_ && elapsed_seconds > alive_timeout_) {
        car_on_ = false;

        auto m = std_msgs::msg::Bool();
        m.data = false;
        alive_pub_->publish(m);

        RCLCPP_WARN(this->get_logger(), "No ALIVE received for %.3f seconds, disarming (deadman false)", elapsed_seconds);
    }
}

} // namespace deadman_pub

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<deadman_pub::DeadmanNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}