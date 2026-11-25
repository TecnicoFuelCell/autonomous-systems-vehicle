#include "imu_pub/imu_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <functional>
#include <exception>
#include <sstream>

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace imu_pub {

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

ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: Node("imu_node", options)
{
    apply_log_level(*this);

    std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/imu_data");
    std::string frame_id = this->declare_parameter<std::string>("frame_id", "imu_link");

    current_imu_msg_.header.frame_id = frame_id;

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);
    std::string serial_acc_topic =
        this->declare_parameter<std::string>("serial_acc_topic", "/vehicle_internal/serial/acc");
    std::string serial_gyro_topic =
        this->declare_parameter<std::string>("serial_gyro_topic", "/vehicle_internal/serial/gyro");
    acc_sub_ = this->create_subscription<std_msgs::msg::String>(serial_acc_topic, 10,
        std::bind(&ImuNode::process_acc, this, std::placeholders::_1));
    gyro_sub_ = this->create_subscription<std_msgs::msg::String>(serial_gyro_topic, 10,
        std::bind(&ImuNode::process_gyro, this, std::placeholders::_1));
}

void ImuNode::process_acc(const std_msgs::msg::String::ConstSharedPtr msg) {
    std::vector<std::string> parts = split(msg->data, ',');
    if (parts.size() >= 3) {
        try {
            current_imu_msg_.linear_acceleration.x = std::stof(parts[1]) * 0.00980665f;
            current_imu_msg_.linear_acceleration.y = std::stof(parts[2]) * 0.00980665f;
            current_imu_msg_.linear_acceleration.z = std::stof(parts[0]) * 0.00980665f;

            if (std::abs(current_imu_msg_.linear_acceleration.x) < 1e-5 &&
                std::abs(current_imu_msg_.linear_acceleration.y) < 1e-5 &&
                std::abs(current_imu_msg_.linear_acceleration.z) < 1e-5) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "IMU ALERT: All Accelerometer values are exactly zero! Check IMU connection.");
            }

            current_imu_msg_.orientation.x = 0.0;
            current_imu_msg_.orientation.y = 0.0;
            current_imu_msg_.orientation.z = 0.0;
            current_imu_msg_.orientation.w = 0.0;

            for (int i = 0; i < 9; i++) {
                current_imu_msg_.orientation_covariance[i] = 0.0;
                current_imu_msg_.linear_acceleration_covariance[i] = 0.0;
            }

            RCLCPP_DEBUG(this->get_logger(), "Parsed ACC (m/s^2): x=%.3f, y=%.3f, z=%.3f",
                         current_imu_msg_.linear_acceleration.x,
                         current_imu_msg_.linear_acceleration.y,
                         current_imu_msg_.linear_acceleration.z);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "ACC parse exception: %s. Payload: %s", e.what(), msg->data.c_str());
        }
    } else {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "ACC parse error: Insufficient data pieces (%zu)", parts.size());
    }
}

void ImuNode::process_gyro(const std_msgs::msg::String::ConstSharedPtr msg) {
    std::vector<std::string> parts = split(msg->data, ',');
    if (parts.size() >= 3) {
        try {
            constexpr float DEG_TO_RAD = 0.01745329251f;

            current_imu_msg_.angular_velocity.x = std::stof(parts[0]) * DEG_TO_RAD;
            current_imu_msg_.angular_velocity.y = std::stof(parts[1]) * DEG_TO_RAD;
            current_imu_msg_.angular_velocity.z = std::stof(parts[2]) * DEG_TO_RAD;

            if (std::abs(current_imu_msg_.angular_velocity.x) < 1e-5 &&
                std::abs(current_imu_msg_.angular_velocity.y) < 1e-5 &&
                std::abs(current_imu_msg_.angular_velocity.z) < 1e-5) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "IMU ALERT: All Gyroscope values are exactly zero! Check IMU connection.");
            }

            current_imu_msg_.orientation.x = 0.0;
            current_imu_msg_.orientation.y = 0.0;
            current_imu_msg_.orientation.z = 0.0;
            current_imu_msg_.orientation.w = 0.0;

            for (int i = 0; i < 9; i++) {
                current_imu_msg_.orientation_covariance[i] = 0.0;
                current_imu_msg_.angular_velocity_covariance[i] = 0.0;
            }

            current_imu_msg_.header.stamp = safe_now(this);

            RCLCPP_DEBUG(this->get_logger(), "Parsed GYRO (rad/s): x=%.3f, y=%.3f, z=%.3f",
                         current_imu_msg_.angular_velocity.x,
                         current_imu_msg_.angular_velocity.y,
                         current_imu_msg_.angular_velocity.z);

            publisher_->publish(current_imu_msg_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "GYRO parse exception: %s. Payload: %s", e.what(), msg->data.c_str());
        }
    } else {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "GYRO parse error: Insufficient data pieces (%zu)", parts.size());
    }
}

} // namespace imu_pub

RCLCPP_COMPONENTS_REGISTER_NODE(imu_pub::ImuNode)
