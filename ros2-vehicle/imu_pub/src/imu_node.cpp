#include "imu_pub/imu_node.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <exception>

#include "imu_pub/uart_utils.hpp"

using namespace std::chrono_literals;

namespace imu_pub {

rclcpp::Time safe_now(rclcpp::Node* n) {
    rclcpp::Time t = n->get_clock()->now();
    if (t.nanoseconds() != 0) return t;
    static rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    return wall_clock.now();
}

ImuNode::ImuNode()
: Node("imu_node"), uart_fd_(-1)
{
    std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/imu_data");
    std::string frame_id = this->declare_parameter<std::string>("frame_id", "imu_link");

    uart_fd_ = uart_utils::discover_or_fallback(this->get_logger());
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UART device for reading IMU data.");
        rclcpp::shutdown();
        return;
    }

    current_imu_msg_.header.frame_id = frame_id;

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);
    timer_ = this->create_wall_timer(5ms, std::bind(&ImuNode::read_uart, this));
}

void ImuNode::process_acc(const std::string& data) {
    std::vector<std::string> parts = uart_utils::split(data, ',');
    if (parts.size() >= 3) {
        try {
            current_imu_msg_.linear_acceleration.x = std::stof(parts[1]) * 0.00980665f;
            current_imu_msg_.linear_acceleration.y = std::stof(parts[2]) * 0.00980665f;
            current_imu_msg_.linear_acceleration.z = std::stof(parts[0]) * 0.00980665f;

            if (std::abs(current_imu_msg_.linear_acceleration.x) < 1e-5 &&
                std::abs(current_imu_msg_.linear_acceleration.y) < 1e-5 &&
                std::abs(current_imu_msg_.linear_acceleration.z) < 1e-5) {
                RCLCPP_ERROR(this->get_logger(), "IMU ALERT: All Accelerometer values are exactly zero! Check IMU connection.");
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
            RCLCPP_ERROR(this->get_logger(), "ACC parse exception: %s. Payload: %s", e.what(), data.c_str());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "ACC parse error: Insufficient data pieces (%zu)", parts.size());
    }
}

void ImuNode::process_gyro(const std::string& data) {
    std::vector<std::string> parts = uart_utils::split(data, ',');
    if (parts.size() >= 3) {
        try {
            constexpr float DEG_TO_RAD = 0.01745329251f;

            current_imu_msg_.angular_velocity.x = std::stof(parts[0]) * DEG_TO_RAD;
            current_imu_msg_.angular_velocity.y = std::stof(parts[1]) * DEG_TO_RAD;
            current_imu_msg_.angular_velocity.z = std::stof(parts[2]) * DEG_TO_RAD;

            if (std::abs(current_imu_msg_.angular_velocity.x) < 1e-5 &&
                std::abs(current_imu_msg_.angular_velocity.y) < 1e-5 &&
                std::abs(current_imu_msg_.angular_velocity.z) < 1e-5) {
                RCLCPP_ERROR(this->get_logger(), "IMU ALERT: All Gyroscope values are exactly zero! Check IMU connection.");
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
            RCLCPP_ERROR(this->get_logger(), "GYRO parse exception: %s. Payload: %s", e.what(), data.c_str());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "GYRO parse error: Insufficient data pieces (%zu)", parts.size());
    }
}

void ImuNode::read_uart() {
    if (uart_fd_ < 0) return;

    static std::string line_buffer;
    char c;
    while (read(uart_fd_, &c, 1) == 1) {
        if (c == '\n') {
            std::string line = uart_utils::trim(line_buffer);
            if (line.find("ACC:") == 0) {
                process_acc(line.substr(4));
            } else if (line.find("GYRO:") == 0) {
                process_gyro(line.substr(5));
            }
            line_buffer.clear();
        } else {
            line_buffer += c;
        }
    }
}

} // namespace imu_pub

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<imu_pub::ImuNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
