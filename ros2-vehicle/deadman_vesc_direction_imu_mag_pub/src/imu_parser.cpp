#include "joystick_actuator.hpp"

/*
 * Handler for ACC data messages. Parses payload and stores 
 * acceleration info on current_imu_msg_ that will be published when gyro data arrives.
 * @param1 data (string) - the raw line read from UART
 */
void JoystickActuator::process_acc(const std::string& data) {
    std::vector<std::string> parts = split(data, ',');
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

/*
 * Handler for gyroscope messages. Parses payload, converts from deg/s to rad/s,
 * and publishes ROS message to /imu_data (acc data in m/s^2 and gyro data in rad/s).
 * @param1 data (string) - the raw line read from UART
 */
void JoystickActuator::process_gyro(const std::string& data) {
    std::vector<std::string> parts = split(data, ',');
    if (parts.size() >= 3) {
        try {
            // Fator de conversão: de graus/s para rad/s (pi / 180)
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

            if (imu_publisher_) {
                imu_publisher_->publish(current_imu_msg_);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "GYRO parse exception: %s. Payload: %s", e.what(), data.c_str());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "GYRO parse error: Insufficient data pieces (%zu)", parts.size());
    }
}
