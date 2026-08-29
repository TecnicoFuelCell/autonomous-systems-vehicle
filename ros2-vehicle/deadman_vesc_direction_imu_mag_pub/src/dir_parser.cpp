#include "joystick_actuator.hpp"

/*
 * Handler for Dir data messages.
 * Parses the payload and publishes a ROS message to /dir_data.
 * @param1 data (string) - the raw line read from UART
 */
void JoystickActuator::process_dir(const std::string& data) {
    try {
        int value = std::stoi(deadman_vesc_direction_imu_mag_pub::uart_utils::trim(data));
        auto msg = car_msgs::msg::Dir();
        msg.header.stamp = safe_now(this);
        msg.dir = value;
        
        RCLCPP_DEBUG(this->get_logger(), "Parsed Dir: %d", value);
        if (dir_publisher_) {
            dir_publisher_->publish(msg);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Dir parse exception: %s. Payload: %s", e.what(), data.c_str());
    }
}
