#include "joystick_actuator.hpp"

/*
 * Handler for VESC data messages.
 * Parses the payload and publishes a ROS message to /vesc_data.
 * @param1 data (string) - the raw line read from UART
 */
void JoystickActuator::process_vesc(const std::string& data) {
    std::vector<std::string> parts = split(data, ',');
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
        if (vesc_dir_publisher_) {
            vesc_dir_publisher_->publish(msg);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "VESC parse exception: %s. Payload: %s", e.what(), data.c_str());
    }
}

/*
 * Splits a string by a given delimiter and trims whitespace from each resulting token.
 *  @param1 s (string), param2 delimiter (char)
 *  @return vector of trimmed tokens
*/
std::vector<std::string> JoystickActuator::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(deadman_vesc_direction_imu_mag_pub::uart_utils::trim(token));
    }
    return tokens;
}
