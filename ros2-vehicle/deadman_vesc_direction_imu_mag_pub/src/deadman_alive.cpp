#include "joystick_actuator.hpp"

void JoystickActuator::deadman_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    car_on = msg->data;
    // Feed the watchdog so a fresh deadman=true keeps the actuator armed in
    // actuator_only mode (where there is no serial ALIVE). A false simply
    // disarms and is left to go stale, so it sticks until the next true.
    if (msg->data) {
        last_alive_time_ = std::chrono::steady_clock::now();
    }
    RCLCPP_DEBUG(this->get_logger(), "Deadman status updated: %s", car_on ? "ALIVE" : "DEAD");
}

void JoystickActuator::process_alive() {
    const auto current_time = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = current_time - last_alive_time_;
    const double elapsed_seconds = elapsed.count();

    // Watchdog only ever DISARMS. Re-arming is done exclusively by a real ALIVE
    // event (process_data, serial) or a fresh /deadman/alive=true
    // (deadman_callback). The previous "re-arm when elapsed < 0.5" branch let a
    // recent ALIVE override an explicit deadman=false e-stop — removed.
    if (car_on && elapsed_seconds > 0.5) {
        car_on = false;

        auto m = std_msgs::msg::Bool();
        m.data = false;
        if (alive_pub_) {
            alive_pub_->publish(m);
        }

        RCLCPP_WARN(this->get_logger(), "No ALIVE received for %.3f seconds, disarming (deadman false)", elapsed_seconds);
    }
}
