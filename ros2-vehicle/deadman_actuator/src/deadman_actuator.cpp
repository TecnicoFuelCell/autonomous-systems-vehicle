#include "deadman_actuator/deadman_actuator.hpp"

#include <cmath>
#include <string>
#include <functional>

#include <unistd.h>

#include "deadman_actuator/uart_utils.hpp"

using namespace std::chrono_literals;

namespace deadman_actuator {

DeadmanActuatorNode::DeadmanActuatorNode()
: Node("deadman_actuator"), uart_fd_(-1)
{
    std::string serial_port =
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");

    uart_fd_ = uart_utils::discover_or_fallback(this->get_logger(), serial_port);
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UART device for deadman/serial IO.");
        rclcpp::shutdown();
        return;
    }

    deadman_sub_ = this->create_subscription<std_msgs::msg::Bool>("/deadman/alive", 10,
        std::bind(&DeadmanActuatorNode::deadman_callback, this, std::placeholders::_1));
    subscription_ = this->create_subscription<car_msgs::msg::ImSpeed>("/joystick_movement", 10,
        std::bind(&DeadmanActuatorNode::listener_callback, this, std::placeholders::_1));

    // Start the deadman stale so the actuator stays disarmed until a real
    // ALIVE (serial) or a fresh /deadman/alive=true actually arrives.
    last_alive_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);

    alive_pub_ = this->create_publisher<std_msgs::msg::Bool>("/deadman/alive", 1);

    // Raw line passthrough topics consumed by the per-sensor decode nodes.
    vesc_pub_ = this->create_publisher<std_msgs::msg::String>("/serial/vesc", 10);
    dir_pub_ = this->create_publisher<std_msgs::msg::String>("/serial/dir", 10);
    acc_pub_ = this->create_publisher<std_msgs::msg::String>("/serial/acc", 10);
    gyro_pub_ = this->create_publisher<std_msgs::msg::String>("/serial/gyro", 10);
    mag_pub_ = this->create_publisher<std_msgs::msg::String>("/serial/mag", 10);

    read_timer_ = this->create_wall_timer(5ms, std::bind(&DeadmanActuatorNode::read_uart, this));
    // Deadman watchdog only ever DISARMS; arming is done solely by a real
    // ALIVE/deadman-true event.
    alive_timer_ = this->create_wall_timer(50ms, std::bind(&DeadmanActuatorNode::process_alive, this));

    RCLCPP_INFO(this->get_logger(), "Deadman Actuator Node initialized successfully.");
}

DeadmanActuatorNode::~DeadmanActuatorNode() {
    if (uart_fd_ >= 0) close(uart_fd_);
}

void DeadmanActuatorNode::deadman_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    car_on_ = msg->data;
    // Feed the watchdog so a fresh deadman=true keeps the actuator armed.
    // A false simply disarms and is left to go stale, so it sticks until the next true.
    if (msg->data) {
        last_alive_time_ = std::chrono::steady_clock::now();
    }
    RCLCPP_DEBUG(this->get_logger(), "Deadman status updated: %s", car_on_ ? "ALIVE" : "DEAD");
}

void DeadmanActuatorNode::listener_callback(const car_msgs::msg::ImSpeed::SharedPtr msg) {
    int joystick_data = msg->move;
    std::string joystick_side = msg->which;
    int joystick_steering = msg->analog;

    if (!car_on_) {
        RCLCPP_DEBUG(this->get_logger(), "car NOT alive");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Joystick RX -> side: %s | move: %d | analog: %d",
                 joystick_side.c_str(), joystick_data, joystick_steering);

    if (std::abs(joystick_steering) > 25) {
        joystick_steering += (joystick_steering < 0) ? 25 : -25;
        write_serial("Dir: " + std::to_string(joystick_steering));
    } else {
        write_serial("Dir: 0");
    }
    write_serial(joystick_side + ": " + std::to_string(joystick_data));
}

void DeadmanActuatorNode::process_alive() {
    const auto current_time = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = current_time - last_alive_time_;
    const double elapsed_seconds = elapsed.count();

    // Watchdog only ever DISARMS. Re-arming is done exclusively by a real ALIVE
    // event (read_uart, serial) or a fresh /deadman/alive=true (deadman_callback).
    if (car_on_ && elapsed_seconds > 0.5) {
        car_on_ = false;

        auto m = std_msgs::msg::Bool();
        m.data = false;
        alive_pub_->publish(m);

        RCLCPP_WARN(this->get_logger(), "No ALIVE received for %.3f seconds, disarming (deadman false)", elapsed_seconds);
    }
}

void DeadmanActuatorNode::read_uart() {
    if (uart_fd_ < 0) return;

    char buf[256];
    ssize_t n;
    while ((n = read(uart_fd_, buf, sizeof(buf))) > 0) {
        line_buffer_.append(buf, buf + n);
        size_t pos;
        while ((pos = line_buffer_.find('\n')) != std::string::npos) {
            std::string line = line_buffer_.substr(0, pos);
            line_buffer_.erase(0, pos + 1);
            handle_line(line);
        }
    }
}

void DeadmanActuatorNode::handle_line(const std::string& line) {
    std::string trimmed = uart_utils::trim(line);
    if (trimmed.empty()) return;

    std_msgs::msg::String msg;
    if (trimmed.find("ALIVE") == 0) {
        last_alive_time_ = std::chrono::steady_clock::now();
        car_on_ = true;
        auto m = std_msgs::msg::Bool();
        m.data = true;
        alive_pub_->publish(m);
        RCLCPP_DEBUG(this->get_logger(), "Received ALIVE signal");
        return;
    } else if (trimmed.find("VESC:") == 0) {
        msg.data = trimmed.substr(5);
        vesc_pub_->publish(msg);
    } else if (trimmed.find("Dir:") == 0) {
        msg.data = trimmed.substr(4);
        dir_pub_->publish(msg);
    } else if (trimmed.find("ACC:") == 0) {
        msg.data = trimmed.substr(4);
        acc_pub_->publish(msg);
    } else if (trimmed.find("GYRO:") == 0) {
        msg.data = trimmed.substr(5);
        gyro_pub_->publish(msg);
    } else if (trimmed.find("MAG:") == 0) {
        msg.data = trimmed.substr(4);
        mag_pub_->publish(msg);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Unknown serial line dropped: %s", trimmed.c_str());
    }
}

void DeadmanActuatorNode::write_serial(const std::string& message) {
    if (uart_fd_ >= 0) {
        std::string msg_with_newline = message + "\n";
        ssize_t bytes_written = write(uart_fd_, msg_with_newline.c_str(), msg_with_newline.size());
        if (bytes_written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port!");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Serial Write: %s", message.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Attempted to write to serial, but fd is closed.");
    }
}

} // namespace deadman_actuator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<deadman_actuator::DeadmanActuatorNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
