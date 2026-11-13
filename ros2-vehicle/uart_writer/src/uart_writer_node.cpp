#include "uart_writer/uart_writer_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace uart_writer {

namespace {

// Joystick steering values within +/-25 are treated as centred.
constexpr int kSteeringDeadzone = 25;
// joystick_reader scales the triggers to 0-50, the R2/L2 range PCToCAN expects.
constexpr int kJoystickMaxMove = 50;

double seconds_between(std::chrono::steady_clock::time_point from,
                       std::chrono::steady_clock::time_point to)
{
    return std::chrono::duration<double>(to - from).count();
}

} // namespace

UartWriterNode::UartWriterNode()
: Node("uart_writer_node"), uart_fd_(-1)
{
    std::string serial_port =
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM1");

    uart_fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open UART device for writing to PCToCAN: %s", serial_port.c_str());
        rclcpp::shutdown();
        return;
    }

    struct termios tty{};
    if (tcgetattr(uart_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting termios attributes");
        rclcpp::shutdown();
        return;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting termios attributes");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "UART port opened for PCToCAN: %s", serial_port.c_str());

    mpc_max_rpm_ = this->declare_parameter<int>("mpc_max_rpm", 338);
    if (mpc_max_rpm_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "mpc_max_rpm must be positive, using 338");
        mpc_max_rpm_ = 338;
    }
    mpc_timeout_ = this->declare_parameter<double>("mpc_timeout", 0.5);
    joystick_override_hold_ = this->declare_parameter<double>("joystick_override_hold", 1.0);

    // Start both sources stale, so neither is in control until it publishes.
    last_mpc_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    last_joystick_active_time_ = last_mpc_time_;

    const std::string movement_topic =
        this->declare_parameter<std::string>(
            "joystick_movement_topic", "/vehicle_internal/joystick/commands");
    movement_sub_ = this->create_subscription<car_msgs::msg::ImSpeed>(
        movement_topic, 10,
        std::bind(&UartWriterNode::joy_callback, this, std::placeholders::_1));

    const std::string mpc_movement_topic =
        this->declare_parameter<std::string>("mpc_movement_topic", "/mpc");
    mpc_sub_ = this->create_subscription<car_msgs::msg::ImSpeed>(
        mpc_movement_topic, 1,
        std::bind(&UartWriterNode::mpc_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "UART Writer Node initialized successfully.");
}

UartWriterNode::~UartWriterNode() {
    if (uart_fd_ >= 0) close(uart_fd_);
}

void UartWriterNode::joy_callback(const car_msgs::msg::ImSpeed::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    const bool joystick_active =
        msg->move > 0 || std::abs(msg->analog) > kSteeringDeadzone;
    if (joystick_active) {
        last_joystick_active_time_ = now;
    }

    // joy_node keeps repeating idle messages (Dir: 0, R2: 0). Drop them while the
    // MPC is driving, otherwise the two sources alternate on the serial line.
    const bool joystick_in_control = joystick_active ||
        seconds_between(last_joystick_active_time_, now) < joystick_override_hold_;
    const bool mpc_alive = seconds_between(last_mpc_time_, now) < mpc_timeout_;
    if (!joystick_in_control && mpc_alive) {
        return;
    }

    int joystick_data = msg->move;
    std::string joystick_side = msg->which;
    int joystick_steering = msg->analog;

    if (std::abs(joystick_steering) > kSteeringDeadzone) {
        joystick_steering += (joystick_steering < 0) ? kSteeringDeadzone : -kSteeringDeadzone;
        write_serial("Dir: " + std::to_string(joystick_steering));
    } else {
        write_serial("Dir: 0");
    }

    write_serial(joystick_side + ": " + std::to_string(joystick_data));
}

void UartWriterNode::mpc_callback(const car_msgs::msg::ImSpeed::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    last_mpc_time_ = now;

    // The safety driver always wins: any joystick input takes over and keeps
    // control until joystick_override_hold seconds after the last input.
    if (seconds_between(last_joystick_active_time_, now) < joystick_override_hold_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Joystick override active, ignoring MPC commands");
        return;
    }

    // analog is already in steering actuator units, so the joystick deadzone does
    // not apply. PCToCAN clamps it to +/-250.
    write_serial("Dir: " + std::to_string(msg->analog));

    // move is a forward motor RPM target (the MPC never reverses or brakes), but
    // PCToCAN reads R2 as a drive current on the joystick scale. Map
    // 0..mpc_max_rpm onto 0..kJoystickMaxMove and cap it, so the MPC can never
    // request more current than the joystick. This is open loop, not speed control.
    const int rpm = std::max(0, static_cast<int>(msg->move));
    const int move = std::min(kJoystickMaxMove, rpm * kJoystickMaxMove / mpc_max_rpm_);
    write_serial("R2: " + std::to_string(move));
}

void UartWriterNode::write_serial(const std::string& message) {
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

} // namespace uart_writer

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uart_writer::UartWriterNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
