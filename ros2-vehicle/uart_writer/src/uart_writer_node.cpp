#include "uart_writer/uart_writer_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <string>

namespace uart_writer {

UartWriterNode::UartWriterNode()
: Node("uart_writer_node"), uart_fd_(-1)
{
    std::string serial_port =
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM1");

    uart_fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open UART device for writing to PCSender: %s", serial_port.c_str());
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

    RCLCPP_INFO(this->get_logger(), "UART port opened for PCSender: %s", serial_port.c_str());

    movement_sub_ = this->create_subscription<car_msgs::msg::ImSpeed>(
        "/joystick_movement", 10, std::bind(&UartWriterNode::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "UART Writer Node initialized successfully.");
}

UartWriterNode::~UartWriterNode() {
    if (uart_fd_ >= 0) close(uart_fd_);
}

void UartWriterNode::joy_callback(const car_msgs::msg::ImSpeed::SharedPtr msg) {
    int joystick_data = msg->move;
    std::string joystick_side = msg->which;
    int joystick_steering = msg->analog;

    if (std::abs(joystick_steering) > 25) {
        joystick_steering += (joystick_steering < 0) ? 25 : -25;
        write_serial("Dir: " + std::to_string(joystick_steering));
    } else {
        write_serial("Dir: 0");
    }

    write_serial(joystick_side + ": " + std::to_string(joystick_data));
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
