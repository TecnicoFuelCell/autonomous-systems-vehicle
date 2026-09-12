#include "uart_reader/uart_node.hpp"

#include <unistd.h>

#include "uart_reader/uart_utils.hpp"

using namespace std::chrono_literals;

namespace uart_reader {

UartNode::UartNode()
: Node("uart_node"), uart_fd_(-1)
{
    std::string serial_port =
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");

    uart_fd_ = uart_utils::discover_or_fallback(this->get_logger(), serial_port);
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UART device for reading from CANToPC.");
        rclcpp::shutdown();
        return;
    }

    // Raw line passthrough topics consumed by the per-sensor decode nodes.
    vesc_pub_ = this->create_publisher<std_msgs::msg::String>("/vehicle_internal/serial/vesc", 10);
    dir_pub_ = this->create_publisher<std_msgs::msg::String>("/vehicle_internal/serial/dir", 10);
    acc_pub_ = this->create_publisher<std_msgs::msg::String>("/vehicle_internal/serial/acc", 10);
    gyro_pub_ = this->create_publisher<std_msgs::msg::String>("/vehicle_internal/serial/gyro", 10);
    mag_pub_ = this->create_publisher<std_msgs::msg::String>("/vehicle_internal/serial/mag", 10);
    deadman_pub_ = this->create_publisher<std_msgs::msg::String>("/vehicle_internal/serial/deadman", 10);

    read_timer_ = this->create_wall_timer(5ms, std::bind(&UartNode::read_uart, this));

    RCLCPP_INFO(this->get_logger(), "UART Reader Node initialized successfully.");
}

UartNode::~UartNode() {
    if (uart_fd_ >= 0) close(uart_fd_);
}

void UartNode::read_uart() {
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

void UartNode::handle_line(const std::string& line) {
    std::string trimmed = uart_utils::trim(line);
    if (trimmed.empty()) return;

    std_msgs::msg::String msg;
    if (trimmed.find("ALIVE") == 0) {
        msg.data = trimmed;
        deadman_pub_->publish(msg);
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

} // namespace uart_reader

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uart_reader::UartNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}