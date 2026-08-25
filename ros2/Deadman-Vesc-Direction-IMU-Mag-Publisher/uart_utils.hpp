#ifndef ACTUATINATOR_3000_UART_UTILS_HPP_
#define ACTUATINATOR_3000_UART_UTILS_HPP_

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace actuatinator_3000 {
namespace uart_utils {

struct UartFds {
    int reader_fd = -1;
    int writer_fd = -1;
    bool discovered = false;
};

UartFds discover_or_fallback(
    const rclcpp::Logger& logger,
    const std::string& fallback_reader_port = "/dev/ttyACM7",
    const std::string& fallback_writer_port = "/dev/ttyACM6",
    std::chrono::seconds timeout = std::chrono::seconds(5));

bool configure_uart(int fd, const rclcpp::Logger& logger);
std::string trim(const std::string& s);

}  // namespace uart_utils
}  // namespace actuatinator_3000

#endif  // ACTUATINATOR_3000_UART_UTILS_HPP_
