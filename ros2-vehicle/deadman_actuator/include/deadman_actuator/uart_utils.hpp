#ifndef DEADMAN_ACTUATOR_UART_UTILS_HPP_
#define DEADMAN_ACTUATOR_UART_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fcntl.h>
#include <filesystem>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace deadman_actuator {
namespace uart_utils {

inline std::string trim(const std::string& str) {
    std::string s = str;
    s.erase(s.begin(), std::find_if_not(s.begin(), s.end(),
        [](unsigned char ch) { return std::isspace(ch); }));
    s.erase(std::find_if_not(s.rbegin(), s.rend(),
        [](unsigned char ch) { return std::isspace(ch); }).base(), s.end());
    return s;
}

inline std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::istringstream stream(s);
    std::string token;
    while (std::getline(stream, token, delimiter)) {
        tokens.push_back(trim(token));
    }
    return tokens;
}

inline int discover_or_fallback(
    const rclcpp::Logger& logger,
    const std::string& preferred_port = "/dev/ttyUSB0",
    const std::string& fallback_port = "/dev/ttyACM0",
    std::chrono::seconds timeout = std::chrono::seconds(5))
{
    std::string selected_port = fallback_port;
    auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(logger, "Searching for UART port: %s", preferred_port.c_str());

    while (rclcpp::ok()) {
        if (std::filesystem::exists(preferred_port)) {
            selected_port = preferred_port;
            break;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time);

        if (elapsed >= timeout) {
            RCLCPP_WARN(logger, "Timeout for %s. Using fallback: %s",
                        preferred_port.c_str(), fallback_port.c_str());
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    int fd = open(selected_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        RCLCPP_ERROR(logger, "Failed to open UART port: %s", selected_port.c_str());
    } else {
        RCLCPP_INFO(logger, "UART port opened (fd: %d): %s", fd, selected_port.c_str());
    }
    return fd;
}

} // namespace uart_utils
} // namespace deadman_actuator

#endif // DEADMAN_ACTUATOR_UART_UTILS_HPP_
