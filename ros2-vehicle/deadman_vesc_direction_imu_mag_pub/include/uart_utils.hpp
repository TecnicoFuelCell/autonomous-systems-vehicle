#ifndef UART_UTILS_HPP_
#define UART_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

namespace deadman_vesc_direction_imu_mag_pub {
namespace uart_utils {

struct UartFDs {
    int reader_fd{-1};
    int writer_fd{-1};
};

inline std::string trim(const std::string& str) {
    auto start = str.begin();
    while (start != str.end() && std::isspace(static_cast<unsigned char>(*start))) {
        start++;
    }

    auto end = str.end();
    do {
        end--;
    } while (std::distance(start, end) > 0 && std::isspace(static_cast<unsigned char>(*end)));

    return std::string(start, end + 1);
}

inline UartFDs discover_or_fallback(
    const rclcpp::Logger& logger,
    const std::string& preferred_port = "/dev/ttyUSB0",
    const std::string& fallback_port = "/dev/ttyACM0",
    std::chrono::seconds timeout = std::chrono::seconds(5))
{
    std::string selected_port = fallback_port;
    auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(logger, "A procurar pela porta UART: %s", preferred_port.c_str());

    while (rclcpp::ok()) {
        if (std::filesystem::exists(preferred_port)) {
            selected_port = preferred_port;
            break;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time);

        if (elapsed >= timeout) {
            RCLCPP_WARN(logger, "Timeout para %s. A usar fallback: %s",
                        preferred_port.c_str(), fallback_port.c_str());
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Abrir a porta UART em modo Read/Write
    int fd = open(selected_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        RCLCPP_ERROR(logger, "Falha ao abrir a porta UART: %s", selected_port.c_str());
    } else {
        RCLCPP_INFO(logger, "Porta UART aberta com sucesso (fd: %d): %s", fd, selected_port.c_str());
    }

    UartFDs fds;
    fds.reader_fd = fd;
    fds.writer_fd = fd;
    return fds;
}

} // namespace uart_utils
} // namespace deadman_vesc_direction_imu_mag_pub

#endif // UART_UTILS_HPP_
