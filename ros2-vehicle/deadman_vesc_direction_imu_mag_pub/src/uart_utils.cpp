#include "deadman_vesc_direction_imu_mag_pub/uart_utils.hpp"

#include <fcntl.h>
#include <glob.h>
#include <termios.h>
#include <unistd.h>

#include <thread>
#include <vector>

namespace deadman_vesc_direction_imu_mag_pub {
namespace uart_utils {

std::string trim(const std::string& s) {
    auto start = s.find_first_not_of(" \t\n\r");
    auto end = s.find_last_not_of(" \t\n\r");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

namespace {

struct UartCandidate {
    std::string port;
    int fd = -1;
    std::string buffer;
};

/*
 * Function with usual lines that CANtoPC sends to Serial, to identify it.
 * @return: true if the line matches expected CANtoPC output, false otherwise.
*/
bool _is_cantopc_line(const std::string& line) {
    return line.find("[CANtoPC]") == 0 ||
           line.find("---------- CANtoPC.INO ----------") == 0 ||
           line.find("VESC:") == 0 ||
           line.find("Dir:") == 0 ||
           line.find("ALIVE") == 0 ||
           line.find("ACC:") == 0 ||
           line.find("GYRO:") == 0 ||
           line.find("MAG:") == 0;
}

/*
 * Function with usual lines that PCSender sends to Serial, to identify it.
 * @return: true if the line matches expected PCSender output, false otherwise.
*/
bool _is_pcsender_line(const std::string& line) {
    return line.find("[PCSender]") == 0 ||
           line.find("---------- PCSender.INO ----------") == 0 ||
           line.find("Updated Angle:") == 0 ||
           line.find("[PC Sender] Brake Current:") == 0 ||
           line.find("[PC Sender] Current:") == 0;
}

std::vector<std::string> list_tty_acm_ports() {
    std::vector<std::string> ports;
    glob_t glob_result{};
    if (glob("/dev/ttyACM*", 0, nullptr, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            ports.push_back(glob_result.gl_pathv[i]);
        }
    }
    globfree(&glob_result);
    return ports;
}

std::vector<std::string> read_available_lines(UartCandidate& candidate) {
    std::vector<std::string> lines;
    char c;
    while (read(candidate.fd, &c, 1) == 1) {
        if (c == '\n') {
            lines.push_back(candidate.buffer);
            candidate.buffer.clear();
        } else {
            candidate.buffer += c;
        }
    }
    return lines;
}

UartFds open_fallback_uart_devices(
    const rclcpp::Logger& logger,
    const std::string& fallback_reader_port,
    const std::string& fallback_writer_port) {
    UartFds fds;

    fds.reader_fd = open(fallback_reader_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fds.reader_fd >= 0) {
        configure_uart(fds.reader_fd, logger);
        RCLCPP_INFO(logger, "Fallback CANtoPC UART opened on %s", fallback_reader_port.c_str());
    }

    fds.writer_fd = open(fallback_writer_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fds.writer_fd >= 0) {
        configure_uart(fds.writer_fd, logger);
        RCLCPP_INFO(logger, "Fallback PCSender UART opened on %s", fallback_writer_port.c_str());
    }

    return fds;
}

UartFds discover_uart_devices(const rclcpp::Logger& logger, std::chrono::seconds timeout) {
    std::vector<std::string> ports = list_tty_acm_ports();
    if (ports.empty()) {
        RCLCPP_WARN(logger, "No /dev/ttyACM* ports found for UART role discovery");
        return UartFds{};
    }

    std::vector<UartCandidate> candidates;
    for (const auto& port : ports) {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
        if (fd < 0) {
            RCLCPP_WARN(logger, "Could not open %s during UART discovery", port.c_str());
            continue;
        }

        configure_uart(fd, logger);
        UartCandidate candidate;
        candidate.port = port;
        candidate.fd = fd;
        candidates.push_back(candidate);
        RCLCPP_INFO(logger, "Probing UART candidate %s", port.c_str());
    }

    if (candidates.empty()) {
        return UartFds{};
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto next_probe = std::chrono::steady_clock::now();
    int reader_index = -1;
    int writer_index = -1;

    while (std::chrono::steady_clock::now() < deadline &&
           (reader_index < 0 || writer_index < 0)) {
        const auto now_time = std::chrono::steady_clock::now();
        if (now_time >= next_probe) {
            for (auto& candidate : candidates) {
                write(candidate.fd, "Dir: 0\n", 7);
            }
            next_probe = now_time + std::chrono::milliseconds(250);
        }

        for (size_t i = 0; i < candidates.size(); ++i) {
            std::vector<std::string> lines = read_available_lines(candidates[i]);
            for (const auto& line_raw : lines) {
                const std::string line = trim(line_raw);
                if (_is_cantopc_line(line)) {
                    reader_index = static_cast<int>(i);
                    RCLCPP_INFO(logger, "Discovered CANtoPC on %s", candidates[i].port.c_str());
                } else if (_is_pcsender_line(line)) {
                    writer_index = static_cast<int>(i);
                    RCLCPP_INFO(logger, "Discovered PCSender on %s", candidates[i].port.c_str());
                } else {
                    RCLCPP_DEBUG(logger, "Discovery read from %s: %s",
                                 candidates[i].port.c_str(), line.c_str());
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (reader_index >= 0 && writer_index >= 0 && reader_index != writer_index) {
        UartFds fds;
        fds.reader_fd = candidates[reader_index].fd;
        fds.writer_fd = candidates[writer_index].fd;
        fds.discovered = true;

        for (size_t i = 0; i < candidates.size(); ++i) {
            if (static_cast<int>(i) != reader_index && static_cast<int>(i) != writer_index) {
                close(candidates[i].fd);
            }
        }
        return fds;
    }

    for (auto& candidate : candidates) {
        close(candidate.fd);
    }
    return UartFds{};
}

}  // namespace

bool configure_uart(int fd, const rclcpp::Logger& logger) {
    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        RCLCPP_ERROR(logger, "Error getting termios attributes for fd: %d", fd);
        return false;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(logger, "Error setting termios attributes for fd: %d", fd);
        return false;
    }

    RCLCPP_DEBUG(logger, "UART successfully configured for fd: %d", fd);
    return true;
}

UartFds discover_or_fallback(
    const rclcpp::Logger& logger,
    const std::string& fallback_reader_port,
    const std::string& fallback_writer_port,
    std::chrono::seconds timeout) {
    UartFds fds = discover_uart_devices(logger, timeout);
    if (fds.discovered) {
        return fds;
    }

    RCLCPP_WARN(logger,
                "UART role discovery timed out; falling back to %s as CANtoPC and %s as PCSender",
                fallback_reader_port.c_str(), fallback_writer_port.c_str());
    return open_fallback_uart_devices(logger, fallback_reader_port, fallback_writer_port);
}

}  // namespace uart_utils
}  // namespace deadman_vesc_direction_imu_mag_pub
