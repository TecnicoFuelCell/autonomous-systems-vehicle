#include "mag_pub/mag_node.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <exception>

#include "mag_pub/uart_utils.hpp"

using namespace std::chrono_literals;

namespace mag_pub {

rclcpp::Time safe_now(rclcpp::Node* n) {
    rclcpp::Time t = n->get_clock()->now();
    if (t.nanoseconds() != 0) return t;
    static rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    return wall_clock.now();
}

MagNode::MagNode()
: Node("mag_node"), uart_fd_(-1)
{
    std::string mag_topic = this->declare_parameter<std::string>("mag_topic", "/mag_data");
    std::string frame_id = this->declare_parameter<std::string>("frame_id", "imu_link");

    load_mag_calibration();

    uart_fd_ = uart_utils::discover_or_fallback(this->get_logger());
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UART device for reading magnetometer data.");
        rclcpp::shutdown();
        return;
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 1);
    timer_ = this->create_wall_timer(5ms, std::bind(&MagNode::read_uart, this));
}

void MagNode::load_mag_calibration() {
    apply_mag_calib_ = this->declare_parameter<bool>("apply_mag_calib", true);
    const std::string path =
        this->declare_parameter<std::string>("mag_calib_path", "config/mag_calib.yaml");

    mag_offset_ = {{0.0, 0.0, 0.0}};
    mag_matrix_ = {{ {{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}} }};

    if (!apply_mag_calib_) {
        RCLCPP_WARN(this->get_logger(),
            "apply_mag_calib=false: publishing raw magnetometer (no offset/matrix). "
            "Use this mode to record a calibration bag.");
        return;
    }

    try {
        const YAML::Node cal = YAML::LoadFile(path)["mag_calibration"];
        if (!cal) throw std::runtime_error("missing 'mag_calibration' key");

        const YAML::Node off = cal["hard_iron_offset"];
        const YAML::Node mat = cal["soft_iron_matrix"];
        for (int i = 0; i < 3; ++i) {
            // Offsets are stored in microtesla but runtime math is tesla.
            mag_offset_[i] = off[i].as<double>() * 1e-6;
            for (int j = 0; j < 3; ++j) {
                mag_matrix_[i][j] = mat[i][j].as<double>();
            }
        }
        RCLCPP_INFO(this->get_logger(),
            "Mag calibration loaded from '%s' (mode=%s).",
            path.c_str(),
            cal["mode"] ? cal["mode"].as<std::string>().c_str() : "?");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
            "Could not load mag calibration from '%s' (%s). Publishing raw.",
            path.c_str(), e.what());
        mag_offset_ = {{0.0, 0.0, 0.0}};
        mag_matrix_ = {{ {{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}} }};
    }
}

void MagNode::read_uart() {
    if (uart_fd_ < 0) return;

    static std::string line_buffer;
    char c;
    while (read(uart_fd_, &c, 1) == 1) {
        if (c == '\n') {
            std::string line = uart_utils::trim(line_buffer);
            if (line.find("MAG:") == 0) {
                std::string data = line.substr(4);
                std::vector<std::string> parts = uart_utils::split(data, ',');
                if (parts.size() >= 3) {
                    try {
                        auto mag_msg = sensor_msgs::msg::MagneticField();
                        mag_msg.header.stamp = safe_now(this);
                        mag_msg.header.frame_id = "imu_link";

                        constexpr float UT_TO_TESLA = 1e-6f;

                        // Raw reading converted to tesla (sensor_msgs unit). With
                        // apply_mag_calib=false the offset is 0 / identity so raw passes through.
                        double raw_x = std::stof(parts[0]) * UT_TO_TESLA;
                        double raw_y = std::stof(parts[1]) * UT_TO_TESLA;
                        double raw_z = std::stof(parts[2]) * UT_TO_TESLA;

                        if (std::abs(raw_x) < 1e-10 &&
                            std::abs(raw_y) < 1e-10 &&
                            std::abs(raw_z) < 1e-10) {
                            RCLCPP_ERROR(this->get_logger(), "IMU ALERT: All Magnetometer values are exactly zero! Check IMU connection.");
                        }

                        RCLCPP_DEBUG(this->get_logger(), "Raw MAG (T): x=%.9f, y=%.9f, z=%.9f", raw_x, raw_y, raw_z);

                        // calib = matrix * (raw - offset)
                        double cx = raw_x - mag_offset_[0];
                        double cy = raw_y - mag_offset_[1];
                        double cz = raw_z - mag_offset_[2];

                        mag_msg.magnetic_field.x = mag_matrix_[0][0]*cx + mag_matrix_[0][1]*cy + mag_matrix_[0][2]*cz;
                        mag_msg.magnetic_field.y = mag_matrix_[1][0]*cx + mag_matrix_[1][1]*cy + mag_matrix_[1][2]*cz;
                        mag_msg.magnetic_field.z = mag_matrix_[2][0]*cx + mag_matrix_[2][1]*cy + mag_matrix_[2][2]*cz;

                        for (int i = 0; i < 9; i++) {
                            mag_msg.magnetic_field_covariance[i] = 0.0;
                        }

                        RCLCPP_DEBUG(this->get_logger(), "Parsed MAG (T): x=%.9f, y=%.9f, z=%.9f",
                                     mag_msg.magnetic_field.x,
                                     mag_msg.magnetic_field.y,
                                     mag_msg.magnetic_field.z);

                        publisher_->publish(mag_msg);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "MAG parse exception: %s. Payload: %s", e.what(), data.c_str());
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "MAG parse error: Insufficient data pieces (%zu)", parts.size());
                }
            }
            line_buffer.clear();
        } else {
            line_buffer += c;
        }
    }
}

} // namespace mag_pub

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mag_pub::MagNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
