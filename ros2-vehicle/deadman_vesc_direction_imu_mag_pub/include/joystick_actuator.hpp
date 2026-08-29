#ifndef JOYSTICK_ACTUATOR_HPP
#define JOYSTICK_ACTUATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <car_msgs/msg/im_speed.hpp>
#include <car_msgs/msg/dir.hpp>
#include <car_msgs/msg/vesc_data.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <unistd.h>
#include <string>
#include <cstring>
#include <sstream>
#include <cmath>
#include <vector>
#include <array>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "uart_utils.hpp"

using namespace std::chrono_literals;

// Returns ROS time; if use_sim_time is true but no /clock has arrived yet
// (e.g. bag without /clock), falls back to wall time so stamps stay non-zero.
// Single definition lives in deploy_joystick.cpp.
rclcpp::Time safe_now(rclcpp::Node* n);

class JoystickActuator : public rclcpp::Node
{
public:
    JoystickActuator();
    ~JoystickActuator();

private:
    void deadman_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void listener_callback(const car_msgs::msg::ImSpeed::SharedPtr msg);
    void process_data(const std::string& data);
    void process_vesc(const std::string& data);
    void process_dir(const std::string& data);
    void process_acc(const std::string& data);
    void process_gyro(const std::string& data);
    void process_mag(const std::string& data);
    void process_alive();
    void read_uart();
    void write_serial(const std::string& message);
    std::vector<std::string> split(const std::string& s, char delimiter);
    void load_mag_calibration();

    int uart_fd_writer;
    int uart_fd_reader;
    bool actuator_only_ = false;
    bool car_on = false;  // fail-safe: stay disarmed until a deadman/ALIVE arrives
    std::string read_buffer_;

    bool apply_mag_calib_ = true;
    std::array<double, 3> mag_offset_{{0.0, 0.0, 0.0}};
    std::array<std::array<double, 3>, 3> mag_matrix_{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}}
    }};

    sensor_msgs::msg::Imu current_imu_msg_;

    std::chrono::steady_clock::time_point last_alive_time_;
    rclcpp::TimerBase::SharedPtr alive_timer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<car_msgs::msg::Dir>::SharedPtr dir_publisher_;
    rclcpp::Publisher<car_msgs::msg::VescData>::SharedPtr vesc_dir_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;

    rclcpp::Subscription<car_msgs::msg::ImSpeed>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alive_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
};

#endif  // JOYSTICK_ACTUATOR_HPP
