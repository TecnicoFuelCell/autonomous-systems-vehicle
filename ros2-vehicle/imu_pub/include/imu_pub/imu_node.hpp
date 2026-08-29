#ifndef IMU_PUB_IMU_NODE_HPP_
#define IMU_PUB_IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace imu_pub {

class ImuNode : public rclcpp::Node
{
public:
    ImuNode();

private:
    void process_acc(const std::string& data);
    void process_gyro(const std::string& data);
    void read_uart();

    int uart_fd_{-1};

    sensor_msgs::msg::Imu current_imu_msg_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace imu_pub

#endif // IMU_PUB_IMU_NODE_HPP_
