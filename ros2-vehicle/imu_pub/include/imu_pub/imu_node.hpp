#ifndef IMU_PUB_IMU_NODE_HPP_
#define IMU_PUB_IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

namespace imu_pub {

class ImuNode : public rclcpp::Node
{
public:
    ImuNode();

private:
    void process_acc(const std_msgs::msg::String::ConstSharedPtr msg);
    void process_gyro(const std_msgs::msg::String::ConstSharedPtr msg);

    sensor_msgs::msg::Imu current_imu_msg_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr acc_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gyro_sub_;
};

} // namespace imu_pub

#endif // IMU_PUB_IMU_NODE_HPP_
