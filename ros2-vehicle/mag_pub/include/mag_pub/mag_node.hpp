#ifndef MAG_PUB_MAG_NODE_HPP_
#define MAG_PUB_MAG_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <string>

#include <yaml-cpp/yaml.h>

namespace mag_pub {

class MagNode : public rclcpp::Node
{
public:
    MagNode();

private:
    void load_mag_calibration();
    void process(const std_msgs::msg::String::ConstSharedPtr msg);

    bool apply_mag_calib_{true};
    std::string frame_id_;
    std::array<double, 3> mag_offset_{{0.0, 0.0, 0.0}};
    std::array<std::array<double, 3>, 3> mag_matrix_{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}}
    }};

    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

} // namespace mag_pub

#endif // MAG_PUB_MAG_NODE_HPP_
