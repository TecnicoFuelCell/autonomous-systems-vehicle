// OdomNode.cpp
#include <chrono>
#include <memory>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "wechat/msg/vesc_data.hpp"
#include "wechat/msg/dir.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class OdomNode : public rclcpp::Node {
  /*
  * Class name: OdomNode
  * 
  * Description: Based on the data retrieved from the sensors, it calculates the odometry of the car and publishes it as a topic and TFs. Based on the Ackerman model.
  *
  * Subscribers: /vesc_data -> custom VescData message
  *              /dir_data -> custom Dir message
  *
  * Publishers: /odom -> Odometry message
  */ 

  public:
    OdomNode() : Node("odom_node") {
      using std::placeholders::_1;

      vesc_sub_ = this->create_subscription<wechat::msg::VescData>( "/vesc_data", 10, std::bind(&OdomNode::vesc_callback, this, _1));
      dir_sub_ = this->create_subscription<wechat::msg::Dir>( "/dir_data", 10, std::bind(&OdomNode::dir_callback, this, _1));
      
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

      last_time_ = this->get_clock()->now();

      // Physical parameters
      wheel_radius_ = 0.2825;
      gearbox_ = 0.1;                  // 10 to 1 ratio
      axle_length_ = 1.13595; 
      wheelbase_length_ = 1.57667;
      rpm_ratio_ = this->declare_parameter<double>("rpm_ratio", 0.25);
      steering_gain_ = this->declare_parameter<double>("steering_gain", -1620.0);
      steering_offset_ = this->declare_parameter<double>("steering_offset", 0.0);

      RCLCPP_INFO(
        this->get_logger(),
        "Odom model: rpm_ratio=%.6f steering_gain=%.6f steering_offset=%.6f",
        rpm_ratio_, steering_gain_, steering_offset_);

      state_pos_.setZero();
      state_rot_.setIdentity();
      linear_velocity_ = 0.0;
      steering_angle_ = 0.0;
      last_time_initialized_ = false;
    }

  private:
    void dir_callback(const wechat::msg::Dir::SharedPtr msg) {
      steering_angle_ = msg->dir;
    }


    void vesc_callback(const wechat::msg::VescData::SharedPtr msg) {
      rclcpp::Time current_time(msg->header.stamp);
      if (current_time.nanoseconds() == 0) {
        current_time = this->get_clock()->now();
      }
      double rpm = msg->rpm;
      double linear_velocity = calculate_linear_velocity(rpm);
      double turn_radius = calculate_turn_radius(steering_angle_);

      if (!last_time_initialized_) {
        last_time_ = current_time;
        last_time_initialized_ = true;
        linear_velocity_ = linear_velocity;
        publish_odom(current_time, turn_radius);
        return;
      }

      double dt = (current_time - last_time_).seconds();
      last_time_ = current_time;

      if (dt <= 0.0 || dt > 1.0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Skipping odom integration with invalid dt=%.3f s", dt);
        linear_velocity_ = linear_velocity;
        publish_odom(current_time, turn_radius);
        return;
      }

      Eigen::Vector3d displacement;
      double sigma = 0.0;

      if (turn_radius == 0) {
        displacement = state_rot_ * Eigen::Vector3d(linear_velocity * dt, 0, 0);
      } 
      else {
        sigma = linear_velocity * dt / turn_radius;
        double x_t = turn_radius * (1 - std::cos(sigma));
        double y_t = turn_radius * std::sin(sigma);
        displacement = state_rot_ * Eigen::Vector3d(y_t, x_t, 0);
      }

      Eigen::AngleAxisd rotation_update(sigma, Eigen::Vector3d::UnitZ());
      state_pos_ += displacement;
      state_rot_ = (rotation_update * state_rot_).normalized();
      linear_velocity_ = linear_velocity;

      publish_odom(current_time, turn_radius);
    }


    double calculate_linear_velocity(double rpm) {
      double real_rpm = rpm * gearbox_;
      double omega = (real_rpm * 2.0 * M_PI) / 60.0;

      return omega * wheel_radius_ * rpm_ratio_;
    }


    double calculate_turn_radius(double angle) {
      double steering_command = angle - steering_offset_;
      if (std::abs(steering_command) < 1e-9) 
        return 0.0;

      return steering_gain_ / steering_command;
    }


    void publish_odom(const rclcpp::Time &time, double turn_radius) {
      // Publish the topic
      geometry_msgs::msg::Quaternion quat;
      quat.x = state_rot_.x();
      quat.y = state_rot_.y();
      quat.z = state_rot_.z();
      quat.w = state_rot_.w();

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      odom.pose.pose.position.x = state_pos_.x();
      odom.pose.pose.position.y = state_pos_.y();
      odom.pose.pose.position.z = state_pos_.z();
      odom.pose.pose.orientation = quat;

      odom.twist.twist.linear.x = linear_velocity_;
      odom.twist.twist.angular.z = (turn_radius == 0) ? 0.0 : linear_velocity_ / turn_radius; //! I THINK THIS IS WRONG

      odom_pub_->publish(odom);
      
      // Publish the TF
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = time;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";

      tf.transform.translation.x = state_pos_.x();
      tf.transform.translation.y = state_pos_.y();
      tf.transform.translation.z = state_pos_.z();
      tf.transform.rotation = quat;

      tf_broadcaster_->sendTransform(tf);
    }

    
    rclcpp::Subscription<wechat::msg::VescData>::SharedPtr vesc_sub_;
    rclcpp::Subscription<wechat::msg::Dir>::SharedPtr dir_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    Eigen::Vector3d state_pos_;
    Eigen::Quaterniond state_rot_;
    double linear_velocity_;
    double steering_angle_;
    rclcpp::Time last_time_;
    bool last_time_initialized_;

    // Parameters of the current state
    double wheel_radius_;
    double gearbox_;
    double axle_length_;
    double wheelbase_length_;
    double rpm_ratio_;
    double steering_gain_;
    double steering_offset_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
