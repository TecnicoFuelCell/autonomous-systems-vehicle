#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "wechat/msg/im_speed.hpp"

class JoystickReader : public rclcpp::Node {
    /*
    * Class name: JoystickReader
    * 
    * Description: Retrieves the joystick movement and transforms it in the current ratio to be sent to the controller. R2 means forward, L2 means stop.
    *
    * Subscribers: /joy -> Joy message
    *
    * Publishers: /joystick_movement -> ImSpeed custom message
    */ 

    public:
        JoystickReader(): Node("joystick_reader") {
            // Declare parameters (with defaults) if you want to change this, change in car_params.yaml
            joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
            output_topic_ = this->declare_parameter<std::string>("output_topic", "/joystick_movement");

            RCLCPP_INFO(this->get_logger(), "Check topics:");
            RCLCPP_INFO(this->get_logger(), "  joy_topic: %s", joy_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "  output_topic: %s", output_topic_.c_str());

            publisher_ = this->create_publisher<wechat::msg::ImSpeed>(output_topic_, 10);
            subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_, 10, std::bind(&JoystickReader::joy_callback, this, std::placeholders::_1));
        }

    private:
        int map_value(float value, float from_min, float from_max, int to_min, int to_max) {
            return static_cast<int>((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
        }

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
            float r2 = (1.0f - msg->axes[5]) / 2.0f;
            float l2 = (1.0f - msg->axes[2]) / 2.0f;

            float left_analog = - msg->axes[0]; // Left stick horizontal
            
            int mapped_r2 = map_value(r2, 0.0, 1.0, 0, 255);
            int mapped_l2 = map_value(l2, 0.0, 1.0, 0, 255);
            int mapped_left_analog = map_value(left_analog, -1.0, 1.0, -200, 200);

            auto im_speed_msg = wechat::msg::ImSpeed();
            // Propagate the source /joy stamp so downstream timing stays
            // consistent in both bag replay and live operation.
            im_speed_msg.header.stamp = msg->header.stamp;

            if (mapped_l2 > 0) {
                im_speed_msg.move = mapped_l2/10;
                im_speed_msg.which = "L2";
            } else {
                im_speed_msg.move = mapped_r2/10;
                im_speed_msg.which = "R2";
            }

            im_speed_msg.analog = mapped_left_analog;

            publisher_->publish(im_speed_msg);
        }

        std::string joy_topic_;
        std::string output_topic_;

        rclcpp::Publisher<wechat::msg::ImSpeed>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickReader>());
    rclcpp::shutdown();
    return 0;
}
