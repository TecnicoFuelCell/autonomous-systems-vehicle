#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "car_msgs/msg/im_speed.hpp"

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

            publisher_ = this->create_publisher<car_msgs::msg::ImSpeed>(output_topic_, 10);
            subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_, 10, std::bind(&JoystickReader::joy_callback, this, std::placeholders::_1));
        }

    private:
        // Per-trigger adaptive calibration state. The DS4 does not report its
        // analog triggers resting at a fixed +1.0 (and may even report a
        // spurious resting offset at boot), so instead of assuming a neutral we
        // learn it: a trigger is "pressed" only when it actually MOVES away from
        // its resting value, and the baseline is re-anchored whenever the axis
        // sits still again (which also absorbs the re-centering the DS4 does
        // right after the first press/release).
        struct TriggerCalib {
            bool   locked  = false;
            int    settle  = 0;     // frames to wait + re-anchor after a release
            float  dir     = 0.0f;   // sign of the press direction (0 = unknown)
            float  neutral = 0.0f;   // learned resting value of the axis
            float  peak    = 0.0f;   // largest |deviation| seen while pressed
            float  prev    = 0.0f;   // previous raw sample (for motion detection)
        };

        static constexpr int   kCalibFrames  = 20;     // frames used to seed the neutral
        static constexpr float kPressThresh  = 0.25f;  // |dev| above which a press is real
        static constexpr float kMotionEps    = 0.02f;  // axis movement/frame that counts as pressing
        static constexpr float kReleaseFrac  = 0.35f;  // release when press depth < this * peak
        static constexpr int   kSettleFrames = 15;     // re-anchor window after a release
        static constexpr float kRestLeak     = 0.08f;  // neutral chases a still axis at this rate
        static constexpr float kHalfSpan     = 2.0f;   // full-travel deviation for normalization

        int map_value(float value, float from_min, float from_max, int to_min, int to_max) {
            return static_cast<int>((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
        }

// Returns the pressed fraction (0..1) for one trigger given the raw axis
        // value, updating its calibration state.
        float trigger_fraction(float raw, TriggerCalib &t) {
            const float dev = raw - t.neutral;
            const float speed = std::fabs(raw - t.prev);
            t.prev = raw;

            if (t.settle > 0) {
                // After a release, wait for the axis to settle at its (possibly
                // re-centered) resting value and re-anchor the neutral to it
                // before allowing arming again. Output stays zero the whole time.
                --t.settle;
                t.locked = false;
                t.dir = 0.0f;
                t.peak = 0.0f;
                t.neutral += kRestLeak * dev;
                return 0.0f;
            }

            if (!t.locked) {
                if (std::fabs(dev) > kPressThresh && speed > kMotionEps) {
                    // Genuine press: the axis is actually moving away from rest.
                    t.locked = true;
                    t.dir = (dev > 0.0f) ? 1.0f : -1.0f;
                    t.peak = std::fabs(dev);
                    RCLCPP_INFO(this->get_logger(),
                                "Joystick trigger armed: sign=%+.0f raw=%.3f neutral=%.3f",
                                t.dir, raw, t.neutral);
                } else if (speed < kMotionEps) {
                    // Resting: pull the neutral toward wherever the axis sits
                    // (covers the spurious boot offset and post-press re-centering).
                    t.neutral += kRestLeak * dev;
                }
                return 0.0f;
            }

            t.peak = std::max(t.peak, std::fabs(dev));

            // Press depth signed in the direction the user is pressing. Release
            // fires when it falls back toward rest OR crosses the neutral, even
            // if the DS4 re-centers its trigger far past the old baseline.
            const float depth = dev * t.dir;
            if (depth < kReleaseFrac * t.peak) {
                t.settle = kSettleFrames;
                t.locked = false;
                t.dir = 0.0f;
                t.peak = 0.0f;
                return 0.0f;
            }

            if (depth <= 0.0f) return 0.0f;
            return std::min(depth / kHalfSpan, 1.0f);
        }

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
            const size_t n_axes = msg->axes.size();
            const float axis_l2 = (n_axes > 4) ? msg->axes[4] : 0.0f;
            const float axis_r2 = (n_axes > 5) ? msg->axes[5] : 0.0f;
            const float left_analog = (n_axes > 0) ? -msg->axes[0] : 0.0f;

            if (!calibrated_) {
                calib_sum_l2_ += axis_l2;
                calib_sum_r2_ += axis_r2;
                t_l2_.prev = axis_l2;
                t_r2_.prev = axis_r2;
                if (++calib_samples_ >= kCalibFrames) {
                    t_l2_.neutral = calib_sum_l2_ / static_cast<float>(kCalibFrames);
                    t_r2_.neutral = calib_sum_r2_ / static_cast<float>(kCalibFrames);
                    calibrated_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "Joystick trigger baselines calibrated over %d frames: "
                                "neutral_l2=%.3f neutral_r2=%.3f",
                                kCalibFrames, t_l2_.neutral, t_r2_.neutral);
                }
            }

            const float frac_l2 = calibrated_ ? trigger_fraction(axis_l2, t_l2_) : 0.0f;
            const float frac_r2 = calibrated_ ? trigger_fraction(axis_r2, t_r2_) : 0.0f;

            const int mapped_l2 = static_cast<int>(frac_l2 * 255.0f);
            const int mapped_r2 = static_cast<int>(frac_r2 * 255.0f);
            const int mapped_left_analog = map_value(left_analog, -1.0f, 1.0f, -200, 200);

            auto im_speed_msg = car_msgs::msg::ImSpeed();
            // Propagate the source /joy stamp so downstream timing stays
            // consistent in both bag replay and live operation.
            im_speed_msg.header.stamp = msg->header.stamp;

            if (mapped_l2 > 0) {
                im_speed_msg.move = mapped_l2 / 10;
                im_speed_msg.which = "L2";
            } else {
                im_speed_msg.move = mapped_r2 / 10;
                im_speed_msg.which = "R2";
            }

            im_speed_msg.analog = mapped_left_analog;

            publisher_->publish(im_speed_msg);
        }

        std::string joy_topic_;
        std::string output_topic_;

        bool  calibrated_ = false;
        int   calib_samples_ = 0;
        float calib_sum_l2_ = 0.0f;
        float calib_sum_r2_ = 0.0f;
        TriggerCalib t_l2_;
        TriggerCalib t_r2_;

        rclcpp::Publisher<car_msgs::msg::ImSpeed>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickReader>());
    rclcpp::shutdown();
    return 0;
}
