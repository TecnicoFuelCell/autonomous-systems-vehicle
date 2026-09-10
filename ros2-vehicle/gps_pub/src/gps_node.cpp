#include <rclcpp/rclcpp.hpp>
// https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gps_pub/nmea.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <string>

namespace codec = gps_pub::codec;

namespace {
// Returns ROS time; if use_sim_time is true but no /clock has arrived yet
// (e.g. bag without /clock), falls back to wall time so stamps stay non-zero.
rclcpp::Time safe_now(rclcpp::Node* n) {
    rclcpp::Time t = n->get_clock()->now();
    if (t.nanoseconds() != 0) return t;
    static rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    return wall_clock.now();
}
}  // namespace

class GPSNode : public rclcpp::Node {
public:
  GPSNode() : Node("gps_node") {
    // params
    port_     = this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "gps_link");
    pose_frame_id_ = this->declare_parameter<std::string>("pose_frame_id", "gps_local");
    pub_pose_ = this->declare_parameter<bool>("publish_pose", true);
    synthetic_mode_ = this->declare_parameter<bool>("synthetic_mode", false);
    synthetic_rate_hz_ = this->declare_parameter<double>("synthetic_rate_hz", 10.0);
    use_first_fix_as_origin_ = this->declare_parameter<bool>("use_first_fix_as_origin", true);
    origin_latitude_deg_ = this->declare_parameter<double>("origin_latitude", 0.0);
    origin_longitude_deg_ = this->declare_parameter<double>("origin_longitude", 0.0);
    origin_altitude_m_ = this->declare_parameter<double>("origin_altitude", 0.0);

    // pubs (TODO: review to just keep 1?)
    // topic with important stuff (latitude, longitude)
    fix_pub_  = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    if (pub_pose_) pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gps_pose", 10);

    if (!use_first_fix_as_origin_) {
      setOrigin(origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_);
      RCLCPP_INFO(
        get_logger(),
        "Using configured GPS origin lat=%.8f lon=%.8f alt=%.2f in frame %s",
        origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_, pose_frame_id_.c_str());
    }

    if (synthetic_mode_) {
      if (synthetic_rate_hz_ <= 0.0) {
        synthetic_rate_hz_ = 10.0;
      }

      RCLCPP_INFO(
        get_logger(),
        "Synthetic GPS enabled @ %.1f Hz | publishing fixed fix lat=%.8f lon=%.8f alt=%.2f",
        synthetic_rate_hz_, origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_);

      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / synthetic_rate_hz_),
        std::bind(&GPSNode::publishSyntheticFix, this));
      return;
    }

    // serial port opening
    uart_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s", port_.c_str());
      rclcpp::shutdown(); return;
    }
    if (!configure_uart(uart_fd_, baudrate_)) {
      RCLCPP_FATAL(get_logger(), "Failed to configure UART %s @ %d", port_.c_str(), baudrate_);
      rclcpp::shutdown(); return;
    }
    RCLCPP_INFO(
      get_logger(),
      "Opened %s @ %d, fix_frame=%s, pose_frame=%s",
      port_.c_str(), baudrate_, frame_id_.c_str(), pose_frame_id_.c_str());

    // timer to poll serial (like deploy_joystick)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&GPSNode::pollSerial, this));
  }

  ~GPSNode() override {
    if (uart_fd_ >= 0) close(uart_fd_);
  }

private:
  // ---------- Serial stuff ----------
  static bool set_baud(termios &tty, int baud) {
    speed_t sp;
    switch (baud) {
      case 4800: sp = B4800; break; case 9600: sp = B9600; break;
      case 19200: sp = B19200; break; case 38400: sp = B38400; break;
      case 57600: sp = B57600; break; case 115200: sp = B115200; break;
      default: return false;
    }
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);
    return true;
  }

  static bool configure_uart(int fd, int baud) {
    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) return false;

    if (!set_baud(tty, baud)) return false;

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;               // no canonical, no echo
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;           // non-blocking read
    tty.c_cc[VTIME] = 10;          // 1.0s read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    return (tcsetattr(fd, TCSANOW, &tty) == 0);
  }

  // ---------- Utils ----------
  void setOrigin(double lat_deg, double lon_deg, double alt_m) {
    origin_latitude_deg_ = lat_deg;
    origin_longitude_deg_ = lon_deg;
    origin_altitude_m_ = alt_m;
    origin_set_ = true;
  }

  // ---------- Main read loop ----------
  void pollSerial() {
    if (synthetic_mode_) return;
    if (uart_fd_ < 0) return;

    // Read all available bytes
    char buf[256];
    ssize_t n;
    while ((n = read(uart_fd_, buf, sizeof(buf))) > 0) {
      line_buffer_.append(buf, buf + n);
      // Extract complete lines
      size_t pos;
      while ((pos = line_buffer_.find('\n')) != std::string::npos) {
        std::string raw = codec::trim(line_buffer_.substr(0, pos));
        line_buffer_.erase(0, pos + 1);
        handleLine(raw);
      }
    }
  }

  void handleLine(const std::string &raw) {
    if (raw.empty() || raw[0] != '$') return;
    if (!codec::nmea_checksum_ok(raw)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Dropping NMEA sentence with bad checksum");
      return;
    }

    auto now = safe_now(this);

    // Parse
    if (raw.size() >= 6) {
      auto typ = raw.substr(3, 3);
      if (typ == "GGA") {
        last_gga_ = codec::parseGGA(raw);
      } else if (typ == "RMC") {
        auto rmc = codec::parseRMC(raw);
        if (rmc.valid) publishFix(now, rmc, last_gga_);
      }
    }
  }

  // Shared by the real (NMEA) and synthetic paths: publishes the fix and the
  // optional pose topic with identical semantics for both modes.
  void publishAll(const rclcpp::Time &stamp,
                  double latitude, double longitude, double altitude,
                  uint8_t status,
                  const std::array<double, 9> &covariance,
                  uint8_t covariance_type) {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = frame_id_;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.status.status = status;
    fix.latitude = latitude;
    fix.longitude = longitude;
    fix.altitude = altitude;
    fix.position_covariance = covariance;
    fix.position_covariance_type = covariance_type;

    fix_pub_->publish(fix);

    // The pose origin is lazily locked on the first actually-published fix.
    if (pub_pose_ && status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      if (!origin_set_) {
        setOrigin(latitude, longitude, altitude);
        RCLCPP_INFO(
          get_logger(),
          "Locked GPS pose origin to lat=%.8f lon=%.8f alt=%.2f in frame %s",
          origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_, pose_frame_id_.c_str());
      }

      const codec::Enu enu = codec::enu_from_origin(
        origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_,
        latitude, longitude, altitude);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = pose_frame_id_;
      pose.pose.position.x = enu.e;
      pose.pose.position.y = enu.n;
      pose.pose.position.z = enu.u;
      pose.pose.orientation.w = 1.0;
      pose_pub_->publish(pose);
    }
  }

  void publishFix(const rclcpp::Time &stamp, const codec::Rmc &rmc, const codec::Gga &gga) {
    // GGA fix quality: 0=invalid,1=GPS,2=DGPS,4=RTK Fixed,5=RTK Float...
    int q = gga.fixq;
    uint8_t status = (q >= 1) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                              : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

    double altitude = std::isnan(gga.alt_m) ? 0.0 : gga.alt_m;

    // Covariance from HDOP (rough, optional)
    std::array<double, 9> covariance{};
    uint8_t covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    if (gga.hdop > 0.0) {
      double sigma_h = gga.hdop * 5.0;      // ~5 m per 1 HDOP (conservative)
      double sigma_v = sigma_h * 2.0;
      covariance = {
        sigma_h*sigma_h, 0, 0,
        0, sigma_h*sigma_h, 0,
        0, 0, sigma_v*sigma_v
      };
      covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    }

    publishAll(stamp, rmc.lat, rmc.lon, altitude, status, covariance, covariance_type);
  }

  void publishSyntheticFix() {
    if (!origin_set_) {
      setOrigin(origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_);
    }

    const std::array<double, 9> covariance = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 4.0
    };

    publishAll(safe_now(this),
               origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_,
               sensor_msgs::msg::NavSatStatus::STATUS_FIX,
               covariance,
               sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED);
  }

  // Members
  int uart_fd_{-1};
  std::string port_, frame_id_, pose_frame_id_;
  int baudrate_{115200};
  bool pub_pose_{true};
  bool synthetic_mode_{false};
  bool use_first_fix_as_origin_{true};
  bool origin_set_{false};
  double synthetic_rate_hz_{10.0};
  double origin_latitude_deg_{0.0};
  double origin_longitude_deg_{0.0};
  double origin_altitude_m_{0.0};
  std::string line_buffer_;

  codec::Gga last_gga_{};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSNode>());
  rclcpp::shutdown();
  return 0;
}