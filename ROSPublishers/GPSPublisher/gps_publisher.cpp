#include <rclcpp/rclcpp.hpp>
// https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html
#include <sensor_msgs/msg/nav_sat_fix.hpp> 
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nmea_msgs/msg/sentence.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <string>
#include <vector>
#include <sstream>
#include <cmath>

using std::placeholders::_1;

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
    pub_raw_  = this->declare_parameter<bool>("publish_raw_nmea", true);
    pub_vel_  = this->declare_parameter<bool>("publish_velocity", true);
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
    if (pub_vel_) vel_pub_  = this->create_publisher<geometry_msgs::msg::TwistStamped>("/gps/vel", 10);
    if (pub_pose_) pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gps_pose", 10);
    // publisher for the raw line (ex: $GPGGA,...,..,..)
    if (pub_raw_) nmea_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("/gps/nmea_sentence", 10);

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
  static std::string trim(const std::string &s) {
    auto b = s.find_first_not_of(" \r\n\t");
    auto e = s.find_last_not_of(" \r\n\t");
    if (b == std::string::npos) return "";
    return s.substr(b, e - b + 1);
  }

  static std::vector<std::string> split(const std::string &s, char delim=',') {
    std::vector<std::string> out; std::string tok; std::istringstream ss(s);
    while (std::getline(ss, tok, delim)) out.push_back(tok);
    return out;
  }

  static bool ddmm_to_deg(const std::string &ddmm, char hemi, double &out_deg) {
    if (ddmm.empty()) return false;
    auto dot = ddmm.find('.');
    if (dot == std::string::npos || dot < 2) return false;
    int mm_start = static_cast<int>(dot) - 2;
    try {
      double deg = std::stod(ddmm.substr(0, mm_start));
      double minutes = std::stod(ddmm.substr(mm_start));
      double dec = deg + minutes/60.0;
      if (hemi=='S' || hemi=='W') dec = -dec;
      out_deg = dec;
      return true;
    } catch (const std::exception &) {
      return false;  // malformed numeric field — drop this sentence
    }
  }

  // Validate the NMEA checksum: XOR of all chars between '$' and '*' must equal the
  // two hex digits after '*'. Returns true when valid; also true when no '*' is
  // present at all (some receivers omit it) so we don't reject otherwise-good lines.
  static bool nmea_checksum_ok(const std::string &line) {
    auto star = line.rfind('*');
    if (star == std::string::npos) return true;        // no checksum field present
    if (star + 2 >= line.size()) return false;         // '*' but missing the 2 hex digits
    unsigned char sum = 0;
    for (size_t i = 1; i < star; ++i) sum ^= static_cast<unsigned char>(line[i]);
    try {
      unsigned int given = std::stoul(line.substr(star + 1, 2), nullptr, 16);
      return sum == given;
    } catch (const std::exception &) {
      return false;
    }
  }

  // ---------- NMEA parsers (RMC + GGA) ----------
  struct RMC {
    bool valid=false; double lat=std::nan(""), lon=std::nan(""),
    speed_kn=0.0, course_deg=std::nan(""); // A/V validity flag
  };

  struct GGA {
    bool have=false; int fixq=0; int nsat=0; double hdop=0.0; double alt_m=std::nan("");
  };

  struct Ecef {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  static constexpr double kWgs84A = 6378137.0;
  static constexpr double kWgs84F = 1.0 / 298.257223563;
  static constexpr double kWgs84ESq = kWgs84F * (2.0 - kWgs84F);

  static double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
  }

  static Ecef geodeticToEcef(double lat_deg, double lon_deg, double alt_m) {
    const double lat_rad = deg2rad(lat_deg);
    const double lon_rad = deg2rad(lon_deg);
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84ESq * sin_lat * sin_lat);

    Ecef ecef;
    ecef.x = (n + alt_m) * cos_lat * cos_lon;
    ecef.y = (n + alt_m) * cos_lat * sin_lon;
    ecef.z = (n * (1.0 - kWgs84ESq) + alt_m) * sin_lat;
    return ecef;
  }

  void setOrigin(double lat_deg, double lon_deg, double alt_m) {
    origin_latitude_deg_ = lat_deg;
    origin_longitude_deg_ = lon_deg;
    origin_altitude_m_ = alt_m;
    origin_latitude_rad_ = deg2rad(lat_deg);
    origin_longitude_rad_ = deg2rad(lon_deg);
    origin_ecef_ = geodeticToEcef(lat_deg, lon_deg, alt_m);
    origin_set_ = true;
  }

  geometry_msgs::msg::PoseStamped buildPoseMessage(
      const rclcpp::Time &stamp,
      double latitude_deg,
      double longitude_deg,
      double altitude_m) const {
    const Ecef current_ecef = geodeticToEcef(latitude_deg, longitude_deg, altitude_m);
    const double dx = current_ecef.x - origin_ecef_.x;
    const double dy = current_ecef.y - origin_ecef_.y;
    const double dz = current_ecef.z - origin_ecef_.z;

    const double sin_lat = std::sin(origin_latitude_rad_);
    const double cos_lat = std::cos(origin_latitude_rad_);
    const double sin_lon = std::sin(origin_longitude_rad_);
    const double cos_lon = std::cos(origin_longitude_rad_);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = pose_frame_id_;

    // Pose is a local ENU position in meters anchored at the configured origin.
    pose.pose.position.x = -sin_lon * dx + cos_lon * dy;
    pose.pose.position.y =
      -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    pose.pose.position.z =
      cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  static RMC parseRMC(const std::string &line) {
    RMC r{};
    if (line.size()<6 || line.substr(3,3)!="RMC") return r;
    auto p = split(line, ',');
    if (p.size() < 10) return r;
    // $GxRMC,1:time,2:status(A/V),3:lat,4:N/S,5:lon,6:E/W,7:speed(kn),8:course,9:date,...
    if (p[2]!="A") return r; // not valid
    double lat, lon;
    if (!ddmm_to_deg(p[3], p[4].empty() ? 'N' : p[4][0], lat)) return r;
    if (!ddmm_to_deg(p[5], p[6].empty() ? 'E' : p[6][0], lon)) return r;
    try {
      r.speed_kn = p[7].empty()? 0.0 : std::stod(p[7]);
      r.course_deg = p[8].empty()? std::nan("") : std::stod(p[8]);
    } catch (const std::exception &) {
      return r;  // malformed speed/course — leave r.valid false, drop sentence
    }
    r.valid = true;
    r.lat = lat; r.lon = lon;
    return r;
  }

  static GGA parseGGA(const std::string &line) {
    GGA g{};
    if (line.size()<6 || line.substr(3,3)!="GGA") return g;
    auto p = split(line, ',');
    if (p.size() < 10) return g;
    // $GxGGA,time,lat,N,lon,E,fix,nsat,hdop,alt,M,...
    try {
      g.fixq = p[6].empty()? 0 : std::stoi(p[6]);
      g.nsat = p[7].empty()? 0 : std::stoi(p[7]);
      g.hdop = p[8].empty()? 0.0 : std::stod(p[8]);
      g.alt_m = p[9].empty()? std::nan("") : std::stod(p[9]);
    } catch (const std::exception &) {
      return GGA{};  // malformed field — return a fresh (have=false) struct
    }
    g.have = true;
    return g;
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
        std::string raw = trim(line_buffer_.substr(0, pos));
        line_buffer_.erase(0, pos + 1);
        handleLine(raw);
      }
    }
  }

  void handleLine(const std::string &raw) {
    if (raw.empty() || raw[0] != '$') return;
    if (!nmea_checksum_ok(raw)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Dropping NMEA sentence with bad checksum");
      return;
    }

    auto now = safe_now(this);

    if (pub_raw_) {
      nmea_msgs::msg::Sentence s;
      s.header.stamp = now;
      s.sentence = raw;
      nmea_pub_->publish(s);
    }

    // Parse
    if (raw.size() >= 6) {
      auto typ = raw.substr(3,3);
      if (typ == "GGA") {
        last_gga_ = parseGGA(raw);
      } else if (typ == "RMC") {
        auto rmc = parseRMC(raw);
        if (rmc.valid) publishFix(now, rmc, last_gga_);
      }
    }
  }

  void publishFix(const rclcpp::Time &stamp, const RMC &rmc, const GGA &gga) {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = frame_id_;

    // Status
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    // GGA fix quality: 0=invalid,1=GPS,2=DGPS,4=RTK Fixed,5=RTK Float...
    int q = gga.fixq;
    fix.status.status = (q >= 1) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                 : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

    fix.latitude  = rmc.lat;
    fix.longitude = rmc.lon;
    fix.altitude  = std::isnan(gga.alt_m) ? 0.0 : gga.alt_m;

    // Covariance from HDOP (rough, optional)
    if (gga.hdop > 0.0) {
      double sigma_h = gga.hdop * 5.0;      // ~5 m per 1 HDOP (conservative)
      double sigma_v = sigma_h * 2.0;
      fix.position_covariance = {
        sigma_h*sigma_h, 0, 0,
        0, sigma_h*sigma_h, 0,
        0, 0, sigma_v*sigma_v
      };
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    } else {
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    fix_pub_->publish(fix);

    if (pub_vel_) {
      geometry_msgs::msg::TwistStamped vel;
      vel.header = fix.header;
      vel.twist.linear.x = rmc.speed_kn * 0.514444; // knots -> m/s
      vel_pub_->publish(vel);
    }

    if (pub_pose_ && fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      if (!origin_set_) {
        setOrigin(fix.latitude, fix.longitude, fix.altitude);
        RCLCPP_INFO(
          get_logger(),
          "Locked GPS pose origin to lat=%.8f lon=%.8f alt=%.2f in frame %s",
          origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_, pose_frame_id_.c_str());
      }

      pose_pub_->publish(buildPoseMessage(stamp, fix.latitude, fix.longitude, fix.altitude));
    }
  }

  void publishSyntheticFix() {
    if (!origin_set_) {
      setOrigin(origin_latitude_deg_, origin_longitude_deg_, origin_altitude_m_);
    }

    const auto stamp = safe_now(this);

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = frame_id_;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.latitude = origin_latitude_deg_;
    fix.longitude = origin_longitude_deg_;
    fix.altitude = origin_altitude_m_;
    fix.position_covariance = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 4.0
    };
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    fix_pub_->publish(fix);

    if (pub_vel_) {
      geometry_msgs::msg::TwistStamped vel;
      vel.header = fix.header;
      vel.twist.linear.x = 0.0;
      vel_pub_->publish(vel);
    }

    if (pub_pose_) {
      pose_pub_->publish(buildPoseMessage(stamp, fix.latitude, fix.longitude, fix.altitude));
    }
  }

  // Members
  int uart_fd_{-1};
  std::string port_, frame_id_, pose_frame_id_;
  int baudrate_{115200};
  bool pub_raw_{true}, pub_vel_{true}, pub_pose_{true};
  bool synthetic_mode_{false};
  bool use_first_fix_as_origin_{true};
  bool origin_set_{false};
  double synthetic_rate_hz_{10.0};
  double origin_latitude_deg_{0.0};
  double origin_longitude_deg_{0.0};
  double origin_altitude_m_{0.0};
  double origin_latitude_rad_{0.0};
  double origin_longitude_rad_{0.0};
  std::string line_buffer_;

  GGA last_gga_{};
  Ecef origin_ecef_{};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSNode>());
  rclcpp::shutdown();
  return 0;
}
