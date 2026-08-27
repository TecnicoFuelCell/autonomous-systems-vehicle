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

class JoystickActuator : public rclcpp::Node
{
public:
    JoystickActuator()
    : Node("joystick_actuator"), uart_fd_writer(-1), uart_fd_reader(-1)
    {   
        actuator_only_ = this->declare_parameter<bool>("actuator_only", false);

        load_mag_calibration();

        auto uart_fds = actuatinator_3000::uart_utils::discover_or_fallback(this->get_logger());
        uart_fd_reader = uart_fds.reader_fd;
        uart_fd_writer = uart_fds.writer_fd;
        
        if (!actuator_only_ && uart_fd_reader < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open UART device for reading from CANtoPC");
            rclcpp::shutdown();
            return;
        }

        if (uart_fd_writer < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open UART device for writing to PCSender");
            rclcpp::shutdown();
            return;
        }

        if (actuator_only_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Running in actuator_only mode: serial reads and VESC/IMU/mag/dir publishers are disabled.");
        }

        // Subscriptions
        deadman_sub_ = this->create_subscription<std_msgs::msg::Bool>("/deadman/alive", 10,
            std::bind(&JoystickActuator::deadman_callback, this, std::placeholders::_1));
        subscription_ = this->create_subscription<wechat::msg::ImSpeed>("/joystick_movement", 10, 
            std::bind(&JoystickActuator::listener_callback, this, std::placeholders::_1));
        
        // Start the deadman stale so the actuator stays disarmed until a real
        // ALIVE (serial, normal mode) or a fresh /deadman/alive=true (actuator_only
        // mode) actually arrives. Combined with car_on=false this removes the
        // ~0.5 s fail-ON-at-boot window.
        last_alive_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);

        if (!actuator_only_) {
            // Publishers
            alive_pub_ = this->create_publisher<std_msgs::msg::Bool>("/deadman/alive", 1);
            vesc_dir_publisher_ = this->create_publisher<wechat::msg::VescData>("vesc_data", 1);
            dir_publisher_ = this->create_publisher<wechat::msg::Dir>("dir_data", 1);
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);
            mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag_data", 1);

            timer_ = this->create_wall_timer(5ms, std::bind(&JoystickActuator::read_uart, this));
        }

        // Deadman watchdog runs in BOTH modes: normal mode feeds last_alive_time_
        // from serial ALIVE, actuator_only mode feeds it from the /deadman/alive
        // topic (deadman_callback). It only ever DISARMS on staleness — arming is
        // done solely by a real ALIVE/deadman-true event.
        alive_timer_ = this->create_wall_timer(50ms, std::bind(&JoystickActuator::process_alive, this));
        
        RCLCPP_INFO(this->get_logger(), "Joystick Actuator Node Initialized successfully.");
        RCLCPP_INFO(
            this->get_logger(),
            "Deadman bridge is %s",
            actuator_only_ ? "disabled in actuator_only mode" : "reading CANtoPC UART and publishing /deadman/alive");
    }

    ~JoystickActuator() {
        if (uart_fd_reader >= 0) close(uart_fd_reader);
        if (uart_fd_writer >= 0) close(uart_fd_writer);
    }

private:
    // ---------------------------------------------------------------------- //
    // ------------------------------ CALLBACKS ----------------------------- //
    // ---------------------------------------------------------------------- //

    void deadman_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        car_on = msg->data;
        // Feed the watchdog so a fresh deadman=true keeps the actuator armed in
        // actuator_only mode (where there is no serial ALIVE). A false simply
        // disarms and is left to go stale, so it sticks until the next true.
        if (msg->data) {
            last_alive_time_ = std::chrono::steady_clock::now();
        }
        RCLCPP_DEBUG(this->get_logger(), "Deadman status updated: %s", car_on ? "ALIVE" : "DEAD");
    }

    void listener_callback(const wechat::msg::ImSpeed::SharedPtr msg) {   
        int joystick_data = msg->move;
        std::string joystick_side = msg->which;
        int joystick_steering = msg->analog;

        if (!car_on) {
            RCLCPP_DEBUG(this->get_logger(), "car NOT alive");
            return; 
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Joystick RX -> side: %s | move: %d | analog: %d", 
                     joystick_side.c_str(), joystick_data, joystick_steering);

        if (std::abs(joystick_steering) > 25) {
            joystick_steering += (joystick_steering < 0) ? 25 : -25;
            write_serial("Dir: " + std::to_string(joystick_steering));
        } else {
            write_serial("Dir: 0");
        }
        write_serial(joystick_side + ": " + std::to_string(joystick_data));
    }

    // ---------------------------------------------------------------------- //
    // ------------------------- PROCESSING FUNCTIONS ----------------------- //
    // ---------------------------------------------------------------------- //

    /*
     * Generic handler for a line of data read from UART.
     * Identifies the message from the header and delegates to the appropriate handler.
     * @param1 data (string) - the raw line read from UART
    */
    void process_data(const std::string& data) {
        std::string line = actuatinator_3000::uart_utils::trim(data);
        if (line.empty()) return;
        
        RCLCPP_DEBUG(this->get_logger(), "UART Read raw line: %s", line.c_str());
        
        if (line.find("VESC:") == 0) {
            process_vesc(line.substr(5));
        } else if (line.find("Dir:") == 0) {
            process_dir(line.substr(4));
        } else if (line.find("ALIVE") == 0) {
            last_alive_time_ = std::chrono::steady_clock::now();
            car_on = true;
            auto m = std_msgs::msg::Bool();
            m.data = true;
            if (alive_pub_) {
                alive_pub_->publish(m);
            }
            RCLCPP_DEBUG(this->get_logger(), "Received ALIVE signal");
        } else if (line.find("ACC:") == 0) {
            process_acc(line.substr(4));
        } else if (line.find("GYRO:") == 0) {
            process_gyro(line.substr(5));
        } else if (line.find("MAG:") == 0) {   
            process_mag(line.substr(4));
        } else {
            RCLCPP_DEBUG(this->get_logger(), "UART Unrecognized header: %s", line.c_str());
        }
    }

    /*
     * Handler for VESC data messages.
     * Parses the payload and publishes a ROS message to /vesc_data.
     * @param1 data (string) - the raw line read from UART
     */
    void process_vesc(const std::string& data) {
        std::vector<std::string> parts = split(data, ',');
        if (parts.size() < 7) {
            RCLCPP_ERROR(this->get_logger(), "VESC parse error: Not enough parts in payload (%zu/7)", parts.size());
            return;
        }

        try {
            auto msg = wechat::msg::VescData();
            msg.header.stamp = safe_now(this);
            
            msg.tempmosfet = std::stof(parts[0]);
            msg.avgmotorcurrent = std::stof(parts[1]);
            msg.avginputcurrent = std::stof(parts[2]);
            msg.dutycyclenow = std::stof(parts[3]);
            msg.rpm = std::stof(parts[4]);
            msg.inpvoltage = std::stof(parts[5]);
            msg.watthours = std::stof(parts[6]);

            if (std::abs(msg.tempmosfet) < 1e-5 && std::abs(msg.avgmotorcurrent) < 1e-5 &&
                std::abs(msg.avginputcurrent) < 1e-5 && std::abs(msg.dutycyclenow) < 1e-5 &&
                std::abs(msg.rpm) < 1e-5 && std::abs(msg.inpvoltage) < 1e-5 && 
                std::abs(msg.watthours) < 1e-5) {
                RCLCPP_ERROR(this->get_logger(), "VESC ALERT: All values are exactly zero!");
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Parsed VESC: rpm=%.2f, duty=%.2f, v_in=%.2f", msg.rpm, msg.dutycyclenow, msg.inpvoltage);
            if (vesc_dir_publisher_) {
                vesc_dir_publisher_->publish(msg);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "VESC parse exception: %s. Payload: %s", e.what(), data.c_str());
        }
    }

    /*
     * Handler for Dir data messages.
     * Parses the payload and publishes a ROS message to /dir_data.
     * @param1 data (string) - the raw line read from UART
     */

    void process_dir(const std::string& data) {
        try {
            int value = std::stoi(actuatinator_3000::uart_utils::trim(data));
            auto msg = wechat::msg::Dir();
            msg.header.stamp = safe_now(this);
            msg.dir = value;
            
            RCLCPP_DEBUG(this->get_logger(), "Parsed Dir: %d", value);
            if (dir_publisher_) {
                dir_publisher_->publish(msg);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Dir parse exception: %s. Payload: %s", e.what(), data.c_str());
        }
    }
    
    /*
     * Handler for ACC data messages. Parses payload and stores 
     * acceleration info on current_imu_msg_ that will be published when gyro data arrives.
     * @param1 data (string) - the raw line read from UART
     */
    void process_acc(const std::string& data) {
        std::vector<std::string> parts = split(data, ',');
        if (parts.size() >= 3) {
            try {
                current_imu_msg_.linear_acceleration.x = std::stof(parts[1]) * 0.00980665f;
                current_imu_msg_.linear_acceleration.y = std::stof(parts[2]) * 0.00980665f;
                current_imu_msg_.linear_acceleration.z = std::stof(parts[0]) * 0.00980665f;

                if (std::abs(current_imu_msg_.linear_acceleration.x) < 1e-5 &&
                    std::abs(current_imu_msg_.linear_acceleration.y) < 1e-5 &&
                    std::abs(current_imu_msg_.linear_acceleration.z) < 1e-5) {
                    RCLCPP_ERROR(this->get_logger(), "IMU ALERT: All Accelerometer values are exactly zero! Check IMU connection.");
                }

                current_imu_msg_.orientation.x = 0.0;
                current_imu_msg_.orientation.y = 0.0;
                current_imu_msg_.orientation.z = 0.0;
                current_imu_msg_.orientation.w = 0.0;

                for (int i = 0; i < 9; i++) {
                    current_imu_msg_.orientation_covariance[i] = 0.0;
                    current_imu_msg_.linear_acceleration_covariance[i] = 0.0;
                }

                RCLCPP_DEBUG(this->get_logger(), "Parsed ACC (m/s^2): x=%.3f, y=%.3f, z=%.3f", 
                             current_imu_msg_.linear_acceleration.x, 
                             current_imu_msg_.linear_acceleration.y, 
                             current_imu_msg_.linear_acceleration.z);

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "ACC parse exception: %s. Payload: %s", e.what(), data.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "ACC parse error: Insufficient data pieces (%zu)", parts.size());
        }
    }

    /*
     * Handler for gyroscope messages. Parses payload, converts from deg/s to rad/s,
     * and publishes ROS message to /imu_data (acc data in m/s^2 and gyro data in rad/s).
     * @param1 data (string) - the raw line read from UART
     */
    void process_gyro(const std::string& data) {
        std::vector<std::string> parts = split(data, ',');
        if (parts.size() >= 3) {
            try {
                // Fator de conversão: de graus/s para rad/s (pi / 180)
                constexpr float DEG_TO_RAD = 0.01745329251f;

                current_imu_msg_.angular_velocity.x = std::stof(parts[0]) * DEG_TO_RAD;
                current_imu_msg_.angular_velocity.y = std::stof(parts[1]) * DEG_TO_RAD;
                current_imu_msg_.angular_velocity.z = std::stof(parts[2]) * DEG_TO_RAD;

                if (std::abs(current_imu_msg_.angular_velocity.x) < 1e-5 &&
                    std::abs(current_imu_msg_.angular_velocity.y) < 1e-5 &&
                    std::abs(current_imu_msg_.angular_velocity.z) < 1e-5) {
                    RCLCPP_ERROR(this->get_logger(), "IMU ALERT: All Gyroscope values are exactly zero! Check IMU connection.");
                }

                current_imu_msg_.orientation.x = 0.0;
                current_imu_msg_.orientation.y = 0.0;
                current_imu_msg_.orientation.z = 0.0;
                current_imu_msg_.orientation.w = 0.0;

                for (int i = 0; i < 9; i++) {
                    current_imu_msg_.orientation_covariance[i] = 0.0;
                    current_imu_msg_.angular_velocity_covariance[i] = 0.0;
                }

                current_imu_msg_.header.stamp = safe_now(this);
                
                RCLCPP_DEBUG(this->get_logger(), "Parsed GYRO (rad/s): x=%.3f, y=%.3f, z=%.3f", 
                             current_imu_msg_.angular_velocity.x, 
                             current_imu_msg_.angular_velocity.y, 
                             current_imu_msg_.angular_velocity.z);

                if (imu_publisher_) {
                    imu_publisher_->publish(current_imu_msg_);
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "GYRO parse exception: %s. Payload: %s", e.what(), data.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "GYRO parse error: Insufficient data pieces (%zu)", parts.size());
        }
    }

    /*
     * Handler for magnetometer messages.
     * Publishes ROS message to /mag_data (T).
     * @param1 data (string) - the raw line read from UART
     */
    void process_mag(const std::string& data) {
        std::vector<std::string> parts = split(data, ',');
        if (parts.size() >= 3) {
            try {
                auto mag_msg = sensor_msgs::msg::MagneticField();
                mag_msg.header.stamp = safe_now(this);
                mag_msg.header.frame_id = "imu_link"; 

                constexpr float UT_TO_TESLA = 1e-6f;

                // Leitura crua convertida para Tesla (unidade do sensor_msgs).
                // O offset (hard iron) e a matriz
                // (soft iron) vem do YAML carregado no arranque; com
                // apply_mag_calib=false sao 0 / identidade, logo o cru passa
                // diretamente.
                double raw_x = std::stof(parts[0]) * UT_TO_TESLA;
                double raw_y = std::stof(parts[1]) * UT_TO_TESLA;
                double raw_z = std::stof(parts[2]) * UT_TO_TESLA;

                // Alerta de IMU desligada (cru todo a zero), antes de calibrar.
                if (std::abs(raw_x) < 1e-10 &&
                    std::abs(raw_y) < 1e-10 &&
                    std::abs(raw_z) < 1e-10) {
                    RCLCPP_ERROR(this->get_logger(), "IMU ALERT: All Magnetometer values are exactly zero! Check IMU connection.");
                }

                // log raw values for debugging
                RCLCPP_DEBUG(this->get_logger(), "Raw MAG (T): x=%.9f, y=%.9f, z=%.9f", raw_x, raw_y, raw_z);

                // calibrado = matriz * (cru - offset)
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

                if (mag_publisher_) {
                    mag_publisher_->publish(mag_msg);
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "MAG parse exception: %s. Payload: %s", e.what(), data.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "MAG parse error: Insufficient data pieces (%zu)", parts.size());
        }
    }

    void process_alive() {
        const auto current_time = std::chrono::steady_clock::now();
        const std::chrono::duration<double> elapsed = current_time - last_alive_time_;
        const double elapsed_seconds = elapsed.count();

        // Watchdog only ever DISARMS. Re-arming is done exclusively by a real ALIVE
        // event (process_data, serial) or a fresh /deadman/alive=true
        // (deadman_callback). The previous "re-arm when elapsed < 0.5" branch let a
        // recent ALIVE override an explicit deadman=false e-stop — removed.
        if (car_on && elapsed_seconds > 0.5) {
            car_on = false;

            auto m = std_msgs::msg::Bool();
            m.data = false;
            if (alive_pub_) {
                alive_pub_->publish(m);
            }

            RCLCPP_WARN(this->get_logger(), "No ALIVE received for %.3f seconds, disarming (deadman false)", elapsed_seconds);
        }
    }

    // ---------------------------------------------------------------------- //
    // -------------------------------- UTILS ------------------------------- //
    // ---------------------------------------------------------------------- //
        
    void read_uart() {
        if (uart_fd_reader < 0) return;

        static std::string line_buffer;
        char c;
        while (read(uart_fd_reader, &c, 1) == 1) {
            if (c == '\n') {
                process_data(line_buffer);
                line_buffer.clear();
            } else {
                line_buffer += c;
            }
        }
    }

    /*
     * Writes a message to the serial port, appending a newline. Logs the message being sent.
     *  @param1 message, param2 uart_fd_writer, param3 logger
    */
    void write_serial(const std::string &message) {
        if (uart_fd_writer >= 0) {
            std::string msg_with_newline = message + "\n";
            ssize_t bytes_written = write(uart_fd_writer, msg_with_newline.c_str(), msg_with_newline.size());
            if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port!");
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Serial Write: %s", message.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Attempted to write to serial, but fd is closed.");
        }
    }

    /*
     * Splits a string by a given delimiter and trims whitespace from each resulting token.
     *  @param1 s (string), param2 delimiter (char)
     *  @return vector of trimmed tokens
    */
    std::vector<std::string> split(const std::string& s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(actuatinator_3000::uart_utils::trim(token));
        }
        return tokens;
    }

    /*
     * Carrega a calibracao do magnetometro (offset hard iron + matriz soft iron)
     * a partir de um YAML escrito pelo calibrate_magnetemeter.py.
     *
     * Parametros:
     *   apply_mag_calib (bool, default true) - se false, publica o mag CRU
     *       (offset 0 / matriz identidade), p.ex. para gravar um bag de calibracao.
     *   mag_calib_path (string) - caminho para o YAML de calibracao.
     *
     * Se o ficheiro faltar ou nao for legivel, cai para cru (identidade) e
     * regista um erro -- o no nunca rebenta por falta de calibracao.
     */
    void load_mag_calibration() {
        apply_mag_calib_ = this->declare_parameter<bool>("apply_mag_calib", true);
        const std::string path =
            this->declare_parameter<std::string>("mag_calib_path", "config/mag_calib.yaml");

        // Default: publicar cru (so escalado para Tesla no process_mag).
        mag_offset_ = {{0.0, 0.0, 0.0}};
        mag_matrix_ = {{ {{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}} }};

        if (!apply_mag_calib_) {
            RCLCPP_WARN(this->get_logger(),
                "apply_mag_calib=false: a publicar mag CRU (sem offset/matriz). "
                "Usa este modo para gravar um bag de calibracao.");
            return;
        }

        try {
            const YAML::Node cal = YAML::LoadFile(path)["mag_calibration"];
            if (!cal) throw std::runtime_error("falta a chave 'mag_calibration'");

            const YAML::Node off = cal["hard_iron_offset"];
            const YAML::Node mat = cal["soft_iron_matrix"];
            for (int i = 0; i < 3; ++i) {
                // IMPORTANT: YAML offsets are stored in microtesla BUT runtime math is tesla
                mag_offset_[i] = off[i].as<double>() * 1e-6;
                for (int j = 0; j < 3; ++j) {
                    mag_matrix_[i][j] = mat[i][j].as<double>();
                }
            }
            RCLCPP_INFO(this->get_logger(),
                "Calibracao do mag carregada de '%s' (modo=%s).",
                path.c_str(),
                cal["mode"] ? cal["mode"].as<std::string>().c_str() : "?");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Nao consegui carregar a calibracao do mag de '%s' (%s). A publicar CRU.",
                path.c_str(), e.what());
            mag_offset_ = {{0.0, 0.0, 0.0}};
            mag_matrix_ = {{ {{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}} }};
        }
    }

    // -- MEMBER VARIABLES -- //

    int uart_fd_writer;
    int uart_fd_reader;
    bool actuator_only_ = false;
    bool car_on = false;  // fail-safe: stay disarmed until a deadman/ALIVE arrives
    std::string read_buffer_;

    // Calibracao do magnetometro (carregada do YAML no arranque; identidade se desligada).
    bool apply_mag_calib_ = true;
    std::array<double, 3> mag_offset_{{0.0, 0.0, 0.0}};
    std::array<std::array<double, 3>, 3> mag_matrix_{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}}
    }};

    sensor_msgs::msg::Imu current_imu_msg_;

    // Deadman timeout is elapsed wall time, independent from ROS clock source.
    std::chrono::steady_clock::time_point last_alive_time_;
    rclcpp::TimerBase::SharedPtr alive_timer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<wechat::msg::Dir>::SharedPtr dir_publisher_;
    rclcpp::Publisher<wechat::msg::VescData>::SharedPtr vesc_dir_publisher_;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    
    rclcpp::Subscription<wechat::msg::ImSpeed>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alive_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickActuator>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
