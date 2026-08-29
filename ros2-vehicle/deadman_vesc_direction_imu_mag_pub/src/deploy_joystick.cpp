#include "joystick_actuator.hpp"

// Returns ROS time; if use_sim_time is true but no /clock has arrived yet
// (e.g. bag without /clock), falls back to wall time so stamps stay non-zero.
rclcpp::Time safe_now(rclcpp::Node* n) {
    rclcpp::Time t = n->get_clock()->now();
    if (t.nanoseconds() != 0) return t;
    static rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    return wall_clock.now();
}

JoystickActuator::JoystickActuator()
: Node("joystick_actuator"), uart_fd_writer(-1), uart_fd_reader(-1)
{   
    actuator_only_ = this->declare_parameter<bool>("actuator_only", false);

    load_mag_calibration();

    auto uart_fds = deadman_vesc_direction_imu_mag_pub::uart_utils::discover_or_fallback(this->get_logger());
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
    subscription_ = this->create_subscription<car_msgs::msg::ImSpeed>("/joystick_movement", 10, 
        std::bind(&JoystickActuator::listener_callback, this, std::placeholders::_1));
    
    // Start the deadman stale so the actuator stays disarmed until a real
    // ALIVE (serial, normal mode) or a fresh /deadman/alive=true (actuator_only
    // mode) actually arrives. Combined with car_on=false this removes the
    // ~0.5 s fail-ON-at-boot window.
    last_alive_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);

    if (!actuator_only_) {
        // Publishers
        alive_pub_ = this->create_publisher<std_msgs::msg::Bool>("/deadman/alive", 1);
        vesc_dir_publisher_ = this->create_publisher<car_msgs::msg::VescData>("vesc_data", 1);
        dir_publisher_ = this->create_publisher<car_msgs::msg::Dir>("dir_data", 1);
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

JoystickActuator::~JoystickActuator() {
    if (uart_fd_reader >= 0) close(uart_fd_reader);
    if (uart_fd_writer >= 0) close(uart_fd_writer);
}

void JoystickActuator::listener_callback(const car_msgs::msg::ImSpeed::SharedPtr msg) {   
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

/*
 * Generic handler for a line of data read from UART.
 * Identifies the message from the header and delegates to the appropriate handler.
 * @param1 data (string) - the raw line read from UART
*/
void JoystickActuator::process_data(const std::string& data) {
    std::string line = deadman_vesc_direction_imu_mag_pub::uart_utils::trim(data);
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

void JoystickActuator::read_uart() {
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
void JoystickActuator::write_serial(const std::string &message) {
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
