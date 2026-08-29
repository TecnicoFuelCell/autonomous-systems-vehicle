#include "joystick_actuator.hpp"

/*
 * Handler for magnetometer messages.
 * Publishes ROS message to /mag_data (T).
 * @param1 data (string) - the raw line read from UART
 */
void JoystickActuator::process_mag(const std::string& data) {
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
void JoystickActuator::load_mag_calibration() {
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
