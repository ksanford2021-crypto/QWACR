#include "mega_diff_drive_control/mega_diff_drive_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

// POSIX headers for fallback serial I/O
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>

// Required for the plugin to be exported
#include "pluginlib/class_list_macros.hpp"

namespace mega_diff_drive_control
{
    CallbackReturn MegaDiffDriveHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"), "=== on_init CALLED ===");
        
        // Call base implementation to populate info_ and parse interfaces
        if (hardware_interface::HardwareComponentInterface::on_init(params) !=
            CallbackReturn::SUCCESS)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MegaDiffDriveHardware"), "Base on_init failed");
            return CallbackReturn::ERROR;
        }

        if (info_.joints.size() != 4)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("MegaDiffDriveHardware"),
                "Expected 4 joints, got %zu.",
                info_.joints.size());
            return CallbackReturn::ERROR;
        }

        // Read Serial parameters from URDF/YAML
        auto it_port = info_.hardware_parameters.find("serial_port");
        auto it_baud = info_.hardware_parameters.find("baud_rate");
        if (it_port != info_.hardware_parameters.end())
        {
            serial_port_name_ = it_port->second;
        }
        if (it_baud != info_.hardware_parameters.end())
        {
            baud_rate_ = std::stoi(it_baud->second);
        }

        // Sizing the data vectors, 4 States per type, 4 commands
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(
            rclcpp::get_logger("MegaDiffDriveHardware"),
            "MegaDiffDriveHardware initialized successfully on port %s!",
            serial_port_name_.c_str());
        return CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface>
    MegaDiffDriveHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); ++i)
        {
            // position state interfaces (4 total)
            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
                
            // velocity state interfaces (4 total)
            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        }
        
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Exported 8 state interfaces");
        return state_interfaces;
    }

    // Interface Export
    std::vector<hardware_interface::CommandInterface>
    MegaDiffDriveHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        // Export velocity command interfaces for all joints
        for (uint i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_commands_[i]);
        }
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Exported %zu command interfaces", command_interfaces.size());
        return command_interfaces;
    }

    // Lifecycle Methods
    CallbackReturn MegaDiffDriveHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Configuring hardware, opening serial port %s", serial_port_name_.c_str());
        
        // Open serial port
        serial_fd_ = ::open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Failed to open serial port %s: %s",
                        serial_port_name_.c_str(), std::strerror(errno));
            return CallbackReturn::ERROR;
        }
        
        // Configure serial port
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Error from tcgetattr: %s", std::strerror(errno));
            ::close(serial_fd_);
            serial_fd_ = -1;
            return CallbackReturn::ERROR;
        }
        
        // Set baud rate
        speed_t baud;
        switch (baud_rate_) {
            case 9600: baud = B9600; break;
            case 19200: baud = B19200; break;
            case 38400: baud = B38400; break;
            case 57600: baud = B57600; break;
            case 115200: baud = B115200; break;
            default:
                RCLCPP_WARN(rclcpp::get_logger("MegaDiffDriveHardware"),
                           "Unsupported baud rate %d, using 115200", baud_rate_);
                baud = B115200;
        }
        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);
        
        // 8N1 mode, no hardware flow control, raw mode (no echo)
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~(IGNBRK | ICRNL);  // Disable break, disable CRLF conversion
        tty.c_lflag &= ~ECHO;  // Disable echo
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; // 100ms timeout
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Error from tcsetattr: %s", std::strerror(errno));
            ::close(serial_fd_);
            serial_fd_ = -1;
            return CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                   "Serial port opened successfully: %s at %d baud",
                   serial_port_name_.c_str(), baud_rate_);
        
        // Wait for Arduino to boot after DTR reset (opening serial port resets Arduino)
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                   "Waiting 2.5 seconds for Arduino to boot...");
        usleep(2500000);  // 2.5 seconds
        
        // Flush any boot messages from Arduino
        tcflush(serial_fd_, TCIOFLUSH);
        
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                   "Arduino should be ready");
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn 
    MegaDiffDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Activating hardware");
        // Initialize states to NaN
        hw_positions_.assign(hw_positions_.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.assign(hw_velocities_.size(), std::numeric_limits<double>::quiet_NaN());
        // Set initial command values to zero to prevent unexpected movement
        for (uint i = 0; i < hw_commands_.size(); ++i)
        {
            hw_commands_[i] = 0.0;
        }   
        last_read_time_ = std::chrono::steady_clock::now();
        
        // Reset odometry on activation
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_theta_ = 0.0;
        
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Hardware activated, odometry reset");
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    MegaDiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Deactivating hardware");
        // Set internal commands to 0.0 for safety
        hw_commands_.assign(hw_commands_.size(), 0.0);
        
        // Send stop command to Arduino: [0x01][0.0][0.0] (COBS framed)
        if (serial_fd_ >= 0) {
            uint8_t payload[9] = {0x01, 0, 0, 0, 0, 0, 0, 0, 0};
            uint8_t encoded[COBS_BUFFER_SIZE];
            uint8_t encoded_len = cobs_encode(payload, sizeof(payload), encoded);
            encoded[encoded_len++] = 0x00;
            ::write(serial_fd_, encoded, encoded_len);
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    MegaDiffDriveHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Cleaning up hardware");
        // Close POSIX serial port
        if (serial_fd_ >= 0) {
            ::close(serial_fd_);
            RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"), "Serial port closed during cleanup");
            serial_fd_ = -1;
        }
        return CallbackReturn::SUCCESS;
    }
    

    // Real-time read, 8 states from serial (binary PacketSerial protocol)
    // Expected format: [0x10][4× int32 encoder counts][4× float velocities]
    // Total: 1 + 16 + 16 = 33 bytes per packet

    hardware_interface::return_type MegaDiffDriveHardware::read(
        const rclcpp::Time & time,
        const rclcpp::Duration & /*period*/)
    {
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Serial port not open");
            return hardware_interface::return_type::ERROR;
        }

        // Request feedback from Arduino (CMD_REQUEST_FEEDBACK = 0x02)
        uint8_t request_cmd[] = {0x02};  // Unencoded payload
        uint8_t encoded_request[8];      // COBS needs overhead space
        size_t encoded_len = cobs_encode(request_cmd, sizeof(request_cmd), encoded_request);
        encoded_request[encoded_len] = 0;  // Add COBS delimiter
        
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Sending request: %02x %02x %02x (%zu bytes)", 
                    encoded_request[0], encoded_request[1], encoded_request[2], encoded_len + 1);
        
        ssize_t bytes_written = ::write(serial_fd_, encoded_request, encoded_len + 1);
        if (bytes_written < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Serial write error: %s", std::strerror(errno));
            return hardware_interface::return_type::ERROR;
        }
        
        // Give Arduino time to process and respond
        usleep(5000);  // 5ms should be plenty for Arduino to respond
        
        // Read response with timeout - keep reading until we get delimiter or timeout
        auto start_time = std::chrono::steady_clock::now();
        const int timeout_ms = 50;  // 50ms timeout for full packet
        bool packet_complete = false;
        
        while (!packet_complete) {
            uint8_t buf[128];
            ssize_t n = ::read(serial_fd_, buf, sizeof(buf));

            if (n < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                                "Serial read error: %s", std::strerror(errno));
                    return hardware_interface::return_type::ERROR;
                }
                // No data available yet - check timeout
            } else if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    // Accumulate until delimiter (0)
                    if (rx_buffer_pos_ < rx_buffer_.size()) {
                        rx_buffer_[rx_buffer_pos_++] = buf[i];
                    }

                    if (buf[i] == 0) {
                        // Found delimiter - packet complete
                        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                                    "Received COBS packet: %zu bytes (excluding delimiter), first 5: %02x %02x %02x %02x %02x", 
                                    rx_buffer_pos_ - 1,  // Exclude the delimiter from count
                                    rx_buffer_pos_ > 0 ? rx_buffer_[0] : 0,
                                    rx_buffer_pos_ > 1 ? rx_buffer_[1] : 0,
                                    rx_buffer_pos_ > 2 ? rx_buffer_[2] : 0,
                                    rx_buffer_pos_ > 3 ? rx_buffer_[3] : 0,
                                    rx_buffer_pos_ > 4 ? rx_buffer_[4] : 0);
                        uint8_t decoded[COBS_BUFFER_SIZE];
                        size_t decoded_len = 0;
                        // Decode WITHOUT the delimiter (rx_buffer_pos_ - 1)
                        if (cobs_decode(rx_buffer_.data(), rx_buffer_pos_ - 1, decoded, decoded_len)) {
                            RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                                        "COBS decoded: %zu bytes, ID=0x%02X", decoded_len, decoded_len > 0 ? decoded[0] : 0);
                            if (parse_binary_feedback(decoded, decoded_len)) {
                                last_read_time_ = std::chrono::steady_clock::now();
                                RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                                            "Feedback parsed successfully");
                                packet_complete = true;
                            } else {
                                RCLCPP_WARN(rclcpp::get_logger("MegaDiffDriveHardware"),
                                           "parse_binary_feedback failed (len=%zu, expected>=33)", decoded_len);
                            }
                        } else {
                            RCLCPP_WARN(rclcpp::get_logger("MegaDiffDriveHardware"),
                                       "COBS decode failed");
                        }
                        rx_buffer_pos_ = 0;  // Reset for next packet
                        break;  // Exit byte loop
                    }
                }
            }
            
            // Check timeout
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed > timeout_ms) {
                RCLCPP_WARN(rclcpp::get_logger("MegaDiffDriveHardware"),
                           "Timeout waiting for Arduino response (%ldms)", elapsed);
                break;  // Timeout - exit while loop
            }
            
            if (!packet_complete) {
                usleep(1000);  // 1ms between read attempts
            }
        }

        // Watchdog: if no feedback for too long, zero states (avoid stale data)
        auto now = std::chrono::steady_clock::now();
        auto ms_since_read = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read_time_).count();
        if (ms_since_read > WATCHDOG_TIMEOUT_MS) {
            for (auto &v : hw_velocities_) v = 0.0;
            for (auto &p : hw_positions_) p = std::numeric_limits<double>::quiet_NaN();
        }

        // Update and publish odometry
        update_odometry();
        publish_odometry(time);

        return hardware_interface::return_type::OK;
    }

    // Helper function to parse binary feedback packet
    bool MegaDiffDriveHardware::parse_binary_feedback(const uint8_t* packet, size_t len) {
        if (len < 33 || packet[0] != 0x10) {
            RCLCPP_DEBUG(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Feedback validation failed: len=%zu (need>=33), ID=0x%02X (need 0x10)",
                        len, len > 0 ? packet[0] : 0);
            return false;
        }

        const size_t num_joints = 4;

        // Parse 4 encoder counts (int32, little-endian) from bytes 1-16
        int32_t encoder_counts[4];
        for (size_t i = 0; i < num_joints; ++i) {
            uint32_t enc_bytes = (packet[1 + i*4] |
                                 (packet[2 + i*4] << 8) |
                                 (packet[3 + i*4] << 16) |
                                 (packet[4 + i*4] << 24));
            encoder_counts[i] = static_cast<int32_t>(enc_bytes);
            
            // Convert encoder counts to radians: (counts / COUNTS_PER_REV) * 2π
            // COUNTS_PER_REV = 3200 (2x quadrature on 1600 PPR encoder)
            const float COUNTS_PER_REV = 3200.0f;
            hw_positions_[i] = (encoder_counts[i] / COUNTS_PER_REV) * 2.0f * M_PI;
        }
        
        static int log_counter = 0;
        if (++log_counter % 30 == 0) {  // Log every ~1 second (30 cycles at 30Hz)
            RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Encoders: FL=%d BL=%d FR=%d BR=%d | Pos: FL=%.3f BL=%.3f FR=%.3f BR=%.3f rad",
                        encoder_counts[0], encoder_counts[1], encoder_counts[2], encoder_counts[3],
                        hw_positions_[0], hw_positions_[1], hw_positions_[2], hw_positions_[3]);
        }

        // Parse 4 velocities (float, little-endian) from bytes 17-32
        for (size_t i = 0; i < num_joints; ++i) {
            uint32_t vel_bytes = (packet[17 + i*4] |
                                 (packet[18 + i*4] << 8) |
                                 (packet[19 + i*4] << 16) |
                                 (packet[20 + i*4] << 24));
            float velocity = *reinterpret_cast<float*>(&vel_bytes);
            hw_velocities_[i] = static_cast<double>(velocity);
        }

        return true;
    }

    // Real-time write, velocity commands to serial (binary PacketSerial protocol)
    // Format: [0x01][float left_velocity][float right_velocity] = 9 bytes
    // Left velocity = average of FL and BL wheels
    // Right velocity = average of FR and BR wheels
    hardware_interface::return_type 
    MegaDiffDriveHardware::write(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Serial port not open");
            return hardware_interface::return_type::ERROR;
        }

        // Expected hw_commands_ layout: [FL, FR, BL, BR] wheels
        // Robot kinematics: average left pair (FL+BL) and right pair (FR+BR)
        const size_t FL = 0, FR = 1, BL = 2, BR = 3;

        // Calculate average velocities for left and right wheel pairs
        float left_velocity = static_cast<float>((hw_commands_[FL] + hw_commands_[BL]) / 2.0);
        float right_velocity = static_cast<float>((hw_commands_[FR] + hw_commands_[BR]) / 2.0);

        // Clamp to max velocity (±9.42 rad/s = ±90 RPM)
        const float MAX_VEL = 9.42f;
        left_velocity = std::max(-MAX_VEL, std::min(MAX_VEL, left_velocity));
        right_velocity = std::max(-MAX_VEL, std::min(MAX_VEL, right_velocity));

        // Build binary command payload: [0x01][left_vel_float][right_vel_float]
        uint8_t payload[9];
        payload[0] = 0x01;  // Command packet type

        // Encode left velocity (float, little-endian)
        uint32_t left_bits = *reinterpret_cast<uint32_t*>(&left_velocity);
        payload[1] = (left_bits >> 0) & 0xFF;
        payload[2] = (left_bits >> 8) & 0xFF;
        payload[3] = (left_bits >> 16) & 0xFF;
        payload[4] = (left_bits >> 24) & 0xFF;

        // Encode right velocity (float, little-endian)
        uint32_t right_bits = *reinterpret_cast<uint32_t*>(&right_velocity);
        payload[5] = (right_bits >> 0) & 0xFF;
        payload[6] = (right_bits >> 8) & 0xFF;
        payload[7] = (right_bits >> 16) & 0xFF;
        payload[8] = (right_bits >> 24) & 0xFF;

        // COBS-encode and send
        uint8_t encoded[COBS_BUFFER_SIZE];
        uint8_t encoded_len = cobs_encode(payload, sizeof(payload), encoded);
        encoded[encoded_len++] = 0x00;  // Packet delimiter

        ssize_t written = ::write(serial_fd_, encoded, encoded_len);
        if (written != (ssize_t)encoded_len) {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                         "Serial write error: expected %u bytes, wrote %zd", encoded_len, written);
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    // COBS encode helper
    uint8_t MegaDiffDriveHardware::cobs_encode(const uint8_t* data, size_t len, uint8_t* encoded)
    {
        size_t read_index = 0;
        size_t write_index = 1;  // leave room for first code
        size_t code_index = 0;
        uint8_t code = 1;

        while (read_index < len)
        {
            if (data[read_index] == 0)
            {
                encoded[code_index] = code;
                code = 1;
                code_index = write_index++;
                ++read_index;
            }
            else
            {
                encoded[write_index++] = data[read_index++];
                ++code;
                if (code == 0xFF)
                {
                    encoded[code_index] = code;
                    code = 1;
                    code_index = write_index++;
                }
            }
        }

        encoded[code_index] = code;
        return static_cast<uint8_t>(write_index);
    }

    // COBS decode helper
    bool MegaDiffDriveHardware::cobs_decode(const uint8_t* encoded, size_t len, uint8_t* decoded, size_t& decoded_len)
    {
        decoded_len = 0;
        size_t read_index = 0;

        while (read_index < len)
        {
            uint8_t code = encoded[read_index];
            if (code == 0)
            {
                return false;  // invalid code
            }
            ++read_index;

            for (uint8_t i = 1; i < code; ++i)
            {
                if (read_index >= len || decoded_len >= COBS_BUFFER_SIZE)
                {
                    return false;
                }
                decoded[decoded_len++] = encoded[read_index++];
            }

            if (code < 0xFF && read_index < len)
            {
                if (decoded_len >= COBS_BUFFER_SIZE)
                {
                    return false;
                }
                decoded[decoded_len++] = 0;
            }
        }

        return decoded_len > 0;
    }

    void MegaDiffDriveHardware::update_odometry()
    {
        // Compute odometry from wheel positions
        // For differential drive: 
        // - Left wheels: FL (index 0) + BL (index 2)
        // - Right wheels: FR (index 1) + BR (index 3)
        double left_pos = (hw_positions_[0] + hw_positions_[2]) / 2.0;
        double right_pos = (hw_positions_[1] + hw_positions_[3]) / 2.0;
        
        // Compute distance traveled by each side
        static double prev_left_pos = 0.0;
        static double prev_right_pos = 0.0;
        
        double left_dist = (left_pos - prev_left_pos) * WHEEL_RADIUS;
        double right_dist = (right_pos - prev_right_pos) * WHEEL_RADIUS;
        
        // Update previous positions
        prev_left_pos = left_pos;
        prev_right_pos = right_pos;
        
        // Compute change in position and orientation
        double delta_s = (left_dist + right_dist) / 2.0;  // Average distance
        double delta_theta = (right_dist - left_dist) / WHEEL_SEPARATION;  // Angular change
        
        // Update odometry
        odom_theta_ += delta_theta;
        odom_x_ += delta_s * std::cos(odom_theta_);
        odom_y_ += delta_s * std::sin(odom_theta_);
    }

    void MegaDiffDriveHardware::publish_odometry(const rclcpp::Time & /*timestamp*/)
    {
        // Odometry calculation is done in update_odometry()
        // For now, we just log it. A separate odometry publisher node or the
        // DiffDriveController can read joint states and compute/broadcast odometry.
        static int counter = 0;
        if (++counter % 30 == 0) {  // Log every 30 cycles (~1 second at 30Hz)
            RCLCPP_DEBUG(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Odometry: x=%.3f y=%.3f theta=%.3f", 
                        odom_x_, odom_y_, odom_theta_);
        }
    }
    
} // namespace mega_diff_drive_control

PLUGINLIB_EXPORT_CLASS(mega_diff_drive_control::MegaDiffDriveHardware, hardware_interface::SystemInterface)









