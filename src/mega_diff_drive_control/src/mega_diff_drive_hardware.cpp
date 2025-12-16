#include "mega_diff_drive_control/mega_diff_drive_hardware.hpp"

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
        RCLCPP_FATAL(rclcpp::get_logger("MegaDiffDriveHardware"), "=== on_init CALLED ===");
        fflush(stdout);
        fflush(stderr);
        // Call base implementation to populate info_ and parse interfaces
        if (hardware_interface::HardwareComponentInterface::on_init(params) !=
            CallbackReturn::SUCCESS)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MegaDiffDriveHardware"), "Base on_init failed");
            return CallbackReturn::ERROR;
        }

       RCLCPP_FATAL(rclcpp::get_logger("MegaDiffDriveHardware"), "=== Base on_init SUCCESS ===");
       fflush(stdout);
       fflush(stderr);
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
            // 1. position state inerfaces (4 total)
            state_interfaces.emplace_back(
                info_.joints[i].name,

            hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
                
            state_interfaces.emplace_back(
                info_.joints[i].name,

                hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        }
        
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Exported 8 state interfaces");
        return state_interfaces;
    }

    // 3. Interface Export
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

    // 4. Lifecycle Methods
    CallbackReturn MegaDiffDriveHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Configuring hardware, opening serial port %s", serial_port_name_.c_str());
        
        // Try POSIX fallback first
        fallback_fd_ = ::open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY);
        if (fallback_fd_ >= 0) {
            // Configure serial port
            struct termios tty;
            if (tcgetattr(fallback_fd_, &tty) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                            "Error from tcgetattr: %s", std::strerror(errno));
                ::close(fallback_fd_);
                fallback_fd_ = -1;
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
            tty.c_cc[VTIME] = 10; // 1 second timeout
            
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(PARENB | PARODD);
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;
            
            if (tcsetattr(fallback_fd_, TCSANOW, &tty) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                            "Error from tcsetattr: %s", std::strerror(errno));
                ::close(fallback_fd_);
                fallback_fd_ = -1;
                return CallbackReturn::ERROR;
            }
            
            RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                       "POSIX serial port opened successfully: %s at %d baud",
                       serial_port_name_.c_str(), baud_rate_);
            return CallbackReturn::SUCCESS;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Failed to open serial port %s: %s",
                        serial_port_name_.c_str(), std::strerror(errno));
            return CallbackReturn::ERROR;
        }
    }

    CallbackReturn 
    MegaDiffDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Activating hardware");
        //Initialize states to NaN
        hw_positions_.assign(hw_positions_.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.assign(hw_velocities_.size(), std::numeric_limits<double>::quiet_NaN());
        // Set initial command values to zero to prevent unexpected movement
        for (uint i = 0; i < hw_commands_.size(); ++i)
        {
            hw_commands_[i] = 0.0;
        }   
        return CallbackReturn::SUCCESS;


    }

    CallbackReturn
    MegaDiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Deactivating hardware");
        // Set internal commands to 0.0 for safety
        hw_commands_.assign(hw_commands_.size(), 0.0);
        // send the stop command to the arduino (zeros for all command interfaces)
        std::ostringstream stop_stream;
        for (size_t i = 0; i < hw_commands_.size(); ++i)
        {
            stop_stream << 0.0;
            if (i < hw_commands_.size() - 1)
                stop_stream << ",";
        }
        stop_stream << "\n";
        if (fallback_fd_ >= 0) {
            std::string out = stop_stream.str();
            ::write(fallback_fd_, out.c_str(), out.size());
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    MegaDiffDriveHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "Cleaning up hardware");
        // Close POSIX serial port
        if (fallback_fd_ >= 0) {
            ::close(fallback_fd_);
            RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"), "Serial port closed during cleanup");
            fallback_fd_ = -1;
        }
        return CallbackReturn::SUCCESS;
    }
    

    // 5. Real-time read, 8 states from serial

    hardware_interface::return_type MegaDiffDriveHardware::read(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        std::string received_data;
        if (fallback_fd_ >= 0) {
            // POSIX fallback: read with minimal timeout to not block controller manager
            const int timeout_ms = 100;  // 100ms timeout
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fallback_fd_, &readfds);
            struct timeval tv;
            tv.tv_sec = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            int rv = select(fallback_fd_ + 1, &readfds, NULL, NULL, &tv);
            if (rv <= 0) {
                // Timeout is OK - just return OK with last known state
                RCLCPP_DEBUG(rclcpp::get_logger("MegaDiffDriveHardware"), "Serial read timeout (expected), returning last state");
                return hardware_interface::return_type::OK;
            }
            // read available bytes up to newline
            char buf[256];
            ssize_t n = 0;
            while (true) {
                ssize_t r = ::read(fallback_fd_, buf, sizeof(buf));
                if (r > 0) {
                    received_data.append(buf, buf + r);
                    if (received_data.find('\n') != std::string::npos) break;
                } else if (r == 0) {
                    break;
                } else {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // no more data immediately available
                        break;
                    }
                    RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"), "POSIX read error: %s", std::strerror(errno));
                    return hardware_interface::return_type::ERROR;
                }
            }
            // Trim to newline
            auto pos = received_data.find('\n');
            if (pos != std::string::npos) {
                received_data = received_data.substr(0, pos);
            }
        }

        // Parse the CSV-like data string
        std::stringstream ss(received_data);
        std::string segment;
        std::vector<double> parsed_data;

        while (std::getline(ss, segment, ',')) {
            try {
                parsed_data.push_back(std::stod(segment));
            } catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"),
                             "Failed to parse serial data segment: %s", segment.c_str());
                return hardware_interface::return_type::ERROR;
            }
        }

        if (parsed_data.size() != 8) {
            RCLCPP_WARN(rclcpp::get_logger("MegaDiffDriveHardware"),
                        "Expected 8 data points from serial, got %zu. Data: '%s'", 
                        parsed_data.size(), received_data.c_str());
            return hardware_interface::return_type::ERROR;
        }

        const size_t num_joints = info_.joints.size();
        for (uint i = 0; i < num_joints; ++i) {
            hw_positions_[i] = parsed_data[i];
            hw_velocities_[i] = parsed_data[i + num_joints];
        }

        return hardware_interface::return_type::OK;
    }

    // 6. Real-time write, 2 commands to serial
   hardware_interface::return_type 
   MegaDiffDriveHardware::write(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        // Debug: log the command vector to verify controller writes non-zero values
        std::ostringstream debug_stream;
        debug_stream << "hw_commands:";
        for (size_t i = 0; i < hw_commands_.size(); ++i) {
            debug_stream << (i == 0 ? " " : ", ") << hw_commands_[i];
        }
        static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
        static rclcpp::Logger logger = rclcpp::get_logger("MegaDiffDriveHardware");
        RCLCPP_INFO_THROTTLE(logger,
                     steady_clock,
                     1000 /* ms */,
                     "%s", debug_stream.str().c_str());

        std::ostringstream command_stream;
        for (size_t i = 0; i < hw_commands_.size(); ++i) {
            command_stream << hw_commands_[i];
            if (i < hw_commands_.size() - 1) {
                command_stream << ",";
            }
        }
        command_stream << "\n";

        if (fallback_fd_ >= 0) {
            std::string out = command_stream.str();
            ssize_t written = ::write(fallback_fd_, out.c_str(), out.size());
            if (written < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"), "POSIX write error: %s", std::strerror(errno));
                return hardware_interface::return_type::ERROR;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("MegaDiffDriveHardware"), "Serial port not open");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
    
} // namespace mega_diff_drive_control

PLUGINLIB_EXPORT_CLASS(mega_diff_drive_control::MegaDiffDriveHardware, hardware_interface::SystemInterface)









