#ifndef MEGA_DIFF_DRIVE_CONTROL_MEGA_DIFF_DRIVE_HARDWARE_HPP_
#define MEGA_DIFF_DRIVE_CONTROL_MEGA_DIFF_DRIVE_HARDWARE_HPP_

#include <string>
#include <vector>
#include <array>
#include <chrono>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


namespace mega_diff_drive_control
{
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class MegaDiffDriveHardware : public hardware_interface::SystemInterface
    {
    public:
        MegaDiffDriveHardware() = default;
        virtual ~MegaDiffDriveHardware() = default;
        
        virtual CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time & time,
                                            const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time,
                                             const rclcpp::Duration & period) override;

    private:
        // State and command arrays
        std::vector<double> hw_commands_;      // 4 wheel velocities commanded
        std::vector<double> hw_positions_;     // 4 wheel positions (radians)
        std::vector<double> hw_velocities_;    // 4 wheel velocities (rad/s)

        // Odometry state
        double odom_x_ = 0.0;      // meters
        double odom_y_ = 0.0;      // meters
        double odom_theta_ = 0.0;  // radians

        // Serial communication
        int serial_fd_ = -1;
        std::string serial_port_name_ = "/dev/ttyACM0";
        int baud_rate_ = 115200;

        // COBS packet handling
        static constexpr int COBS_BUFFER_SIZE = 256;
        std::array<uint8_t, COBS_BUFFER_SIZE> rx_buffer_;
        size_t rx_buffer_pos_ = 0;
        
        // Robot constants
        static constexpr float WHEEL_RADIUS = 0.165f;       // meters
        static constexpr float WHEEL_SEPARATION = 0.38f;    // meters (left-right distance)
        static constexpr int COUNTS_PER_REV = 3200;         // 2x quadrature from Arduino
        static constexpr float MAX_VELOCITY_RAD_S = 9.42f;  // 90 RPM
        static constexpr float LOOP_RATE_HZ = 50.0f;        // Match Arduino loop rate
        
        // Serial communication timeouts and tracking
        std::chrono::steady_clock::time_point last_read_time_;
        static constexpr int READ_TIMEOUT_MS = 100;  // Warn if no data for 100ms
        static constexpr int WATCHDOG_TIMEOUT_MS = 500;  // Stop motors if no reads for 500ms

        // Helper methods
        bool open_serial_port();
        void close_serial_port();
        bool write_serial_command(const std::array<uint8_t, 9>& packet);
        bool read_serial_feedback();
        bool parse_binary_feedback(const uint8_t* packet, size_t len);
        uint8_t cobs_encode(const uint8_t* data, size_t len, uint8_t* encoded);
        bool cobs_decode(const uint8_t* encoded, size_t len, uint8_t* decoded, size_t& decoded_len);
        int set_serial_attributes(int fd, int baud);
        void publish_odometry(const rclcpp::Time & timestamp);
        void update_odometry();
    };
}  // namespace mega_diff_drive_control

#endif  // MEGA_DIFF_DRIVE_CONTROL_MEGA_DIFF_DRIVE_HARDWARE_HPP_