#ifndef MEGA_DIFF_DRIVE_CONTROL_MEGA_DIFF_DRIVE_HARDWARE_HPP_
#define MEGA_DIFF_DRIVE_CONTROL_MEGA_DIFF_DRIVE_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


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
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_velocity_commands_;

        // POSIX file descriptor for serial port only - no LibSerial to avoid initialization issues
        int fallback_fd_ = -1;

        std::string serial_port_name_;
        int baud_rate_;
    };
}  // namespace mega_diff_drive_control

#endif  // MEGA_DIFF_DRIVE_CONTROL_MEGA_DIFF_DRIVE_HARDWARE_HPP_