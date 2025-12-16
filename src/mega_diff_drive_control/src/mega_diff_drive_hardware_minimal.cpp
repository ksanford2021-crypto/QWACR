#include "mega_diff_drive_control/mega_diff_drive_hardware.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>

#include "pluginlib/class_list_macros.hpp"

namespace mega_diff_drive_control
{
    CallbackReturn MegaDiffDriveHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        RCLCPP_FATAL(rclcpp::get_logger("MegaDiffDriveHardware"), "===  on_init CALLED ===");
        fflush(stdout);
        
        if (hardware_interface::HardwareComponentInterface::on_init(params) !=
            CallbackReturn::SUCCESS)
        {
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
        RCLCPP_FATAL(rclcpp::get_logger("MegaDiffDriveHardware"), "=== export_state_interfaces ===");
        fflush(stdout);
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); ++i)
        {
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

    std::vector<hardware_interface::CommandInterface>
    MegaDiffDriveHardware::export_command_interfaces()
    {
        RCLCPP_FATAL(rclcpp::get_logger("MegaDiffDriveHardware"), "=== export_command_interfaces ===");
        fflush(stdout);
        std::vector<hardware_interface::CommandInterface> command_interfaces;
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

    CallbackReturn MegaDiffDriveHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "on_configure - STUB (disabled for debug)");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn 
    MegaDiffDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "on_activate - STUB (disabled for debug)");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    MegaDiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "on_deactivate - STUB (disabled for debug)");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    MegaDiffDriveHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MegaDiffDriveHardware"),
                    "on_cleanup - STUB (disabled for debug)");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type 
    MegaDiffDriveHardware::read(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type 
    MegaDiffDriveHardware::write(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }
    
} // namespace mega_diff_drive_control

PLUGINLIB_EXPORT_CLASS(mega_diff_drive_control::MegaDiffDriveHardware, hardware_interface::SystemInterface)
