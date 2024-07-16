#include "robot_hardware/robot_system.hpp"

namespace robot_hardware
{

    hardware_interface::CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotSystem::on_configure(
        const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        return command_interfaces;
    }

    hardware_interface::CallbackReturn RobotSystem::on_activate(
        const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotSystem::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return hardware_interface::return_type::OK;
    }

} // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware::RobotSystem, hardware_interface::SystemInterface)