#include "robot_hardware/robot_system.hpp"
#include "rclcpp/rclcpp.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using return_type = hardware_interface::return_type;

namespace robot_hardware
{

    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        joint_position_.assign(info_.joints.size(), 0);
        joint_velocities_.assign(info_.joints.size(), 0);
        joint_position_command_.assign(info_.joints.size(), 0);
        // joint_velocities_command_.assign(info_.joints.size(), 0);

        for (const auto &joint : info_.joints)
        {
            for (const auto &interface : joint.state_interfaces)
            {
                joint_interfaces[interface.name].push_back(joint.name);
            }
        }

        return CallbackReturn::SUCCESS;
    }

    // CallbackReturn RobotSystem::on_configure(
    //     const rclcpp_lifecycle::State &)
    // {
    //     return CallbackReturn::SUCCESS;
    // }

    std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        int ind = 0;
        for (const auto &joint_name : joint_interfaces["position"])
        {
            state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
        }

        ind = 0;
        for (const auto &joint_name : joint_interfaces["velocity"])
        {
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        int ind = 0;
        for (const auto &joint_name : joint_interfaces["position"])
        {
            command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
        }

        // ind = 0;
        // for (const auto &joint_name : joint_interfaces["velocity"])
        // {
        //     command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
        // }

        return command_interfaces;
    }

    // CallbackReturn RobotSystem::on_activate(
    //     const rclcpp_lifecycle::State &)
    // {
    //     return CallbackReturn::SUCCESS;
    // }

    // CallbackReturn RobotSystem::on_deactivate(
    //     const rclcpp_lifecycle::State &)
    // {
    //     return CallbackReturn::SUCCESS;
    // }

    return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // for (size_t i = 0; i < joint_velocities_command_.size(); i++)
        // {
        //     joint_velocities_[i] = joint_velocities_command_[i];
        //     joint_position_[i] += joint_velocities_command_[i] * period.seconds();
        // }

        for (size_t i = 0; i < joint_position_command_.size(); i++)
        {
            joint_position_[i] = joint_position_command_[i];
        }

        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return return_type::OK;
    }

} // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware::RobotSystem, hardware_interface::SystemInterface)