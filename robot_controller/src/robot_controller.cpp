#include "robot_controller/robot_controller.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace robot_controller
{

    RobotController::RobotController() : controller_interface::ControllerInterface()
    {
    }

    RobotController::~RobotController()
    {
    }

    controller_interface::CallbackReturn RobotController::on_init()
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        return conf;
    }

    controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        return conf;
    }

    controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RobotController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

} // namespace robot_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(robot_controller::RobotController, controller_interface::ControllerInterface)