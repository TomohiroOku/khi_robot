#pragma once

#include "robot_controller/visibility_control.h"
#include "controller_interface/controller_interface.hpp"

namespace robot_controller
{

  class RobotController : public controller_interface::ControllerInterface
  {
  public:
    CONTROLLER_INTERFACE_PUBLIC
    RobotController();

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    CONTROLLER_INTERFACE_PUBLIC
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    virtual ~RobotController();
  };

} // namespace robot_controller
