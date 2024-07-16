#pragma once

#include <string>
#include <vector>

#include "robot_hardware/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace robot_hardware
{

  class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystem);

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  };

} // namespace robot_hardware
