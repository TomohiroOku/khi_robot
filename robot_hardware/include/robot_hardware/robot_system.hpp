#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace robot_hardware
{

  class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystem);

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    // CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    // CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    std::vector<double> joint_position_command_;
    // std::vector<double> joint_velocities_command_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;

    std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
        {"position", {}},
        {"velocity", {}}};
  };

} // namespace robot_hardware
