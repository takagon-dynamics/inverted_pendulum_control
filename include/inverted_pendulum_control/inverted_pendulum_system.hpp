#ifndef INVERTED_PENDULUM_CONTROL__DIFFBOT_SYSTEM_HPP_
#define INVERTED_PENDULUM_CONTROL__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "inverted_pendulum_control/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace inverted_pendulum_control
{
class InvertedPendulumSystemHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(InvertedPendulumSystemHardware);

  INVERTED_PENDULUM_CONTROL_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  INVERTED_PENDULUM_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  INVERTED_PENDULUM_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  INVERTED_PENDULUM_CONTROL_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  INVERTED_PENDULUM_CONTROL_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  INVERTED_PENDULUM_CONTROL_PUBLIC
  hardware_interface::return_type read() override;

  INVERTED_PENDULUM_CONTROL_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;
};

}  // namespace inverted_pendulum_control

#endif  // INVERTED_PENDULUM_CONTROL__InvertedPendulum_SYSTEM_HPP_
