#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_7link_dynamics_id {

/**
 * @brief Velocity-based joint position controller.
 *
 * This controller uses joint velocity commands to move the robot to a target position.
 * By using velocity interface instead of position interface, we avoid libfranka's
 * strict motion continuity checks that cause position control to fail.
 */
class VelocityBasedJointPositionController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr int kNumJoints = 7;

  double smoothStep(double ratio) const;
  double smoothStepDerivative(double ratio) const;

  std::string arm_id_{"fr3"};
  std::array<double, kNumJoints> initial_q_{};
  std::array<double, kNumJoints> target_q_{};
  std::array<double, kNumJoints> current_q_{};
  std::array<double, kNumJoints> current_dq_{};
  
  double motion_duration_{8.0};
  bool motion_finished_{false};

  rclcpp::Duration elapsed_time_{0, 0};
  bool first_update_{true};
};

}  // namespace franka_7link_dynamics_id
