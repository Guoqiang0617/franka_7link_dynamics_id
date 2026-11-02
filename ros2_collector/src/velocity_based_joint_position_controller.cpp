#include <franka_7link_dynamics_id/velocity_based_joint_position_controller.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

namespace franka_7link_dynamics_id {

controller_interface::InterfaceConfiguration
VelocityBasedJointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
VelocityBasedJointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type VelocityBasedJointPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  // Read current joint positions and velocities
  // State interfaces are ordered: [joint1/position, joint1/velocity, joint2/position, joint2/velocity, ...]
  for (int i = 0; i < kNumJoints; ++i) {
    current_q_[i] = state_interfaces_[i * 2].get_value();      // position
    current_dq_[i] = state_interfaces_[i * 2 + 1].get_value(); // velocity
  }

  // On first update, record initial position and send zero velocity
  if (first_update_) {
    initial_q_ = current_q_;
    for (int i = 0; i < kNumJoints; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    first_update_ = false;
    return controller_interface::return_type::OK;
  }

  elapsed_time_ = elapsed_time_ + period;

  const double motion_seconds = std::max(0.001, motion_duration_);
  const double ratio = std::min(1.0, elapsed_time_.seconds() / motion_seconds);

  if (ratio >= 1.0 && !motion_finished_) {
    // Motion finished, send zero velocity
    for (int i = 0; i < kNumJoints; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    motion_finished_ = true;
    return controller_interface::return_type::OK;
  }

  if (motion_finished_) {
    // Keep sending zero velocity
    for (int i = 0; i < kNumJoints; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Compute desired velocity using derivative of smooth step function
  const double s_dot = smoothStepDerivative(ratio) / motion_seconds;

  for (int i = 0; i < kNumJoints; ++i) {
    const double delta_q = target_q_[i] - initial_q_[i];
    const double desired_velocity = s_dot * delta_q;
    command_interfaces_[i].set_value(desired_velocity);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn VelocityBasedJointPositionController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("motion_duration", 8.0);
    auto_declare<std::vector<double>>(
        "target_q", {0.0, -0.78539816339, 0.0, -2.35619449019, 0.0, 1.57079632679, 0.78539816339});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn VelocityBasedJointPositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  motion_duration_ = get_node()->get_parameter("motion_duration").as_double();

  const auto target_q = get_node()->get_parameter("target_q").as_double_array();
  if (target_q.size() != static_cast<size_t>(kNumJoints)) {
    RCLCPP_ERROR(get_node()->get_logger(), "target_q must contain exactly %d values.", kNumJoints);
    return CallbackReturn::FAILURE;
  }
  std::copy(target_q.begin(), target_q.end(), target_q_.begin());

  return CallbackReturn::SUCCESS;
}

CallbackReturn VelocityBasedJointPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);
  first_update_ = true;
  motion_finished_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn VelocityBasedJointPositionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Send zero velocity on deactivation
  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

double VelocityBasedJointPositionController::smoothStep(double ratio) const {
  // 5th-order polynomial: s(r) = r³(6r² - 15r + 10)
  const double r = std::clamp(ratio, 0.0, 1.0);
  return r * r * r * (r * (r * 6.0 - 15.0) + 10.0);
}

double VelocityBasedJointPositionController::smoothStepDerivative(double ratio) const {
  // Derivative of 5th-order polynomial: s'(r) = 30r²(r² - 2r + 1) = 30r²(r-1)²
  const double r = std::clamp(ratio, 0.0, 1.0);
  const double r_minus_1 = r - 1.0;
  return 30.0 * r * r * r_minus_1 * r_minus_1;
}

}  // namespace franka_7link_dynamics_id

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_7link_dynamics_id::VelocityBasedJointPositionController,
                       controller_interface::ControllerInterface)
