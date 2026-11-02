#include <franka_7link_dynamics_id/ffs_trajectory_controller.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

namespace franka_7link_dynamics_id {

controller_interface::InterfaceConfiguration
FFSTrajectoryController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
FFSTrajectoryController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type FFSTrajectoryController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;
  const double t = elapsed_time_.seconds();

  // Check if trajectory is finished
  if (t >= duration_) {
    if (!trajectory_finished_) {
      // Send zero velocity
      for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(0.0);
      }
      // Publish trajectory done signal
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      trajectory_done_pub_->publish(msg);
      trajectory_finished_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "FFS trajectory execution completed.");
    } else {
      // Keep sending zero velocity after completion
      for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(0.0);
      }
    }
    return controller_interface::return_type::OK;
  }

  // Compute and command velocity for each joint
  for (int j = 0; j < kNumJoints; ++j) {
    double velocity = computeFFSVelocity(j, t);

    // Safety: clamp velocity to limits
    velocity = std::clamp(velocity, -joint_velocity_max_[j], joint_velocity_max_[j]);

    // Safety: check current position against limits
    // State interfaces: [joint1/position, joint1/velocity, joint2/position, ...]
    double current_pos = state_interfaces_[j * 2].get_value();
    if (current_pos <= joint_position_min_[j] && velocity < 0.0) {
      velocity = 0.0;
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Joint %d at lower position limit, clamping velocity to 0.", j + 1);
    }
    if (current_pos >= joint_position_max_[j] && velocity > 0.0) {
      velocity = 0.0;
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Joint %d at upper position limit, clamping velocity to 0.", j + 1);
    }

    command_interfaces_[j].set_value(velocity);
  }

  return controller_interface::return_type::OK;
}

FFSTrajectoryController::CallbackReturn FFSTrajectoryController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<int>("num_harmonics", 5);
    auto_declare<double>("base_frequency", 0.05);
    auto_declare<double>("duration", 20.0);
    auto_declare<std::vector<double>>("initial_q",
        {0.0, -0.78539816339, 0.0, -2.35619449019, 0.0, 1.57079632679, 0.78539816339});
    auto_declare<std::vector<double>>("ak", std::vector<double>{});
    auto_declare<std::vector<double>>("bk", std::vector<double>{});
    auto_declare<std::vector<double>>("joint_position_min",
        {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973});
    auto_declare<std::vector<double>>("joint_position_max",
        {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973});
    auto_declare<std::vector<double>>("joint_velocity_max",
        {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

FFSTrajectoryController::CallbackReturn FFSTrajectoryController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  num_harmonics_ = get_node()->get_parameter("num_harmonics").as_int();
  base_frequency_ = get_node()->get_parameter("base_frequency").as_double();
  duration_ = get_node()->get_parameter("duration").as_double();

  omega_f_ = 2.0 * M_PI * base_frequency_;

  // Load initial joint positions
  const auto init_q = get_node()->get_parameter("initial_q").as_double_array();
  if (init_q.size() != static_cast<size_t>(kNumJoints)) {
    RCLCPP_ERROR(get_node()->get_logger(), "initial_q must have exactly %d elements.", kNumJoints);
    return CallbackReturn::ERROR;
  }
  std::copy(init_q.begin(), init_q.end(), initial_q_.begin());

  // FFS coefficients (ak, bk) are set dynamically by the orchestrator before activation.
  // We don't validate them here — validation happens in on_activate().

  // Load joint limits
  const auto pos_min = get_node()->get_parameter("joint_position_min").as_double_array();
  const auto pos_max = get_node()->get_parameter("joint_position_max").as_double_array();
  const auto vel_max = get_node()->get_parameter("joint_velocity_max").as_double_array();
  std::copy(pos_min.begin(), pos_min.end(), joint_position_min_.begin());
  std::copy(pos_max.begin(), pos_max.end(), joint_position_max_.begin());
  std::copy(vel_max.begin(), vel_max.end(), joint_velocity_max_.begin());

  // Create trajectory done publisher
  trajectory_done_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/trajectory_done", rclcpp::QoS(1).reliable());

  RCLCPP_INFO(get_node()->get_logger(),
              "FFS Trajectory Controller configured: %d harmonics, base_freq=%.4f Hz, "
              "duration=%.1f s",
              num_harmonics_, base_frequency_, duration_);

  return CallbackReturn::SUCCESS;
}

FFSTrajectoryController::CallbackReturn FFSTrajectoryController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);
  trajectory_finished_ = false;

  // Re-read FFS parameters (set by orchestrator before activation)
  num_harmonics_ = get_node()->get_parameter("num_harmonics").as_int();
  base_frequency_ = get_node()->get_parameter("base_frequency").as_double();
  duration_ = get_node()->get_parameter("duration").as_double();
  omega_f_ = 2.0 * M_PI * base_frequency_;

  const auto ak_flat = get_node()->get_parameter("ak").as_double_array();
  const auto bk_flat = get_node()->get_parameter("bk").as_double_array();

  const size_t expected_size = static_cast<size_t>(kNumJoints * num_harmonics_);
  if (ak_flat.size() != expected_size || bk_flat.size() != expected_size) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "ak and bk must each have %zu elements (7 joints x %d harmonics). "
                 "Got ak=%zu, bk=%zu. Set them before activating!",
                 expected_size, num_harmonics_, ak_flat.size(), bk_flat.size());
    return CallbackReturn::ERROR;
  }

  // Reshape into [joint][harmonic]
  ak_.resize(kNumJoints);
  bk_.resize(kNumJoints);
  for (int j = 0; j < kNumJoints; ++j) {
    ak_[j].resize(num_harmonics_);
    bk_[j].resize(num_harmonics_);
    for (int k = 0; k < num_harmonics_; ++k) {
      ak_[j][k] = ak_flat[j * num_harmonics_ + k];
      bk_[j][k] = bk_flat[j * num_harmonics_ + k];
    }
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "FFS Trajectory Controller activated: %d harmonics, freq=%.4f Hz, duration=%.1f s",
              num_harmonics_, base_frequency_, duration_);
  return CallbackReturn::SUCCESS;
}

FFSTrajectoryController::CallbackReturn FFSTrajectoryController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Send zero velocity on deactivation
  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "FFS Trajectory Controller deactivated.");
  return CallbackReturn::SUCCESS;
}

double FFSTrajectoryController::computeFFSVelocity(int joint, double t) const {
  // dq_j(t) = Σ_{k=1}^{N} [ a_{j,k} · cos(k·ω_f·t) + b_{j,k} · sin(k·ω_f·t) ]
  double velocity = 0.0;
  for (int k = 0; k < num_harmonics_; ++k) {
    const double k_omega = static_cast<double>(k + 1) * omega_f_;
    velocity += ak_[joint][k] * std::cos(k_omega * t) +
                bk_[joint][k] * std::sin(k_omega * t);
  }
  return velocity;
}

double FFSTrajectoryController::computeFFSPosition(int joint, double t) const {
  // q_j(t) = q_j0 + Σ_{k=1}^{N} [ a_{j,k}/(k·ω_f) · sin(k·ω_f·t)
  //                                - b_{j,k}/(k·ω_f) · cos(k·ω_f·t) ]
  // Note: at t=0, the position offset is -Σ b_{j,k}/(k·ω_f), so we add
  // a constant to ensure q_j(0) = q_j0:
  // q_j(t) = q_j0 + Σ_{k=1}^{N} [ a_{j,k}/(k·ω_f) · sin(k·ω_f·t)
  //                                - b_{j,k}/(k·ω_f) · (cos(k·ω_f·t) - 1) ]
  double position = initial_q_[joint];
  for (int k = 0; k < num_harmonics_; ++k) {
    const double k_omega = static_cast<double>(k + 1) * omega_f_;
    position += ak_[joint][k] / k_omega * std::sin(k_omega * t) -
                bk_[joint][k] / k_omega * (std::cos(k_omega * t) - 1.0);
  }
  return position;
}

}  // namespace franka_7link_dynamics_id

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_7link_dynamics_id::FFSTrajectoryController,
                       controller_interface::ControllerInterface)
