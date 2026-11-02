#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>

namespace franka_7link_dynamics_id {

/**
 * @brief FFS Trajectory Controller
 *
 * Executes a Finite Fourier Series (FFS) trajectory using joint velocity commands.
 * The velocity is computed in real-time from FFS parameters (ak, bk):
 *
 *   q_j(t) = q_j0 + Σ_{k=1}^{N} [ a_{j,k}/(k·ω_f) · sin(k·ω_f·t)
 *                                  - b_{j,k}/(k·ω_f) · cos(k·ω_f·t) ]
 *
 *   dq_j(t) = Σ_{k=1}^{N} [ a_{j,k} · cos(k·ω_f·t) + b_{j,k} · sin(k·ω_f·t) ]
 *
 * A smooth ramp is applied at start and end to avoid discontinuities.
 */
class FFSTrajectoryController : public controller_interface::ControllerInterface {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr int kNumJoints = 7;

  // Compute FFS velocity for joint j at time t
  double computeFFSVelocity(int joint, double t) const;

  // Compute FFS position for joint j at time t
  double computeFFSPosition(int joint, double t) const;

  // Smooth ramp function: 0 at t=0, 1 at t=ramp_time
  double rampFunction(double t) const;

  std::string arm_id_;
  int num_harmonics_{5};
  double base_frequency_{0.05};  // Hz
  double omega_f_{0.0};          // 2*pi*base_frequency (rad/s)
  double duration_{20.0};
  double ramp_time_{2.0};

  // FFS coefficients: ak_[joint][harmonic], bk_[joint][harmonic]
  std::vector<std::vector<double>> ak_;
  std::vector<std::vector<double>> bk_;

  // Initial joint positions (q0)
  std::array<double, kNumJoints> initial_q_{};

  // Joint safety limits
  std::array<double, kNumJoints> joint_position_min_{};
  std::array<double, kNumJoints> joint_position_max_{};
  std::array<double, kNumJoints> joint_velocity_max_{};

  // Runtime state
  rclcpp::Duration elapsed_time_{0, 0};
  bool trajectory_finished_{false};

  // Publisher for trajectory completion signal
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trajectory_done_pub_;
};

}  // namespace franka_7link_dynamics_id
