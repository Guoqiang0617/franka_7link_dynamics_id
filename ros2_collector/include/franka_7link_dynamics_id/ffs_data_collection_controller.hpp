#pragma once

#include <array>
#include <atomic>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <franka/robot_state.h>

namespace franka_7link_dynamics_id {

/**
 * @brief FFS Data Collection Controller
 *
 * Records Franka robot state data (joint positions, velocities, torques,
 * model matrices) to CSV files during FFS trajectory execution.
 * No force sensor data is collected.
 *
 * Uses an asynchronous ring-buffer + writer thread to avoid blocking
 * the real-time control loop with disk I/O.
 */
class FFSDataCollectionController : public controller_interface::ControllerInterface {
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
  static constexpr size_t kSampleBufferCapacity = 16384;
  static constexpr size_t kWriterBatchSize = 256;

  static const std::string k_robot_model_interface_name;
  static const std::string k_robot_state_interface_name;

  struct DataSample {
    int64_t ros_time_ns{0};
    int64_t period_ns{0};
    std::array<double, 7> q{};
    std::array<double, 7> dq{};
    std::array<double, 7> q_d{};
    std::array<double, 7> dq_d{};
    std::array<double, 7> tau_J{};
    std::array<double, 7> tau_J_d{};
    std::array<double, 7> tau_ext_hat_filtered{};
    std::array<double, 16> O_T_EE{};
    std::array<double, 49> mass{};
    std::array<double, 7> coriolis{};
    std::array<double, 7> gravity{};
  };

  // Ring buffer operations
  bool push_sample(const DataSample& sample);
  void writer_loop();
  void write_header();
  void write_sample(const DataSample& sample);

  // Get robot state pointer from hardware interface
  franka::RobotState* get_robot_state_pointer();

  std::string arm_id_;
  std::string output_file_;
  double collection_rate_hz_{1000.0};
  int64_t collection_interval_ns_{1000000};  // 1ms default

  // Franka model interface
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  franka::RobotState* robot_state_{nullptr};

  // CSV file
  std::ofstream csv_file_;

  // Async writer
  std::vector<DataSample> sample_buffer_;
  std::atomic<size_t> write_index_{0};
  std::atomic<size_t> read_index_{0};
  std::atomic<size_t> dropped_samples_{0};
  std::atomic<bool> writer_running_{false};
  std::thread writer_thread_;

  // Timing
  int64_t next_sample_time_ns_{0};
};

}  // namespace franka_7link_dynamics_id
