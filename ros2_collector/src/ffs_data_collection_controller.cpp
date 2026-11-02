#include <franka_7link_dynamics_id/ffs_data_collection_controller.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <iomanip>
#include <type_traits>

namespace {

template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value &&
                     std::is_trivially_copyable<To>::value,
                 To>
bit_cast(const From& src) noexcept {
  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

template <class T, size_t N>
void write_array(std::ostream& stream, const std::array<T, N>& values) {
  for (const auto& value : values) {
    stream << ',' << value;
  }
}

void write_names(std::ostream& stream, const std::string& prefix, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    stream << ',' << prefix << '_' << i;
  }
}

}  // namespace

namespace franka_7link_dynamics_id {

const std::string FFSDataCollectionController::k_robot_model_interface_name = "robot_model";
const std::string FFSDataCollectionController::k_robot_state_interface_name = "robot_state";

controller_interface::CallbackReturn FFSDataCollectionController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::string>("output_file", "/tmp/ffs_data_collection.csv");
    auto_declare<double>("collection_rate_hz", 1000.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FFSDataCollectionController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FFSDataCollectionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (franka_robot_model_) {
    for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
      state_interfaces_config.names.push_back(franka_robot_model_name);
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn FFSDataCollectionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  output_file_ = get_node()->get_parameter("output_file").as_string();

  collection_rate_hz_ = get_node()->get_parameter("collection_rate_hz").as_double();
  if (!std::isfinite(collection_rate_hz_) || collection_rate_hz_ <= 0.0 ||
      collection_rate_hz_ > 1000.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "collection_rate_hz must be in (0, 1000].");
    return CallbackReturn::ERROR;
  }
  collection_interval_ns_ =
      std::max<int64_t>(1, static_cast<int64_t>(std::llround(1000000000.0 / collection_rate_hz_)));
  sample_buffer_.resize(kSampleBufferCapacity);
  write_index_.store(0, std::memory_order_relaxed);
  read_index_.store(0, std::memory_order_relaxed);
  dropped_samples_.store(0, std::memory_order_relaxed);

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/" + k_robot_model_interface_name, arm_id_ + "/" + k_robot_state_interface_name);

  RCLCPP_INFO(get_node()->get_logger(), "FFS Data Collection configured: file=%s, rate=%.1f Hz",
              output_file_.c_str(), collection_rate_hz_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FFSDataCollectionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  output_file_ = get_node()->get_parameter("output_file").as_string();

  // Re-read parameters changed while inactive, before opening the new recording.
  collection_rate_hz_ = get_node()->get_parameter("collection_rate_hz").as_double();
  if (!std::isfinite(collection_rate_hz_) || collection_rate_hz_ <= 0.0 ||
      collection_rate_hz_ > 1000.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "collection_rate_hz must be in (0, 1000].");
    return CallbackReturn::ERROR;
  }
  collection_interval_ns_ =
      std::max<int64_t>(1, static_cast<int64_t>(std::llround(1000000000.0 / collection_rate_hz_)));

  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }

  csv_file_.open(output_file_, std::ios::out | std::ios::trunc);
  if (!csv_file_.is_open()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to open output CSV file: %s",
                 output_file_.c_str());
    return CallbackReturn::ERROR;
  }
  csv_file_ << std::setprecision(10);
  write_header();
  csv_file_.flush();

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  robot_state_ = get_robot_state_pointer();
  if (robot_state_ == nullptr) {
    franka_robot_model_->release_interfaces();
    return CallbackReturn::ERROR;
  }

  write_index_.store(0, std::memory_order_relaxed);
  read_index_.store(0, std::memory_order_relaxed);
  dropped_samples_.store(0, std::memory_order_relaxed);
  next_sample_time_ns_ = 0;

  writer_running_.store(true, std::memory_order_release);
  writer_thread_ = std::thread(&FFSDataCollectionController::writer_loop, this);

  RCLCPP_INFO(get_node()->get_logger(), "FFS Data Collection activated. Recording to: %s",
              output_file_.c_str());
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FFSDataCollectionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  writer_running_.store(false, std::memory_order_release);
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  const auto dropped = dropped_samples_.load(std::memory_order_relaxed);
  if (dropped > 0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "FFS Data Collection dropped %lu samples (buffer full).",
                static_cast<unsigned long>(dropped));
  }

  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }

  if (franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }
  robot_state_ = nullptr;

  RCLCPP_INFO(get_node()->get_logger(), "FFS Data Collection deactivated. File saved.");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FFSDataCollectionController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  if (robot_state_ == nullptr || sample_buffer_.empty()) {
    return controller_interface::return_type::ERROR;
  }

  const int64_t now_ns = time.nanoseconds();
  if (next_sample_time_ns_ == 0) {
    next_sample_time_ns_ = now_ns;
  }

  if (now_ns < next_sample_time_ns_) {
    return controller_interface::return_type::OK;
  }

  do {
    next_sample_time_ns_ += collection_interval_ns_;
  } while (now_ns >= next_sample_time_ns_);

  DataSample sample;
  sample.ros_time_ns = now_ns;
  sample.period_ns = period.nanoseconds();

  sample.q = robot_state_->q;
  sample.dq = robot_state_->dq;
  sample.q_d = robot_state_->q_d;
  sample.dq_d = robot_state_->dq_d;
  sample.tau_J = robot_state_->tau_J;
  sample.tau_J_d = robot_state_->tau_J_d;
  sample.tau_ext_hat_filtered = robot_state_->tau_ext_hat_filtered;
  sample.O_T_EE = robot_state_->O_T_EE;

  sample.mass = franka_robot_model_->getMassMatrix();
  sample.coriolis = franka_robot_model_->getCoriolisForceVector();
  sample.gravity = franka_robot_model_->getGravityForceVector();

  if (!push_sample(sample)) {
    dropped_samples_.fetch_add(1, std::memory_order_relaxed);
  }

  return controller_interface::return_type::OK;
}

bool FFSDataCollectionController::push_sample(const DataSample& sample) {
  const auto wi = write_index_.load(std::memory_order_relaxed);
  const auto next_wi = (wi + 1) % sample_buffer_.size();
  const auto ri = read_index_.load(std::memory_order_acquire);

  if (next_wi == ri) {
    return false;
  }

  sample_buffer_[wi] = sample;
  write_index_.store(next_wi, std::memory_order_release);
  return true;
}

void FFSDataCollectionController::writer_loop() {
  while (writer_running_.load(std::memory_order_acquire) ||
         read_index_.load(std::memory_order_relaxed) !=
             write_index_.load(std::memory_order_acquire)) {
    size_t written_in_batch = 0;

    while (written_in_batch < kWriterBatchSize) {
      const auto ri = read_index_.load(std::memory_order_relaxed);
      const auto wi = write_index_.load(std::memory_order_acquire);

      if (ri == wi) {
        break;
      }

      write_sample(sample_buffer_[ri]);

      const auto next_ri = (ri + 1) % sample_buffer_.size();
      read_index_.store(next_ri, std::memory_order_release);

      ++written_in_batch;
    }

    if (written_in_batch > 0) {
      csv_file_.flush();
    } else {
      std::this_thread::yield();
    }
  }

  csv_file_.flush();
}

void FFSDataCollectionController::write_header() {
  csv_file_ << "ros_time_ns,period_ns";
  write_names(csv_file_, "q", 7);
  write_names(csv_file_, "dq", 7);
  write_names(csv_file_, "q_d", 7);
  write_names(csv_file_, "dq_d", 7);
  write_names(csv_file_, "tau_J", 7);
  write_names(csv_file_, "tau_J_d", 7);
  write_names(csv_file_, "tau_ext_hat_filtered", 7);
  write_names(csv_file_, "O_T_EE", 16);
  write_names(csv_file_, "mass", 49);
  write_names(csv_file_, "coriolis", 7);
  write_names(csv_file_, "gravity", 7);
  csv_file_ << '\n';
}

void FFSDataCollectionController::write_sample(const DataSample& sample) {
  csv_file_ << sample.ros_time_ns << ',' << sample.period_ns;
  write_array(csv_file_, sample.q);
  write_array(csv_file_, sample.dq);
  write_array(csv_file_, sample.q_d);
  write_array(csv_file_, sample.dq_d);
  write_array(csv_file_, sample.tau_J);
  write_array(csv_file_, sample.tau_J_d);
  write_array(csv_file_, sample.tau_ext_hat_filtered);
  write_array(csv_file_, sample.O_T_EE);
  write_array(csv_file_, sample.mass);
  write_array(csv_file_, sample.coriolis);
  write_array(csv_file_, sample.gravity);
  csv_file_ << '\n';
}

franka::RobotState* FFSDataCollectionController::get_robot_state_pointer() {
  const std::string full_interface_name = arm_id_ + "/" + k_robot_state_interface_name;

  auto franka_state_interface =
      std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                   [&full_interface_name](const auto& interface) {
                     return interface.get_name() == full_interface_name;
                   });

  if (franka_state_interface == state_interfaces_.cend()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Franka robot_state interface does not exist.");
    return nullptr;
  }

  return bit_cast<franka::RobotState*>((*franka_state_interface).get_value());
}

}  // namespace franka_7link_dynamics_id

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_7link_dynamics_id::FFSDataCollectionController,
                       controller_interface::ControllerInterface)
