#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_interfaces/msg/execution_status.hpp"
#include "excavator_visualization/replay_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace excavator_visualization {
namespace {

trajectory_msgs::msg::JointTrajectory ToRosTrajectory(
    const tramcar::planning::JointTrajectory& trajectory) {
  trajectory_msgs::msg::JointTrajectory message;
  message.joint_names = trajectory.joint_names;
  message.points.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points) {
    trajectory_msgs::msg::JointTrajectoryPoint ros_point;
    ros_point.positions = point.positions;
    ros_point.velocities = point.velocities;
    ros_point.accelerations = point.accelerations;
    const double whole_seconds = std::floor(point.t);
    ros_point.time_from_start.sec = static_cast<std::int32_t>(whole_seconds);
    ros_point.time_from_start.nanosec = static_cast<std::uint32_t>(
        std::llround((point.t - whole_seconds) * 1.0e9));
    if (ros_point.time_from_start.nanosec >= 1000000000U) {
      ++ros_point.time_from_start.sec;
      ros_point.time_from_start.nanosec -= 1000000000U;
    }
    message.points.push_back(std::move(ros_point));
  }
  return message;
}

}  // namespace

class TrajectoryReplayNode final : public rclcpp::Node {
 public:
  TrajectoryReplayNode() : Node("excavator_trajectory_replay") {
    const std::string file = declare_parameter<std::string>(
        "trajectory_file", "");
    replay_rate_ = declare_parameter<double>("replay_rate", 1.0);
    publish_joint_states_ =
        declare_parameter<bool>("publish_joint_states", true);
    loop_ = declare_parameter<bool>("loop", false);
    const double state_rate_hz =
        declare_parameter<double>("state_rate_hz", 50.0);
    passive_joint_names_ = declare_parameter<std::vector<std::string>>(
        "passive_joint_names",
        {"boom_cyl_l_body_joint", "boom_cyl_l_rod_joint",
         "boom_cyl_r_body_joint", "boom_cyl_r_rod_joint",
         "stick_cyl_body_joint", "stick_cyl_rod_joint",
         "bucket_cyl_body_joint", "bucket_cyl_rod_joint",
         "bucket_linkage_joint", "stick_rocker_l_joint",
         "stick_rocker_r_joint"});
    passive_joint_positions_ = declare_parameter<std::vector<double>>(
        "passive_joint_positions",
        {-0.30301, -0.23228, -0.30301, -0.23228, 0.04501, -0.36905,
         -0.08259, 0.11620, -0.40, -0.40, -0.40});

    if (file.empty()) {
      throw std::invalid_argument("trajectory_file parameter is required");
    }
    if (!std::isfinite(replay_rate_) || replay_rate_ <= 0.0 ||
        !std::isfinite(state_rate_hz) || state_rate_hz <= 0.0) {
      throw std::invalid_argument(
          "replay_rate and state_rate_hz must be finite and positive");
    }
    if (passive_joint_positions_.size() != passive_joint_names_.size()) {
      throw std::invalid_argument(
          "passive_joint_positions must match passive_joint_names");
    }
    std::string error;
    if (!replay_manager_.LoadTrajectory(file, &trajectory_, &error)) {
      throw std::runtime_error("failed to load replay trajectory: " + error);
    }
    const auto validation =
        tramcar::planning::ValidateJointTrajectory(trajectory_);
    if (!validation.ok) {
      throw std::runtime_error("replay trajectory is invalid: " +
                               validation.message);
    }

    trajectory_publisher_ =
        create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/excavator/planned_trajectory", rclcpp::QoS(1).transient_local());
    joint_state_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    status_publisher_ =
        create_publisher<excavator_interfaces::msg::ExecutionStatus>(
            "/excavator/execution_status", 10);
    trajectory_publisher_->publish(ToRosTrajectory(trajectory_));

    const auto period =
        std::chrono::duration<double>(1.0 / state_rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this, period]() { Tick(period.count()); });
    last_tick_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),
                "Replaying %s (%zu points, %.2f s) at x%.2f",
                file.c_str(), trajectory_.points.size(),
                trajectory_.points.back().t, replay_rate_);
  }

 private:
  void Tick(double nominal_dt) {
    if (!publish_joint_states_) {
      return;
    }
    const auto now_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now_time - last_tick_).count();
    last_tick_ = now_time;
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = nominal_dt;
    }
    dt = std::min(dt, 0.1);
    elapsed_ += dt * replay_rate_;

    const double duration = trajectory_.points.back().t;
    if (elapsed_ > duration) {
      if (!loop_) {
        elapsed_ = duration;
        PublishStatus(elapsed_, true);
        PublishState(trajectory_.points.back(), true);
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "Replay finished");
        return;
      }
      elapsed_ = std::fmod(elapsed_, duration);
    }

    std::size_t segment = 1;
    while (segment < trajectory_.points.size() &&
           trajectory_.points[segment].t < elapsed_) {
      ++segment;
    }
    segment = std::min(segment, trajectory_.points.size() - 1);
    const auto& first = trajectory_.points[segment - 1];
    const auto& second = trajectory_.points[segment];
    const double segment_dt = second.t - first.t;
    const double ratio =
        segment_dt > 0.0
            ? std::clamp((elapsed_ - first.t) / segment_dt, 0.0, 1.0)
            : 0.0;

    // Prefer the velocities/accelerations stored in the trajectory so the
    // replayed state is consistent with the reference values plotted by the
    // RViz panel. Fall back to position slopes only when a CSV omits them.
    const std::size_t joint_count = first.positions.size();
    const bool has_velocities =
        first.velocities.size() == joint_count &&
        second.velocities.size() == joint_count;
    const bool has_accelerations =
        first.accelerations.size() == joint_count &&
        second.accelerations.size() == joint_count;

    tramcar::planning::JointTrajectoryPoint sample;
    sample.t = elapsed_;
    sample.positions.resize(joint_count);
    sample.velocities.resize(joint_count);
    sample.accelerations.resize(joint_count);
    for (std::size_t joint = 0; joint < joint_count; ++joint) {
      sample.positions[joint] =
          first.positions[joint] +
          ratio * (second.positions[joint] - first.positions[joint]);
      sample.velocities[joint] =
          has_velocities
              ? first.velocities[joint] +
                    ratio * (second.velocities[joint] -
                             first.velocities[joint])
              : (segment_dt > 0.0
                     ? (second.positions[joint] -
                        first.positions[joint]) / segment_dt
                     : 0.0);
      sample.accelerations[joint] =
          has_accelerations
              ? first.accelerations[joint] +
                    ratio * (second.accelerations[joint] -
                             first.accelerations[joint])
              : 0.0;
    }
    PublishStatus(elapsed_, false);
    PublishState(sample, false);
  }

  void PublishStatus(double elapsed, bool completed) {
    const double duration = trajectory_.points.back().t;
    excavator_interfaces::msg::ExecutionStatus message;
    message.header.stamp = now();
    message.state = completed
                        ? excavator_interfaces::msg::ExecutionStatus::COMPLETED
                        : excavator_interfaces::msg::ExecutionStatus::RUNNING;
    message.message =
        completed ? "offline replay finished" : "offline replay";
    message.elapsed_time = elapsed;
    message.duration = duration;
    message.progress = duration > 0.0 ? elapsed / duration : 0.0;
    message.tracking_error_norm = 0.0;
    status_publisher_->publish(message);
  }

  void PublishState(const tramcar::planning::JointTrajectoryPoint& point,
                    bool finished) {
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = trajectory_.joint_names;
    state.position = point.positions;
    state.velocity = finished
                         ? tramcar::planning::JointPositions(
                               trajectory_.joint_names.size(), 0.0)
                         : point.velocities;
    for (std::size_t index = 0; index < passive_joint_names_.size();
         ++index) {
      if (std::find(state.name.begin(), state.name.end(),
                    passive_joint_names_[index]) == state.name.end()) {
        state.name.push_back(passive_joint_names_[index]);
        state.position.push_back(passive_joint_positions_[index]);
        state.velocity.push_back(0.0);
      }
    }
    joint_state_publisher_->publish(state);
  }

  tramcar::planning::visualization::ReplayManager replay_manager_;
  tramcar::planning::JointTrajectory trajectory_;
  double replay_rate_{1.0};
  double elapsed_{0.0};
  std::vector<std::string> passive_joint_names_;
  std::vector<double> passive_joint_positions_;
  bool publish_joint_states_{true};
  bool loop_{false};
  std::chrono::steady_clock::time_point last_tick_{};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<excavator_interfaces::msg::ExecutionStatus>::SharedPtr
      status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace excavator_visualization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_visualization::TrajectoryReplayNode>());
  rclcpp::shutdown();
  return 0;
}
