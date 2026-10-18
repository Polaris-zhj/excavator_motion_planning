#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "excavator_sim_viz/kinematic_simulator.hpp"
#include "excavator_interfaces/msg/execution_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace excavator_sim_viz {
namespace {

using Clock = std::chrono::steady_clock;
using JointTrajectoryMessage = trajectory_msgs::msg::JointTrajectory;
using ExecutionStatusMessage = excavator_interfaces::msg::ExecutionStatus;

double ToSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

tramcar::planning::JointTrajectory ToCoreTrajectory(
    const JointTrajectoryMessage& message) {
  tramcar::planning::JointTrajectory trajectory;
  trajectory.joint_names = message.joint_names;
  trajectory.points.reserve(message.points.size());
  for (const auto& message_point : message.points) {
    tramcar::planning::JointTrajectoryPoint point;
    point.t = ToSeconds(message_point.time_from_start);
    point.positions = message_point.positions;
    point.velocities = message_point.velocities;
    point.accelerations = message_point.accelerations;
    trajectory.points.push_back(std::move(point));
  }
  return trajectory;
}

}  // namespace

class TrajectoryPlayer final : public rclcpp::Node {
 public:
  TrajectoryPlayer()
      : Node("excavator_trajectory_player"),
        simulator_({"swing_joint", "boom_joint", "stick_joint", "bucket_joint"},
                   {0.0, -0.35, 0.45, -0.4}) {
    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 50.0);
    passive_joint_names_ = declare_parameter<std::vector<std::string>>(
        "passive_joint_names",
        {"bucket_linkage_joint", "stick_rocker_l_joint",
         "stick_rocker_r_joint"});
    if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and positive");
    }

    joint_state_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    status_publisher_ = create_publisher<ExecutionStatusMessage>(
        "/execution_status", 10);
    PublishStatus(ExecutionStatusMessage::IDLE, "ready", 0.0);
    trajectory_subscription_ = create_subscription<JointTrajectoryMessage>(
        "/joint_trajectory", 10,
        [this](JointTrajectoryMessage::ConstSharedPtr message) {
          ReceiveTrajectory(std::move(message));
        });
    cancel_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/cancel_trajectory", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          std::lock_guard<std::mutex> lock(mutex_);
          simulator_.CancelTrajectory();
          last_step_ = Clock::now();
          PublishStatus(ExecutionStatusMessage::CANCELED,
                        "trajectory canceled by request", 0.0);
        });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { PublishCurrentState(); });
    last_step_ = Clock::now();

    RCLCPP_INFO(get_logger(),
                "Kinematic simulator ready: /joint_trajectory -> /joint_states "
                "(%.1f Hz)",
                publish_rate_hz);
  }

 private:
  void PublishStatus(std::uint8_t state, const std::string& message,
                     double elapsed_time) {
    if (last_status_state_ == state && state != ExecutionStatusMessage::RUNNING) {
      return;
    }
    last_status_state_ = state;
    ExecutionStatusMessage status;
    status.header.stamp = now();
    status.state = state;
    status.message = message;
    status.elapsed_time = elapsed_time;
    status.tracking_error_norm = 0.0;
    status_publisher_->publish(status);
  }

  void ReceiveTrajectory(JointTrajectoryMessage::ConstSharedPtr message) {
    auto trajectory = ToCoreTrajectory(*message);
    const auto validation = tramcar::planning::ValidateJointTrajectory(trajectory);
    if (!validation.ok) {
      RCLCPP_WARN(get_logger(), "Rejected trajectory: %s",
                  validation.message.c_str());
      PublishStatus(ExecutionStatusMessage::REJECTED, validation.message, 0.0);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = simulator_.SubmitTrajectory(trajectory);
    if (!result.ok) {
      RCLCPP_WARN(get_logger(), "Rejected trajectory: %s",
                  result.message.c_str());
      PublishStatus(ExecutionStatusMessage::REJECTED, result.message, 0.0);
      return;
    }
    last_step_ = Clock::now();
    PublishStatus(ExecutionStatusMessage::RUNNING, "trajectory accepted", 0.0);
    RCLCPP_INFO(get_logger(), "Accepted trajectory with %zu joints and %zu points",
                trajectory.joint_names.size(), trajectory.points.size());
  }

  void PublishCurrentState() {
    SimulationState simulation_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto current_time = Clock::now();
      const double dt =
          std::chrono::duration<double>(current_time - last_step_).count();
      last_step_ = current_time;
      const auto result = simulator_.Step(dt, &simulation_state);
      if (!result.ok) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Simulator step failed: %s",
                              result.message.c_str());
        return;
      }
    }

    if (simulation_state.trajectory_active) {
      PublishStatus(ExecutionStatusMessage::RUNNING, "trajectory executing",
                    simulation_state.elapsed_time);
    } else if (simulation_state.trajectory_finished) {
      PublishStatus(ExecutionStatusMessage::SUCCEEDED, "trajectory completed",
                    simulation_state.elapsed_time);
    }

    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = simulation_state.joint_names;
    state.position = simulation_state.positions;
    state.velocity = simulation_state.velocities;
    for (const std::string& joint_name : passive_joint_names_) {
      if (std::find(state.name.begin(), state.name.end(), joint_name) ==
          state.name.end()) {
        state.name.push_back(joint_name);
        state.position.push_back(0.0);
        state.velocity.push_back(0.0);
      }
    }
    joint_state_publisher_->publish(state);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<ExecutionStatusMessage>::SharedPtr status_publisher_;
  std::uint8_t last_status_state_{255};
  rclcpp::Subscription<JointTrajectoryMessage>::SharedPtr
      trajectory_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  KinematicSimulator simulator_;
  std::vector<std::string> passive_joint_names_;
  Clock::time_point last_step_{};
};

}  // namespace excavator_sim_viz

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_sim_viz::TrajectoryPlayer>());
  rclcpp::shutdown();
  return 0;
}
