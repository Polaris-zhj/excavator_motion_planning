#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "excavator_sim_viz/kinematic_simulator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace excavator_sim_viz {
namespace {

using Clock = std::chrono::steady_clock;
using JointTrajectoryMessage = trajectory_msgs::msg::JointTrajectory;

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
    trajectory_subscription_ = create_subscription<JointTrajectoryMessage>(
        "/joint_trajectory", 10,
        [this](JointTrajectoryMessage::ConstSharedPtr message) {
          ReceiveTrajectory(std::move(message));
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
  void ReceiveTrajectory(JointTrajectoryMessage::ConstSharedPtr message) {
    auto trajectory = ToCoreTrajectory(*message);
    const auto validation = tramcar::planning::ValidateJointTrajectory(trajectory);
    if (!validation.ok) {
      RCLCPP_WARN(get_logger(), "Rejected trajectory: %s",
                  validation.message.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = simulator_.SubmitTrajectory(trajectory);
    if (!result.ok) {
      RCLCPP_WARN(get_logger(), "Rejected trajectory: %s",
                  result.message.c_str());
      return;
    }
    last_step_ = Clock::now();
    RCLCPP_INFO(get_logger(), "Accepted trajectory with %zu joints and %zu points",
                trajectory.joint_names.size(), trajectory.points.size());
  }

  void PublishCurrentState() {
    const auto current_time = Clock::now();
    const double dt =
        std::chrono::duration<double>(current_time - last_step_).count();
    last_step_ = current_time;

    SimulationState simulation_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto result = simulator_.Step(dt, &simulation_state);
      if (!result.ok) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Simulator step failed: %s",
                              result.message.c_str());
        return;
      }
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
  rclcpp::Subscription<JointTrajectoryMessage>::SharedPtr
      trajectory_subscription_;
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
