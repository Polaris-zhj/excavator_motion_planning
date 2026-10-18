#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace excavator_sim_viz {
namespace {

using Clock = std::chrono::steady_clock;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;

double ToSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

}  // namespace

class TrajectoryPlayer final : public rclcpp::Node {
 public:
  TrajectoryPlayer() : Node("excavator_trajectory_player") {
    const auto publish_rate_hz =
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
    trajectory_subscription_ = create_subscription<JointTrajectory>(
        "/joint_trajectory", 10,
        [this](JointTrajectory::ConstSharedPtr message) {
          ReceiveTrajectory(std::move(message));
        });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { PublishCurrentState(); });

    RCLCPP_INFO(get_logger(),
                "Trajectory player ready; subscribe to /joint_trajectory and "
                "publishes /joint_states at %.1f Hz",
                publish_rate_hz);
  }

 private:
  void ReceiveTrajectory(JointTrajectory::ConstSharedPtr message) {
    if (message->joint_names.empty() || message->points.empty()) {
      RCLCPP_WARN(get_logger(),
                  "Ignoring trajectory with empty joint_names or points");
      return;
    }

    double previous_time = -1.0;
    for (const JointTrajectoryPoint& point : message->points) {
      const double time = ToSeconds(point.time_from_start);
      if (!std::isfinite(time) || time < 0.0 ||
          (previous_time >= 0.0 && time <= previous_time) ||
          point.positions.size() != message->joint_names.size()) {
        RCLCPP_WARN(get_logger(),
                    "Ignoring malformed trajectory: times must increase and "
                    "each positions array must match joint_names");
        return;
      }
      for (double position : point.positions) {
        if (!std::isfinite(position)) {
          RCLCPP_WARN(get_logger(),
                      "Ignoring trajectory containing a non-finite position");
          return;
        }
      }
      previous_time = time;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    trajectory_ = *message;
    trajectory_start_ = Clock::now();
    has_trajectory_ = true;
    RCLCPP_INFO(get_logger(), "Accepted trajectory with %zu joints and %zu points",
                trajectory_.joint_names.size(), trajectory_.points.size());
  }

  void PublishCurrentState() {
    JointTrajectory trajectory;
    Clock::time_point start;
    bool has_trajectory = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_trajectory = has_trajectory_;
      if (has_trajectory) {
        trajectory = trajectory_;
        start = trajectory_start_;
      }
    }
    if (!has_trajectory) {
      return;
    }

    const double elapsed =
        std::chrono::duration<double>(Clock::now() - start).count();
    const auto& points = trajectory.points;
    std::vector<double> positions = points.front().positions;
    std::vector<double> velocities(trajectory.joint_names.size(), 0.0);

    if (elapsed >= ToSeconds(points.back().time_from_start)) {
      positions = points.back().positions;
    } else {
      for (std::size_t index = 1; index < points.size(); ++index) {
        const double t0 = ToSeconds(points[index - 1].time_from_start);
        const double t1 = ToSeconds(points[index].time_from_start);
        if (elapsed <= t1) {
          const double duration = t1 - t0;
          const double ratio =
              std::clamp((elapsed - t0) / duration, 0.0, 1.0);
          for (std::size_t joint = 0; joint < positions.size(); ++joint) {
            const double delta = points[index].positions[joint] -
                                 points[index - 1].positions[joint];
            positions[joint] =
                points[index - 1].positions[joint] + ratio * delta;
            velocities[joint] = delta / duration;
          }
          break;
        }
      }
    }

    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = trajectory.joint_names;
    state.position = std::move(positions);
    state.velocity = std::move(velocities);
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
  rclcpp::Subscription<JointTrajectory>::SharedPtr trajectory_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  JointTrajectory trajectory_;
  std::vector<std::string> passive_joint_names_;
  Clock::time_point trajectory_start_{};
  bool has_trajectory_{false};
};

}  // namespace excavator_sim_viz

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_sim_viz::TrajectoryPlayer>());
  rclcpp::shutdown();
  return 0;
}
