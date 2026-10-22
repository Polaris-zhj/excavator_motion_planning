#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_interfaces/action/execute_trajectory.hpp"
#include "excavator_visualization/replay_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace excavator_visualization {
namespace {

using ExecuteAction = excavator_interfaces::action::ExecuteTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteAction>;

bool IsAllZero(const std::vector<double>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return value == 0.0; });
}

void StripZeroVelocityAcceleration(
    tramcar::planning::JointTrajectory* trajectory) {
  const bool has_motion_profile = std::any_of(
      trajectory->points.begin(), trajectory->points.end(),
      [](const tramcar::planning::JointTrajectoryPoint& point) {
        return !IsAllZero(point.velocities) ||
               !IsAllZero(point.accelerations);
      });
  if (has_motion_profile) {
    return;
  }
  for (auto& point : trajectory->points) {
    point.velocities.clear();
    point.accelerations.clear();
  }
}

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

class CsvExecuteNode final : public rclcpp::Node {
 public:
  CsvExecuteNode() : Node("excavator_csv_execute") {
    const std::string file =
        declare_parameter<std::string>("trajectory_file", "");
    initialize_to_start_ = declare_parameter<bool>("initialize_to_start", false);
    const std::string action_name =
        declare_parameter<std::string>("action_name",
                                       "/excavator/execute_trajectory");
    const double server_timeout_sec =
        declare_parameter<double>("server_timeout_sec", 5.0);

    if (file.empty()) {
      throw std::invalid_argument("trajectory_file parameter is required");
    }

    tramcar::planning::JointTrajectory trajectory;
    std::string error;
    if (!replay_manager_.LoadTrajectory(file, &trajectory, &error)) {
      throw std::runtime_error("failed to load trajectory: " + error);
    }
    StripZeroVelocityAcceleration(&trajectory);
    const auto validation =
        tramcar::planning::ValidateJointTrajectory(trajectory);
    if (!validation.ok) {
      throw std::runtime_error("trajectory is invalid: " + validation.message);
    }

    trajectory_ = ToRosTrajectory(trajectory);
    const bool position_only = trajectory_.points.empty() ||
                               trajectory_.points.front().velocities.empty();
    RCLCPP_INFO(get_logger(),
                "Loaded trajectory from %s: %zu points, %.2f s, %zu joints%s",
                file.c_str(), trajectory_.points.size(),
                trajectory.points.back().t, trajectory_.joint_names.size(),
                position_only ? " (position-only)" : "");

    planned_trajectory_publisher_ =
        create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/excavator/planned_trajectory", rclcpp::QoS(1).transient_local());
    planned_trajectory_publisher_->publish(trajectory_);
    RCLCPP_INFO(get_logger(),
                "Published trajectory to /excavator/planned_trajectory");

    client_ = rclcpp_action::create_client<ExecuteAction>(this, action_name);
    if (!client_->wait_for_action_server(
            std::chrono::duration<double>(server_timeout_sec))) {
      throw std::runtime_error("execute action server not available at " +
                               action_name);
    }
    SendGoal();
  }

 private:
  void SendGoal() {
    ExecuteAction::Goal goal;
    goal.trajectory = trajectory_;
    goal.require_confirmation = true;
    goal.initialize_to_start = initialize_to_start_;

    rclcpp_action::Client<ExecuteAction>::SendGoalOptions options;
    options.goal_response_callback =
        [this](const GoalHandle::SharedPtr& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Execution goal rejected");
          } else {
            RCLCPP_INFO(get_logger(), "Execution goal accepted");
          }
        };
    options.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const ExecuteAction::Feedback> feedback) {
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 500,
              "Execution feedback: state=%u progress=%.2f%% (%s)",
              feedback->state, feedback->progress * 100.0F,
              feedback->message.c_str());
        };
    options.result_callback =
        [this](const GoalHandle::WrappedResult& result) {
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(get_logger(), "Execution succeeded: %s",
                          result.result ? result.result->message.c_str()
                                        : "");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(get_logger(), "Execution canceled");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(get_logger(), "Execution aborted: %s",
                           result.result ? result.result->message.c_str()
                                         : "no result");
              break;
            default:
              RCLCPP_ERROR(get_logger(), "Execution failed with unknown code");
              break;
          }
          rclcpp::shutdown();
        };

    RCLCPP_INFO(get_logger(), "Sending %zu-point trajectory for execution "
                "(initialize_to_start=%s)",
                trajectory_.points.size(),
                initialize_to_start_ ? "true" : "false");
    client_->async_send_goal(goal, options);
  }

  tramcar::planning::visualization::ReplayManager replay_manager_;
  trajectory_msgs::msg::JointTrajectory trajectory_;
  bool initialize_to_start_{false};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      planned_trajectory_publisher_;
  rclcpp_action::Client<ExecuteAction>::SharedPtr client_;
};

}  // namespace excavator_visualization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<excavator_visualization::CsvExecuteNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("csv_execute"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
