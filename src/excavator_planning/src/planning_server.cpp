#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "excavator_common/planner_interfaces.hpp"
#include "excavator_interfaces/action/plan_trajectory.hpp"
#include "kinematics/urdf_kinematics_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace excavator_planning {
using namespace tramcar::planning;
namespace {

using PlanAction = excavator_interfaces::action::PlanTrajectory;
using GoalHandlePlan = rclcpp_action::ServerGoalHandle<PlanAction>;
double ToSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

bool ReadPositions(const sensor_msgs::msg::JointState& state,
                   const std::vector<std::string>& joint_names,
                   JointPositions* positions, std::string* error) {
  if (positions == nullptr || error == nullptr ||
      state.name.size() != state.position.size()) {
    if (error != nullptr) {
      *error = "JointState name and position arrays must have equal size";
    }
    return false;
  }
  std::unordered_map<std::string, double> values;
  for (std::size_t i = 0; i < state.name.size(); ++i) {
    if (!std::isfinite(state.position[i])) {
      *error = "JointState contains a non-finite joint position";
      return false;
    }
    values[state.name[i]] = state.position[i];
  }
  positions->clear();
  positions->reserve(joint_names.size());
  for (const std::string& name : joint_names) {
    const auto found = values.find(name);
    if (found == values.end()) {
      *error = "JointState is missing planning joint: " + name;
      return false;
    }
    positions->push_back(found->second);
  }
  return true;
}

trajectory_msgs::msg::JointTrajectory ToRosTrajectory(
    const JointTrajectory& trajectory) {
  trajectory_msgs::msg::JointTrajectory message;
  message.joint_names = trajectory.joint_names;
  message.points.reserve(trajectory.points.size());
  for (const JointTrajectoryPoint& point : trajectory.points) {
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

class PlanningServer final : public rclcpp::Node {
 public:
  PlanningServer()
      : Node("planning_server"),
        joint_names_(declare_parameter<std::vector<std::string>>(
            "joint_names",
            {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"})),
        kinematics_(joint_names_) {
    const std::string robot_description =
        declare_parameter<std::string>("robot_description", "");
    const double max_joint_step =
        declare_parameter<double>("max_joint_step", 0.05);
    max_acceleration_ = declare_parameter<std::vector<double>>(
        "max_acceleration", {1.0, 0.6, 0.8, 1.0});

    const auto load_result = kinematics_.InitializeFromString(robot_description);
    if (!load_result.ok) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize model: %s",
                   load_result.message.c_str());
    } else {
      const auto position_limits = kinematics_.GetPositionLimits();
      max_velocity_ = kinematics_.GetVelocityLimits();
      if (position_limits.size() != joint_names_.size() ||
          max_velocity_.size() != joint_names_.size() ||
          max_acceleration_.size() != joint_names_.size() ||
          max_joint_step <= 0.0) {
        RCLCPP_ERROR(get_logger(),
                     "Planning joint limits or max_joint_step are invalid");
      } else {
        path_planner_ = std::make_unique<LinearPathPlanner>(
            joint_names_, position_limits, max_joint_step);
        initialized_ = true;
      }
    }

    action_server_ = rclcpp_action::create_server<PlanAction>(
        this, "/plan_trajectory",
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const PlanAction::Goal> goal) {
          return HandleGoal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<GoalHandlePlan> goal_handle) {
          return HandleCancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandlePlan> goal_handle) {
          HandleAccepted(goal_handle);
        });
  }

 private:
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const PlanAction::Goal>) {
    return initialized_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                       : rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandlePlan>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandlePlan> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanAction::Result>();
    auto feedback = std::make_shared<PlanAction::Feedback>();

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "planning request canceled";
      goal_handle->canceled(result);
      return;
    }

    if (!goal->planning_scene.objects.empty()) {
      result->success = false;
      result->message =
          "planning scene contains obstacles but collision checking is not "
          "implemented; request rejected rather than claiming a safe path";
      goal_handle->abort(result);
      return;
    }

    JointPositions start;
    JointPositions target;
    std::string error;
    if (!ReadPositions(goal->start_state, joint_names_, &start, &error) ||
        !ReadPositions(goal->goal_state, joint_names_, &target, &error)) {
      result->success = false;
      result->message = error;
      goal_handle->abort(result);
      return;
    }
    const double allowed_time = ToSeconds(goal->allowed_planning_time);
    if (allowed_time < 0.0 || !std::isfinite(allowed_time)) {
      result->success = false;
      result->message = "allowed_planning_time must be finite and non-negative";
      goal_handle->abort(result);
      return;
    }

    feedback->stage = "joint_space_path";
    feedback->progress = 0.1F;
    feedback->message = "generating baseline straight-line path";
    goal_handle->publish_feedback(feedback);

    PlanningRequest request;
    request.start = std::move(start);
    request.goal = std::move(target);
    request.group_name = "excavator_arm";
    request.timeout_seconds = allowed_time > 0.0 ? allowed_time : 1.0;
    const PlanningResponse path_response = path_planner_->Plan(request);
    if (!path_response.success) {
      result->success = false;
      result->message = path_response.message;
      goal_handle->abort(result);
      return;
    }

    feedback->stage = "time_parameterization";
    feedback->progress = 0.6F;
    feedback->message = "assigning conservative segment durations";
    goal_handle->publish_feedback(feedback);

    TimeParameterizationRequest time_request;
    time_request.path = path_response.path;
    time_request.max_velocity = max_velocity_;
    time_request.max_acceleration = max_acceleration_;
    const TimeParameterizationResponse time_response =
        SimpleTimeParameterizer().Parameterize(time_request);
    if (!time_response.success) {
      result->success = false;
      result->message = time_response.message;
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "planning request canceled";
      goal_handle->canceled(result);
      return;
    }
    result->success = true;
    result->trajectory = ToRosTrajectory(time_response.trajectory);
    result->message =
        "baseline trajectory generated; collision avoidance and final "
        "collision validation are not implemented";
    feedback->stage = "complete";
    feedback->progress = 1.0F;
    feedback->message = result->message;
    goal_handle->publish_feedback(feedback);
    goal_handle->succeed(result);
  }

  std::vector<std::string> joint_names_;
  UrdfKinematicsModel kinematics_;
  std::vector<double> max_velocity_;
  std::vector<double> max_acceleration_;
  std::unique_ptr<LinearPathPlanner> path_planner_;
  bool initialized_{false};
  rclcpp_action::Server<PlanAction>::SharedPtr action_server_;
};

}  // namespace excavator_planning

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_planning::PlanningServer>());
  rclcpp::shutdown();
  return 0;
}
