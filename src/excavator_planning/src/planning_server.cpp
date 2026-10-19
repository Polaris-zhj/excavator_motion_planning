#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "excavator_common/planner_interfaces.hpp"
#include "excavator_collision/collision_checker.hpp"
#include "excavator_interfaces/action/plan_trajectory.hpp"
#include "excavator_planning/baseline_planners.hpp"
#include "excavator_planning/path_post_processor.hpp"
#include "excavator_planning/planner_plugin.hpp"
#include "pluginlib/class_loader.hpp"
#include "kinematics/urdf_kinematics_model.hpp"
#include "kinematics/workspace_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace excavator_planning {
using namespace tramcar::planning;
namespace {

using PlanAction = excavator_interfaces::action::PlanTrajectory;
using GoalHandlePlan = rclcpp_action::ServerGoalHandle<PlanAction>;
using Clock = std::chrono::steady_clock;

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
  std::unordered_set<std::string> seen_names;
  for (std::size_t i = 0; i < state.name.size(); ++i) {
    if (state.name[i].empty()) {
      *error = "JointState contains an empty joint name";
      return false;
    }
    if (!seen_names.insert(state.name[i]).second) {
      *error = "JointState contains duplicate joint names";
      return false;
    }
    if (!std::isfinite(state.position[i])) {
      *error = "JointState contains a non-finite joint position";
      return false;
    }
    values.emplace(state.name[i], state.position[i]);
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

JointPath MakeStraightPath(const JointPositions& start,
                           const JointPositions& goal,
                           const std::vector<std::string>& joint_names,
                           double max_joint_step) {
  JointPath path;
  path.joint_names = joint_names;
  double max_delta = 0.0;
  for (std::size_t joint = 0; joint < start.size(); ++joint) {
    max_delta = std::max(max_delta, std::abs(goal[joint] - start[joint]));
  }
  const std::size_t steps = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(max_delta / max_joint_step)));
  path.points.reserve(steps + 1);
  for (std::size_t step = 0; step <= steps; ++step) {
    const double ratio =
        static_cast<double>(step) / static_cast<double>(steps);
    JointPositions positions(start.size(), 0.0);
    for (std::size_t joint = 0; joint < start.size(); ++joint) {
      positions[joint] =
          start[joint] + ratio * (goal[joint] - start[joint]);
    }
    path.points.push_back(std::move(positions));
  }
  return path;
}

bool NormalizePose(const geometry_msgs::msg::Pose& message, Pose3* pose,
                   std::string* error) {
  const double quaternion_norm =
      message.orientation.x * message.orientation.x +
      message.orientation.y * message.orientation.y +
      message.orientation.z * message.orientation.z +
      message.orientation.w * message.orientation.w;
  if (!std::isfinite(message.position.x) ||
      !std::isfinite(message.position.y) ||
      !std::isfinite(message.position.z) ||
      !std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-9) {
    *error = "pose contains invalid position or orientation";
    return false;
  }
  pose->position.x = message.position.x;
  pose->position.y = message.position.y;
  pose->position.z = message.position.z;
  const double inverse_norm = 1.0 / std::sqrt(quaternion_norm);
  pose->orientation.x = message.orientation.x * inverse_norm;
  pose->orientation.y = message.orientation.y * inverse_norm;
  pose->orientation.z = message.orientation.z * inverse_norm;
  pose->orientation.w = message.orientation.w * inverse_norm;
  return true;
}

std_msgs::msg::ColorRGBA MakeColor(float red, float green, float blue,
                                   float alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
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
    planner_class_ = declare_parameter<std::string>(
        "planner_class", "excavator_planning/LinearPlannerPlugin");
    const double max_joint_step =
        declare_parameter<double>("max_joint_step", 0.05);
    max_collision_joint_step_ =
        declare_parameter<double>("max_collision_joint_step", 0.03);
    max_acceleration_ = declare_parameter<std::vector<double>>(
        "max_acceleration", {1.0, 0.6, 0.8, 1.0});
    const int max_detour_attempts =
        declare_parameter<int>("max_detour_attempts", 12);
    simplify_path_ = declare_parameter<bool>("simplify_path", true);
    smooth_path_ = declare_parameter<bool>("smooth_path", false);
    smoothing_iterations_ =
        declare_parameter<int>("smoothing_iterations", 1);
    smoothing_alpha_ = declare_parameter<double>("smoothing_alpha", 0.2);
    workspace_link_ =
        declare_parameter<std::string>("workspace_link", "bucket_tip");
    const std::vector<double> workspace_min =
        declare_parameter<std::vector<double>>("workspace_min",
                                               std::vector<double>{});
    const std::vector<double> workspace_max =
        declare_parameter<std::vector<double>>("workspace_max",
                                               std::vector<double>{});
    const double workspace_margin =
        declare_parameter<double>("workspace_margin", 0.05);
    const int planner_max_iterations =
        declare_parameter<int>("planner_max_iterations", 2000);
    const double planner_step_size =
        declare_parameter<double>("planner_step_size", 0.25);
    const double planner_goal_bias =
        declare_parameter<double>("planner_goal_bias", 0.10);
    const double planner_rewire_radius =
        declare_parameter<double>("planner_rewire_radius", 0.60);
    const double planner_goal_tolerance =
        declare_parameter<double>("planner_goal_tolerance", 0.15);
    const int planner_random_seed =
        declare_parameter<int>("planner_random_seed", 42);
    const std::vector<std::string> allowed_collision_pairs =
        declare_parameter<std::vector<std::string>>(
            "allowed_collision_pairs",
            {"base_link:boom_link", "base_link:stick_link",
             "base_link:bucket_link", "swing_link:stick_link",
             "swing_link:bucket_link"});

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
      } else if (max_collision_joint_step_ <= 0.0 ||
                 !std::isfinite(max_collision_joint_step_)) {
        RCLCPP_ERROR(get_logger(),
                     "max_collision_joint_step must be finite and positive");
      } else {
        const auto collision_init = collision_checker_.Initialize(
            robot_description, joint_names_);
        if (!collision_init.ok) {
          RCLCPP_ERROR(get_logger(),
                       "Failed to initialize collision checker: %s",
                       collision_init.message.c_str());
        } else {
          std::vector<std::pair<std::string, std::string>> allowed_pairs;
          for (const std::string& entry : allowed_collision_pairs) {
            const std::size_t separator = entry.find(':');
            if (separator == std::string::npos || separator == 0 ||
                separator + 1 == entry.size()) {
              RCLCPP_WARN(get_logger(),
                          "Ignoring malformed allowed_collision_pairs entry: %s",
                          entry.c_str());
              continue;
            }
            allowed_pairs.emplace_back(entry.substr(0, separator),
                                       entry.substr(separator + 1));
          }
          collision_checker_.SetAllowedCollisionPairs(allowed_pairs);
          if (max_detour_attempts < 0 || planner_max_iterations <= 0 ||
              !std::isfinite(planner_step_size) ||
              planner_step_size <= 0.0 ||
              !std::isfinite(planner_goal_bias) ||
              planner_goal_bias < 0.0 || planner_goal_bias > 1.0 ||
              !std::isfinite(planner_rewire_radius) ||
              planner_rewire_radius <= 0.0 ||
              !std::isfinite(planner_goal_tolerance) ||
              planner_goal_tolerance < 0.0) {
            RCLCPP_ERROR(get_logger(), "Planner plugin parameters are invalid");
          } else {
            PlannerSettings settings;
            settings.joint_names = joint_names_;
            settings.joint_limits = position_limits;
            settings.max_joint_step = max_joint_step;
            settings.max_detour_attempts = static_cast<std::size_t>(
                max_detour_attempts);
            settings.max_iterations =
                static_cast<std::size_t>(planner_max_iterations);
            settings.step_size = planner_step_size;
            settings.goal_bias = planner_goal_bias;
            settings.rewire_radius = planner_rewire_radius;
            settings.goal_tolerance = planner_goal_tolerance;
            settings.random_seed =
                static_cast<unsigned int>(std::max(0, planner_random_seed));
            settings.collision_check_resolution = max_collision_joint_step_;
            try {
              planner_loader_ =
                  std::make_unique<pluginlib::ClassLoader<PlannerPlugin>>(
                      "excavator_planning",
                      "tramcar::planning::PlannerPlugin");
              path_planner_ =
                  planner_loader_->createSharedInstance(planner_class_);
              path_planner_->Configure(settings);
              initialized_ = ConfigureWorkspace(
                  workspace_min, workspace_max, workspace_margin);
            } catch (const pluginlib::PluginlibException& error) {
              RCLCPP_ERROR(get_logger(),
                           "Failed to load planner plugin '%s': %s",
                           planner_class_.c_str(), error.what());
              if (planner_loader_) {
                std::string available;
                for (const std::string& name :
                     planner_loader_->getDeclaredClasses()) {
                  available += " " + name;
                }
                RCLCPP_ERROR(get_logger(), "Available planner plugins:%s",
                             available.c_str());
              }
            }
          }
        }
      }
    }

    planned_trajectory_publisher_ =
        create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/excavator/planned_trajectory", rclcpp::QoS(1).transient_local());
    collision_marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/excavator/collision_result",
            rclcpp::QoS(1).transient_local());

    scene_subscription_ =
        create_subscription<excavator_interfaces::msg::PlanningScene>(
            "/excavator/scene", rclcpp::QoS(1).transient_local(),
            [this](
                excavator_interfaces::msg::PlanningScene::ConstSharedPtr msg) {
              HandleScene(std::move(msg));
            });

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
    if (initialized_) {
      RCLCPP_INFO(get_logger(),
                  "Planning server ready (planner_class=%s, %zu joints)",
                  planner_class_.c_str(), joint_names_.size());
    }
  }

 private:
  bool ConfigureWorkspace(const std::vector<double>& minimum,
                          const std::vector<double>& maximum,
                          double margin) {
    if (minimum.empty() && maximum.empty()) {
      return true;
    }
    if (minimum.size() != 3 || maximum.size() != 3) {
      RCLCPP_ERROR(get_logger(),
                   "workspace_min and workspace_max must both have 3 values");
      return false;
    }
    WorkspaceConfig config;
    WorkspaceBox box;
    box.minimum = Vector3{minimum[0], minimum[1], minimum[2]};
    box.maximum = Vector3{maximum[0], maximum[1], maximum[2]};
    config.allowed_regions.push_back(box);
    config.margin = margin;
    workspace_checker_ = WorkspaceChecker(config);
    const ValidationResult validation = workspace_checker_.Validate();
    if (!validation.ok) {
      RCLCPP_ERROR(get_logger(), "Invalid workspace configuration: %s",
                   validation.message.c_str());
      return false;
    }
    workspace_enabled_ = true;
    RCLCPP_INFO(get_logger(),
                "Workspace check enabled for link %s with margin %.3f m",
                workspace_link_.c_str(), margin);
    return true;
  }

  void HandleScene(
      excavator_interfaces::msg::PlanningScene::ConstSharedPtr message) {
    if (message->is_diff) {
      RCLCPP_WARN(get_logger(),
                  "Rejecting incremental planning scene; full snapshots only");
      return;
    }
    std::lock_guard<std::mutex> lock(scene_mutex_);
    scene_ = *message;
    has_scene_ = true;
  }

  excavator_interfaces::msg::PlanningScene ResolveScene(
      const excavator_interfaces::msg::PlanningScene& requested) const {
    if (!requested.objects.empty() || !requested.header.frame_id.empty()) {
      return requested;
    }
    std::lock_guard<std::mutex> lock(scene_mutex_);
    if (has_scene_) {
      return scene_;
    }
    return requested;
  }

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

  void PublishCollisionMarkers(
      const CollisionCheckResult& collision,
      const excavator_interfaces::msg::PlanningScene& scene) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "base_link";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    if (!collision.collision_positions.empty() &&
        collision.collision_positions.size() == joint_names_.size() &&
        !collision.first_link.empty()) {
      Pose3 pose;
      const ValidationResult fk = kinematics_.GetLinkPose(
          collision.collision_positions, collision.first_link, &pose);
      if (fk.ok) {
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "base_link";
        sphere.header.stamp = now();
        sphere.ns = "excavator_collision_link";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = pose.position.x;
        sphere.pose.position.y = pose.position.y;
        sphere.pose.position.z = pose.position.z;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 0.16;
        sphere.scale.y = 0.16;
        sphere.scale.z = 0.16;
        sphere.color = MakeColor(1.0F, 0.1F, 0.1F, 0.95F);
        markers.markers.push_back(sphere);

        visualization_msgs::msg::Marker text;
        text.header = sphere.header;
        text.ns = "excavator_collision_text";
        text.id = 1;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose = sphere.pose;
        text.pose.position.z += 0.3;
        text.scale.z = 0.12;
        text.color = MakeColor(1.0F, 0.4F, 0.4F, 1.0F);
        text.text = collision.message;
        markers.markers.push_back(text);
      }
    }

    const std::string scene_frame =
        scene.header.frame_id.empty() ? "world" : scene.header.frame_id;
    for (const auto& object : scene.objects) {
      if (!object.enabled || object.id != collision.second_object) {
        continue;
      }
      visualization_msgs::msg::Marker highlight;
      highlight.header.frame_id = scene_frame;
      highlight.header.stamp = now();
      highlight.ns = "excavator_collision_obstacle";
      highlight.id = 2;
      highlight.action = visualization_msgs::msg::Marker::ADD;
      highlight.pose = object.pose;
      highlight.color = MakeColor(1.0F, 0.1F, 0.1F, 0.45F);
      if (object.primitive.type ==
          shape_msgs::msg::SolidPrimitive::SPHERE) {
        highlight.type = visualization_msgs::msg::Marker::SPHERE;
        const double diameter =
            object.primitive.dimensions.empty()
                ? 0.2
                : 2.0 * object.primitive.dimensions[0];
        highlight.scale.x = diameter;
        highlight.scale.y = diameter;
        highlight.scale.z = diameter;
      } else if (object.primitive.type ==
                 shape_msgs::msg::SolidPrimitive::CYLINDER) {
        highlight.type = visualization_msgs::msg::Marker::CYLINDER;
        highlight.scale.x =
            object.primitive.dimensions.size() > 0
                ? object.primitive.dimensions[0]
                : 0.2;
        highlight.scale.y = highlight.scale.x;
        highlight.scale.z =
            object.primitive.dimensions.size() > 1
                ? object.primitive.dimensions[1]
                : 0.2;
      } else {
        highlight.type = visualization_msgs::msg::Marker::CUBE;
        highlight.scale.x =
            object.primitive.dimensions.size() > 0
                ? object.primitive.dimensions[0]
                : 0.2;
        highlight.scale.y =
            object.primitive.dimensions.size() > 1
                ? object.primitive.dimensions[1]
                : 0.2;
        highlight.scale.z =
            object.primitive.dimensions.size() > 2
                ? object.primitive.dimensions[2]
                : 0.2;
      }
      markers.markers.push_back(highlight);
      break;
    }
    collision_marker_publisher_->publish(markers);
  }

  void HandleAccepted(const std::shared_ptr<GoalHandlePlan> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanAction::Result>();
    auto feedback = std::make_shared<PlanAction::Feedback>();

    const double allowed_time = ToSeconds(goal->allowed_planning_time);
    if (allowed_time < 0.0 || !std::isfinite(allowed_time)) {
      result->success = false;
      result->message =
          "allowed_planning_time must be finite and non-negative";
      goal_handle->abort(result);
      return;
    }
    const double budget_seconds = allowed_time > 0.0 ? allowed_time : 1.0;
    auto deadline =
        Clock::now() + std::chrono::duration_cast<Clock::duration>(
                           std::chrono::duration<double>(budget_seconds));
    auto out_of_time = [&deadline]() { return Clock::now() >= deadline; };
    auto stop_if_needed = [&]() -> bool {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "planning request canceled";
        goal_handle->canceled(result);
        return true;
      }
      if (out_of_time()) {
        result->success = false;
        result->message = "planning timed out";
        goal_handle->abort(result);
        return true;
      }
      return false;
    };
    if (stop_if_needed()) {
      return;
    }

    JointPositions start;
    JointPositions target;
    std::string error;
    if (!ReadPositions(goal->start_state, joint_names_, &start, &error)) {
      result->success = false;
      result->message = error;
      goal_handle->abort(result);
      return;
    }
    if (goal->use_pose_start) {
      feedback->stage = "start_inverse_kinematics";
      feedback->progress = 0.03F;
      feedback->message = "solving the dragged start pose with IK";
      goal_handle->publish_feedback(feedback);

      if (goal->target_link.empty()) {
        result->success = false;
        result->message = "target_link must be set for a pose start";
        goal_handle->abort(result);
        return;
      }
      Pose3 start_pose;
      if (!NormalizePose(goal->start_pose, &start_pose, &error)) {
        result->success = false;
        result->message = "start_pose: " + error;
        goal_handle->abort(result);
        return;
      }
      InverseKinematicsOptions start_options;
      start_options.max_iterations = 200;
      if (!goal->constrain_orientation) {
        start_options.orientation_weight = 0.0;
        start_options.orientation_tolerance = 1.0e9;
      }
      const InverseKinematicsResult start_ik = kinematics_.SolveIK(
          start_pose, goal->target_link, start, start_options);
      if (!start_ik.success) {
        result->success = false;
        result->message = "start pose IK failed: " + start_ik.message;
        goal_handle->abort(result);
        return;
      }
      start = start_ik.joint_positions;
    }
    if (goal->use_pose_goal) {
      feedback->stage = "inverse_kinematics";
      feedback->progress = 0.05F;
      feedback->message = "solving the Cartesian target with the IK baseline";
      goal_handle->publish_feedback(feedback);

      if (goal->target_link.empty()) {
        result->success = false;
        result->message = "target_link must be set for a pose goal";
        goal_handle->abort(result);
        return;
      }
      const auto& message_pose = goal->target_pose;
      const double quaternion_norm =
          message_pose.orientation.x * message_pose.orientation.x +
          message_pose.orientation.y * message_pose.orientation.y +
          message_pose.orientation.z * message_pose.orientation.z +
          message_pose.orientation.w * message_pose.orientation.w;
      if (!std::isfinite(message_pose.position.x) ||
          !std::isfinite(message_pose.position.y) ||
          !std::isfinite(message_pose.position.z) ||
          !std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-9) {
        result->success = false;
        result->message =
            "target_pose contains invalid position or orientation";
        goal_handle->abort(result);
        return;
      }
      Pose3 target_pose;
      target_pose.position.x = message_pose.position.x;
      target_pose.position.y = message_pose.position.y;
      target_pose.position.z = message_pose.position.z;
      const double inverse_norm = 1.0 / std::sqrt(quaternion_norm);
      target_pose.orientation.x = message_pose.orientation.x * inverse_norm;
      target_pose.orientation.y = message_pose.orientation.y * inverse_norm;
      target_pose.orientation.z = message_pose.orientation.z * inverse_norm;
      target_pose.orientation.w = message_pose.orientation.w * inverse_norm;

      InverseKinematicsOptions ik_options;
      ik_options.max_iterations = 200;
      if (!goal->constrain_orientation) {
        // A 4-DOF arm generally cannot realize an arbitrary 6-DOF pose. The
        // default pose goal therefore solves the position only.
        ik_options.orientation_weight = 0.0;
        ik_options.orientation_tolerance = 1.0e9;
      }
      const InverseKinematicsResult ik_result =
          kinematics_.SolveIK(target_pose, goal->target_link, start,
                              ik_options);
      if (!ik_result.success) {
        result->success = false;
        result->message = "IK failed: " + ik_result.message;
        goal_handle->abort(result);
        return;
      }
      target = ik_result.joint_positions;
    } else if (!ReadPositions(goal->goal_state, joint_names_, &target,
                              &error)) {
      result->success = false;
      result->message = error;
      goal_handle->abort(result);
      return;
    }
    if (stop_if_needed()) {
      return;
    }

    const excavator_interfaces::msg::PlanningScene scene =
        ResolveScene(goal->planning_scene);
    const CollisionCheckResult start_check =
        collision_checker_.CheckState(start, scene);
    if (!start_check.valid || start_check.collision) {
      PublishCollisionMarkers(start_check, scene);
      result->success = false;
      result->message = "start state is invalid: " + start_check.message;
      goal_handle->abort(result);
      return;
    }
    const CollisionCheckResult goal_check =
        collision_checker_.CheckState(target, scene);
    if (!goal_check.valid || goal_check.collision) {
      PublishCollisionMarkers(goal_check, scene);
      result->success = false;
      result->message = "goal state is invalid: " + goal_check.message;
      goal_handle->abort(result);
      return;
    }
    auto state_validity = [this, scene](const JointPositions& positions,
                                        std::string* message) {
      const CollisionCheckResult check =
          collision_checker_.CheckState(positions, scene);
      if (message != nullptr) {
        *message = check.message;
      }
      return check.valid && !check.collision;
    };
    auto segment_validity = [this, scene](
                                const JointPositions& start_positions,
                                const JointPositions& goal_positions,
                                std::string* message) {
      JointPath segment;
      segment.joint_names = joint_names_;
      segment.points = {start_positions, goal_positions};
      const CollisionCheckResult check = collision_checker_.CheckPath(
          segment, scene, max_collision_joint_step_);
      if (message != nullptr) {
        *message = check.message;
      }
      return check.valid && !check.collision;
    };

    feedback->stage = "joint_space_path";
    feedback->progress = 0.1F;
    feedback->message = "generating a baseline joint-space path";
    goal_handle->publish_feedback(feedback);

    PlanningRequest request;
    request.start = std::move(start);
    request.goal = std::move(target);
    request.group_name = "excavator_arm";
    request.timeout_seconds = budget_seconds;
    request.state_validity_check = state_validity;
    request.should_continue = [&]() {
      return !goal_handle->is_canceling() && !out_of_time();
    };
    const Clock::time_point planner_start = Clock::now();
    const PlanningResponse path_response = path_planner_->Plan(request);
    const double planner_seconds =
        std::chrono::duration<double>(Clock::now() - planner_start).count();
    RCLCPP_INFO(get_logger(), "Planner returned success=%s in %.3f s: %s",
                path_response.success ? "true" : "false", planner_seconds,
                path_response.message.c_str());
    if (path_response.success) {
      // Sampling planners may use the entire planning budget to optimize.
      // Reserve a bounded grace period for path/trajectory verification so a
      // valid result found at the deadline is not discarded.
      const double grace_seconds = std::max(0.5, 0.25 * budget_seconds);
      deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                   std::chrono::duration<double>(grace_seconds));
    }
    if (!path_response.success) {
      if (stop_if_needed()) {
        return;
      }
      const JointPath straight =
          MakeStraightPath(request.start, request.goal, joint_names_,
                           max_collision_joint_step_);
      const CollisionCheckResult straight_check = collision_checker_.CheckPath(
          straight, scene, max_collision_joint_step_);
      if (straight_check.collision) {
        PublishCollisionMarkers(straight_check, scene);
      }
      result->success = false;
      result->message = path_response.message;
      goal_handle->abort(result);
      return;
    }

    feedback->stage = "path_post_processing";
    feedback->progress = 0.35F;
    feedback->message = "simplifying and validating the geometric path";
    goal_handle->publish_feedback(feedback);

    JointPath processed_path = path_response.path;
    if (simplify_path_) {
      JointPath simplified;
      const ValidationResult simplify_result = path_processor_.Simplify(
          processed_path, segment_validity, &simplified);
      if (!simplify_result.ok) {
        result->success = false;
        result->message = "path simplification failed: " +
                          simplify_result.message;
        goal_handle->abort(result);
        return;
      }
      processed_path = std::move(simplified);
    }
    if (smooth_path_) {
      JointPath smoothed;
      const ValidationResult smooth_result = path_processor_.Smooth(
          processed_path, smoothing_alpha_, smoothing_iterations_,
          segment_validity, &smoothed);
      if (!smooth_result.ok) {
        result->success = false;
        result->message =
            "path smoothing failed: " + smooth_result.message;
        goal_handle->abort(result);
        return;
      }
      processed_path = std::move(smoothed);
    }
    if (stop_if_needed()) {
      return;
    }

    feedback->stage = "collision_check";
    feedback->progress = 0.45F;
    feedback->message =
        "checking sampled path segments for self and environment collisions";
    goal_handle->publish_feedback(feedback);

    const CollisionCheckResult path_collision = collision_checker_.CheckPath(
        processed_path, scene, max_collision_joint_step_);
    if (!path_collision.valid || path_collision.collision) {
      PublishCollisionMarkers(path_collision, scene);
      result->success = false;
      result->message =
          "path collision validation failed: " + path_collision.message;
      goal_handle->abort(result);
      return;
    }
    if (stop_if_needed()) {
      return;
    }

    feedback->stage = "time_parameterization";
    feedback->progress = 0.6F;
    feedback->message = "assigning conservative segment durations";
    goal_handle->publish_feedback(feedback);

    TimeParameterizationRequest time_request;
    time_request.path = processed_path;
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
    if (stop_if_needed()) {
      return;
    }

    feedback->stage = "trajectory_validation";
    feedback->progress = 0.75F;
    feedback->message =
        "validating sampled trajectory states, limits and workspace";
    goal_handle->publish_feedback(feedback);

    const CollisionCheckResult trajectory_collision =
        collision_checker_.CheckTrajectory(time_response.trajectory, scene,
                                           max_collision_joint_step_);
    if (!trajectory_collision.valid || trajectory_collision.collision) {
      PublishCollisionMarkers(trajectory_collision, scene);
      result->success = false;
      result->message = "trajectory collision validation failed: " +
                        trajectory_collision.message;
      goal_handle->abort(result);
      return;
    }
    if (workspace_enabled_) {
      for (const JointTrajectoryPoint& point :
           time_response.trajectory.points) {
        Pose3 pose;
        const ValidationResult fk = kinematics_.GetLinkPose(
            point.positions, workspace_link_, &pose);
        if (!fk.ok) {
          result->success = false;
          result->message = "workspace forward kinematics failed: " + fk.message;
          goal_handle->abort(result);
          return;
        }
        const WorkspaceCheckResult workspace =
            workspace_checker_.Check(pose);
        if (!workspace.valid || !workspace.inside) {
          result->success = false;
          result->message =
              "workspace validation failed at t=" + std::to_string(point.t) +
              ": " + workspace.message;
          goal_handle->abort(result);
          return;
        }
      }
    }
    if (stop_if_needed()) {
      return;
    }

    result->success = true;
    result->trajectory = ToRosTrajectory(time_response.trajectory);
    planned_trajectory_publisher_->publish(result->trajectory);
    result->message =
        "trajectory generated; sampled FCL collision, limit and workspace "
        "checks passed; sampling does not prove continuous collision freedom";
    feedback->stage = "complete";
    feedback->progress = 1.0F;
    feedback->message = result->message;
    goal_handle->publish_feedback(feedback);
    goal_handle->succeed(result);
  }

  std::vector<std::string> joint_names_;
  UrdfKinematicsModel kinematics_;
  UrdfFclCollisionChecker collision_checker_;
  std::string planner_class_;
  std::unique_ptr<pluginlib::ClassLoader<PlannerPlugin>> planner_loader_;
  std::shared_ptr<PlannerPlugin> path_planner_;
  PathPostProcessor path_processor_;
  WorkspaceChecker workspace_checker_{WorkspaceConfig{}};
  bool workspace_enabled_{false};
  std::string workspace_link_{"bucket_tip"};
  double max_collision_joint_step_{0.03};
  std::vector<double> max_velocity_;
  std::vector<double> max_acceleration_;
  bool simplify_path_{true};
  bool smooth_path_{false};
  int smoothing_iterations_{1};
  double smoothing_alpha_{0.2};
  bool initialized_{false};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      planned_trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      collision_marker_publisher_;
  rclcpp::Subscription<excavator_interfaces::msg::PlanningScene>::SharedPtr
      scene_subscription_;
  rclcpp_action::Server<PlanAction>::SharedPtr action_server_;

  mutable std::mutex scene_mutex_;
  excavator_interfaces::msg::PlanningScene scene_;
  bool has_scene_{false};
};

}  // namespace excavator_planning

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_planning::PlanningServer>());
  rclcpp::shutdown();
  return 0;
}
