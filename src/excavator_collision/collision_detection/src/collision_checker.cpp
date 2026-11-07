#include "excavator_collision/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "excavator_collision/internal/robot_collision_model.hpp"
#include "fcl_collision_model.hpp"
#include "kinematics/urdf_kinematics_model.hpp"

namespace tramcar {
namespace planning {
namespace {

Eigen::Isometry3d ToTransform(const Pose3& pose, std::string* error) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d translation(pose.position.x, pose.position.y,
                                    pose.position.z);
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
  const double norm = quaternion.norm();
  if (!translation.allFinite() || !quaternion.coeffs().allFinite() ||
      !std::isfinite(norm) || norm < 1.0e-12) {
    *error = "kinematics returned an invalid link pose";
    return transform;
  }

  quaternion.normalize();
  transform.translation() = translation;
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

}  // namespace

struct UrdfFclCollisionChecker::Impl {
  RobotCollisionModel robot_model;
  FclCollisionModel fcl_model;
  std::unique_ptr<UrdfKinematicsModel> owned_kinematics;
  const KinematicsModel* kinematics_model{nullptr};
  std::vector<std::string> joint_names;
  std::vector<std::pair<std::string, std::string>> allowed_pairs;
  bool initialized{false};

  ValidationResult ComputeLinkPoses(
      const JointPositions& positions,
      std::unordered_map<std::string, Eigen::Isometry3d>* link_poses) const {
    if (!initialized || kinematics_model == nullptr) {
      return {false, "collision checker has not been initialized"};
    }
    if (link_poses == nullptr || positions.size() != joint_names.size()) {
      return {false, "joint position count does not match configured group"};
    }
    for (double position : positions) {
      if (!std::isfinite(position)) {
        return {false, "joint positions must be finite"};
      }
    }

    link_poses->clear();
    for (const LinkCollisionGeometry& link : robot_model.links()) {
      Pose3 pose;
      const ValidationResult fk = kinematics_model->GetLinkPose(
          positions, link.link_name, &pose);
      if (!fk.ok) {
        return {false, "forward kinematics failed for link " + link.link_name +
                           ": " + fk.message};
      }

      std::string error;
      const Eigen::Isometry3d reference_to_link =
          ToTransform(pose, &error);
      const Eigen::Isometry3d transform =
          robot_model.root_to_reference_frame() * reference_to_link;
      if (!error.empty()) {
        return {false, error + ": " + link.link_name};
      }
      link_poses->emplace(link.link_name, transform);
    }
    return {true, ""};
  }

  CollisionCheckResult Check(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene,
      bool check_self_collision, bool check_environment_collision,
      const std::string& obstacle_id_filter = {}) const {
    CollisionCheckResult result;
    std::unordered_map<std::string, Eigen::Isometry3d> link_poses;
    const ValidationResult fk = ComputeLinkPoses(positions, &link_poses);
    if (!fk.ok) {
      result.message = fk.message;
      return result;
    }

    return fcl_model.Check(link_poses, scene, allowed_pairs, positions,
                           check_self_collision, check_environment_collision,
                           obstacle_id_filter);
  }

  CollisionCheckResult CheckPath(
      const JointPath& path,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step, bool check_self_collision,
      bool check_environment_collision,
      const std::string& obstacle_id_filter = {}) const {
    CollisionCheckResult result;
    if (!initialized) {
      result.message = "collision checker has not been initialized";
      return result;
    }
    if (path.points.empty() || path.joint_names != joint_names ||
        !std::isfinite(max_joint_step) || max_joint_step <= 0.0) {
      result.message = "invalid path, joint names, or max_joint_step";
      return result;
    }

    std::size_t checked_states = 0;
    for (std::size_t segment = 0; segment + 1 < path.points.size();
         ++segment) {
      const JointPositions& start = path.points[segment];
      const JointPositions& end = path.points[segment + 1];
      if (start.size() != joint_names.size() ||
          end.size() != joint_names.size()) {
        result.message = "path waypoint joint count mismatch";
        return result;
      }

      double max_delta = 0.0;
      for (std::size_t joint = 0; joint < start.size(); ++joint) {
        if (!std::isfinite(start[joint]) || !std::isfinite(end[joint])) {
          result.message = "path contains non-finite joint position";
          return result;
        }
        max_delta = std::max(max_delta, std::abs(end[joint] - start[joint]));
      }

      const std::size_t subdivisions = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(max_delta / max_joint_step)));
      for (std::size_t sample = 0; sample < subdivisions; ++sample) {
        const double alpha = static_cast<double>(sample) /
                             static_cast<double>(subdivisions);
        JointPositions positions(start.size());
        for (std::size_t joint = 0; joint < start.size(); ++joint) {
          positions[joint] =
              start[joint] + alpha * (end[joint] - start[joint]);
        }

        result = Check(positions, scene, check_self_collision,
                       check_environment_collision, obstacle_id_filter);
        ++checked_states;
        result.checked_states = static_cast<double>(checked_states);
        if (!result.valid || result.collision) {
          return result;
        }
      }
    }

    const JointPositions& final_positions = path.points.back();
    if (final_positions.size() != joint_names.size()) {
      result.valid = false;
      result.message = "path waypoint joint count mismatch";
      return result;
    }
    result = Check(final_positions, scene, check_self_collision,
                   check_environment_collision, obstacle_id_filter);
    ++checked_states;
    result.checked_states = static_cast<double>(checked_states);
    if (result.valid && !result.collision) {
      result.message = "no collision detected at sampled path states";
    }
    return result;
  }
};

UrdfFclCollisionChecker::UrdfFclCollisionChecker()
    : impl_(std::make_unique<Impl>()) {}

UrdfFclCollisionChecker::~UrdfFclCollisionChecker() = default;

ValidationResult UrdfFclCollisionChecker::Initialize(
    const CollisionModelConfig& config) {
  return Initialize(config, nullptr);
}

ValidationResult UrdfFclCollisionChecker::Initialize(
    const CollisionModelConfig& config,
    const KinematicsModel* kinematics_model) {
  impl_->initialized = false;
  impl_->owned_kinematics.reset();
  impl_->kinematics_model = nullptr;
  impl_->joint_names.clear();
  impl_->allowed_pairs.clear();

  if (!config.model_format.empty() && config.model_format != "urdf") {
    return {false, "urdf_fcl backend expects a URDF collision model"};
  }
  if (config.model_source.empty() || config.joint_names.empty()) {
    return {false, "URDF model source and planning joints must not be empty"};
  }

  if (kinematics_model == nullptr) {
    auto urdf_kinematics =
        std::make_unique<UrdfKinematicsModel>(config.joint_names);
    const ValidationResult kinematics_result =
        urdf_kinematics->InitializeFromString(config.model_source);
    if (!kinematics_result.ok) {
      return {false, "failed to initialize fallback URDF kinematics: " +
                         kinematics_result.message};
    }
    impl_->owned_kinematics = std::move(urdf_kinematics);
    impl_->kinematics_model = impl_->owned_kinematics.get();
  } else {
    if (kinematics_model->joint_names() != config.joint_names) {
      return {false, "kinematics joint ordering does not match collision "
                     "configuration"};
    }
    impl_->kinematics_model = kinematics_model;
  }

  const ValidationResult model_result =
      impl_->robot_model.InitializeFromUrdf(
          config.model_source, config.joint_names,
          config.kinematics_reference_frame);
  if (!model_result.ok) {
    impl_->kinematics_model = nullptr;
    impl_->owned_kinematics.reset();
    return model_result;
  }

  const ValidationResult fcl_result =
      impl_->fcl_model.Initialize(impl_->robot_model);
  if (!fcl_result.ok) {
    impl_->kinematics_model = nullptr;
    impl_->owned_kinematics.reset();
    return fcl_result;
  }

  impl_->joint_names = config.joint_names;
  impl_->initialized = true;
  return {true, ""};
}

ValidationResult UrdfFclCollisionChecker::Initialize(
    const std::string& urdf_xml, std::vector<std::string> joint_names) {
  CollisionModelConfig config;
  config.joint_names = std::move(joint_names);
  config.model_source = urdf_xml;
  config.model_format = "urdf";
  return Initialize(config, nullptr);
}

void UrdfFclCollisionChecker::SetAllowedCollisionPairs(
    const std::vector<std::pair<std::string, std::string>>& pairs) {
  impl_->allowed_pairs.clear();
  for (const auto& pair : pairs) {
    if (!pair.first.empty() && !pair.second.empty() &&
        pair.first != pair.second) {
      impl_->allowed_pairs.push_back(pair);
    }
  }
}

CollisionCheckResult UrdfFclCollisionChecker::CheckState(
    const JointPositions& positions,
    const excavator_interfaces::msg::PlanningScene& scene) const {
  return impl_->Check(positions, scene, true, true);
}

CollisionCheckResult UrdfFclCollisionChecker::CheckSelfCollision(
    const JointPositions& positions) const {
  const excavator_interfaces::msg::PlanningScene empty_scene;
  return impl_->Check(positions, empty_scene, true, false);
}

CollisionCheckResult UrdfFclCollisionChecker::CheckEnvironmentCollision(
    const JointPositions& positions,
    const excavator_interfaces::msg::PlanningScene& scene) const {
  return impl_->Check(positions, scene, false, true);
}

CollisionCheckResult UrdfFclCollisionChecker::CheckPath(
    const JointPath& path,
    const excavator_interfaces::msg::PlanningScene& scene,
    double max_joint_step) const {
  return impl_->CheckPath(path, scene, max_joint_step, true, true);
}

CollisionCheckResult UrdfFclCollisionChecker::CheckPathCollisionWithObstacle(
    const JointPath& path,
    const excavator_interfaces::msg::PlanningScene& scene,
    const std::string& obstacle_id, double max_joint_step) const {
  if (obstacle_id.empty()) {
    CollisionCheckResult result;
    result.message = "obstacle_id must not be empty";
    return result;
  }
  return impl_->CheckPath(path, scene, max_joint_step, false, true,
                          obstacle_id);
}

CollisionCheckResult UrdfFclCollisionChecker::CheckTrajectory(
    const JointTrajectory& trajectory,
    const excavator_interfaces::msg::PlanningScene& scene,
    double max_joint_step) const {
  CollisionCheckResult result;
  const ValidationResult validation = ValidateJointTrajectory(trajectory);
  if (!validation.ok) {
    result.message = validation.message;
    return result;
  }
  if (trajectory.points.empty()) {
    result.message = "trajectory must contain at least one point";
    return result;
  }

  JointPath path;
  path.joint_names = trajectory.joint_names;
  path.points.reserve(trajectory.points.size());
  for (const JointTrajectoryPoint& point : trajectory.points) {
    path.points.push_back(point.positions);
  }
  return CheckPath(path, scene, max_joint_step);
}

CollisionDistanceResult UrdfFclCollisionChecker::ComputeDistanceToObstacle(
    const JointPositions& positions,
    const excavator_interfaces::msg::PlanningScene& scene,
    const std::string& obstacle_id) const {
  CollisionDistanceResult result;
  std::unordered_map<std::string, Eigen::Isometry3d> link_poses;
  const ValidationResult fk = impl_->ComputeLinkPoses(positions, &link_poses);
  if (!fk.ok) {
    result.message = fk.message;
    return result;
  }
  return impl_->fcl_model.ComputeDistanceToObstacle(
      link_poses, scene, positions, obstacle_id);
}

}  // namespace planning
}  // namespace tramcar
