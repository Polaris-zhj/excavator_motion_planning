#include "excavator_collision/fcl_collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <octomap/octomap.h>

#include "fcl_collision_model_builder.hpp"

namespace excavator {
namespace planning {

// FCL objects built once for a query or for a whole path. Point clouds are
// voxelized here rather than inside the robot-model adapter.
struct FclEnvironment {
  struct Object {
    int id;
    std::shared_ptr<fcl::CollisionObjectd> object;
  };
  std::vector<Object> objects;
};

namespace {

using Transform = Eigen::Isometry3d;

bool IsFinitePoint(const data::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

Transform ToTransform(const data::Point& center) {
  Transform transform = Transform::Identity();
  transform.translation() = Eigen::Vector3d(center.x, center.y, center.z);
  return transform;
}

std::string PairKey(std::string first, std::string second) {
  if (second < first) {
    std::swap(first, second);
  }
  return first + "|" + second;
}

std::shared_ptr<fcl::CollisionGeometryd> CreateBoxGeometry(
    const data::Object& object, std::string* error) {
  const auto valid_dimension = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!valid_dimension(object.length) || !valid_dimension(object.width) ||
      !valid_dimension(object.height)) {
    *error = "obstacle box requires three positive dimensions: " +
             std::to_string(object.id);
    return {};
  }
  return std::make_shared<fcl::Boxd>(object.length, object.width,
                                     object.height);
}

std::shared_ptr<fcl::CollisionGeometryd> CreatePointCloudGeometry(
    const std::vector<data::Point>& points, double resolution, int object_id,
    std::string* error) {
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    *error = "point cloud resolution must be positive: " +
             std::to_string(object_id);
    return {};
  }
  if (points.empty()) {
    *error = "point cloud is empty: " + std::to_string(object_id);
    return {};
  }

  auto tree = std::make_shared<octomap::OcTree>(resolution);
  std::size_t valid_points = 0;
  for (const data::Point& point : points) {
    if (!IsFinitePoint(point)) {
      continue;
    }
    // Mark occupied voxels only. Free-space carving requires a sensor origin
    // and reliable unknown/free semantics, which this interface does not carry
    // yet.
    tree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
    ++valid_points;
  }

  if (valid_points == 0) {
    *error = "point cloud contains no valid points: " +
             std::to_string(object_id);
    return {};
  }
  tree->updateInnerOccupancy();
  return std::make_shared<fcl::OcTreed>(tree);
}

CollisionCheckResult CheckCollisionObjects(
    const std::vector<LinkCollisionInfo>& links,
    const FclEnvironment& environment,
    const std::vector<std::pair<std::string, std::string>>& allowed_pairs,
    const JointPositions& positions, bool check_self_collision,
    bool check_environment_collision, bool collect_all) {
  CollisionCheckResult result;
  if (links.empty()) {
    result.message = "no robot collision geometry is available";
    return result;
  }
  if (!check_self_collision && !check_environment_collision) {
    result.message = "at least one collision query scope must be enabled";
    return result;
  }

  const auto add_colliding_link = [&result](const std::string& link_name) {
    if (std::find(result.colliding_links.begin(), result.colliding_links.end(),
                  link_name) == result.colliding_links.end()) {
      result.colliding_links.push_back(link_name);
    }
  };
  const auto finish_collision = [&result, &positions]() {
    result.valid = true;
    result.collision = true;
    result.collision_positions = positions;
    result.checked_states = 1.0;
    if (result.colliding_links.empty() && !result.first_link.empty()) {
      result.colliding_links.push_back(result.first_link);
    }
    return result;
  };

  bool collision_found = false;
  const auto note_collision = [&](const std::string& first_link,
                                  const std::string& second_object,
                                  const std::string& message) {
    if (!collision_found) {
      result.first_link = first_link;
      result.second_object = second_object;
      result.message = message;
      collision_found = true;
    }
    add_colliding_link(first_link);
  };

  if (check_environment_collision) {
    for (const LinkCollisionInfo& link : links) {
      for (const FclEnvironment::Object& obstacle : environment.objects) {
        fcl::CollisionRequestd request;
        request.num_max_contacts = 1;
        fcl::CollisionResultd collision_result;
        fcl::collide(link.object.get(), obstacle.object.get(), request,
                     collision_result);
        if (collision_result.isCollision()) {
          note_collision(link.link_name, std::to_string(obstacle.id),
                         "environment collision: " + link.link_name + " with " +
                             std::to_string(obstacle.id));
          if (!collect_all) {
            return finish_collision();
          }
        }
      }
    }
  }

  if (check_self_collision) {
    std::unordered_set<std::string> allowed_pair_keys;
    for (const auto& pair : allowed_pairs) {
      if (!pair.first.empty() && !pair.second.empty() &&
          pair.first != pair.second) {
        allowed_pair_keys.insert(PairKey(pair.first, pair.second));
      }
    }

    for (std::size_t first_index = 0; first_index < links.size();
         ++first_index) {
      for (std::size_t second_index = first_index + 1;
           second_index < links.size(); ++second_index) {
        const LinkCollisionInfo& first = links[first_index];
        const LinkCollisionInfo& second = links[second_index];
        if (first.link_name == second.link_name ||
            first.parent_link_name == second.link_name ||
            second.parent_link_name == first.link_name ||
            allowed_pair_keys.count(
                PairKey(first.link_name, second.link_name)) > 0) {
          continue;
        }

        fcl::CollisionRequestd request;
        request.num_max_contacts = 1;
        fcl::CollisionResultd collision_result;
        fcl::collide(first.object.get(), second.object.get(), request,
                     collision_result);
        if (collision_result.isCollision()) {
          note_collision(first.link_name, second.link_name,
                         "self collision: " + first.link_name + " with " +
                             second.link_name);
          add_colliding_link(second.link_name);
          if (!collect_all) {
            return finish_collision();
          }
        }
      }
    }
  }

  if (collision_found) {
    return finish_collision();
  }

  result.valid = true;
  result.collision = false;
  result.collision_positions = positions;
  result.checked_states = 1.0;
  result.message = "no collision detected";
  return result;
}

}  // namespace

FclCollisionChecker::FclCollisionChecker()
    : fcl_model_(std::make_unique<FclCollisionModelBuilder>()) {}

FclCollisionChecker::~FclCollisionChecker() = default;

ValidationResult FclCollisionChecker::Initialize(
    const CollisionModelConfig& config,
    const KinematicsModel* kinematics_model) {
  initialized_ = false;
  allowed_pairs_.clear();

  const ValidationResult result = fcl_model_->Initialize(config, kinematics_model);
  if (!result.ok) {
    return result;
  }
  initialized_ = true;
  return {true, ""};
}

void FclCollisionChecker::SetAllowedCollision(const std::string& link_a,
                                              const std::string& link_b,
                                              bool allowed) {
  if (link_a.empty() || link_b.empty() || link_a == link_b) {
    return;
  }
  const auto matches = [&](const std::pair<std::string, std::string>& pair) {
    return (pair.first == link_a && pair.second == link_b) ||
           (pair.first == link_b && pair.second == link_a);
  };

  if (allowed) {
    if (std::none_of(allowed_pairs_.begin(), allowed_pairs_.end(), matches)) {
      allowed_pairs_.emplace_back(link_a, link_b);
    }
    return;
  }

  allowed_pairs_.erase(
      std::remove_if(allowed_pairs_.begin(), allowed_pairs_.end(), matches),
      allowed_pairs_.end());
}

ValidationResult FclCollisionChecker::BuildEnvironment(
    const std::vector<data::Object>& obstacles,
    const CollisionQueryOptions& options,
    FclEnvironment* environment) const {
  if (environment == nullptr) {
    return {false, "environment collision object output is null"};
  }
  environment->objects.clear();
  if (!initialized_) {
    return {false, "collision checker has not been initialized"};
  }

  if (options.obstacle_type == ObstacleType::kPointCloud) {
    if (!std::isfinite(options.point_cloud_resolution) ||
        options.point_cloud_resolution <= 0.0) {
      return {false, "point cloud resolution must be positive"};
    }
  } else if (options.obstacle_type != ObstacleType::kGeometry) {
    return {false, "unsupported obstacle representation type"};
  }

  std::unordered_set<int> ids;
  for (const data::Object& source : obstacles) {
    if (source.id < 0 || !ids.insert(source.id).second) {
      return {false, "obstacle IDs must be non-negative and unique"};
    }
    if (!IsFinitePoint(source.center)) {
      return {false, "invalid obstacle center: " + std::to_string(source.id)};
    }

    std::string error;
    std::shared_ptr<fcl::CollisionGeometryd> geometry;
    if (options.obstacle_type == ObstacleType::kPointCloud) {
      geometry = CreatePointCloudGeometry(
          source.cloud_points, options.point_cloud_resolution, source.id,
          &error);
    } else {
      geometry = CreateBoxGeometry(source, &error);
    }
    if (!geometry) {
      return {false, error};
    }

    auto object = std::make_shared<fcl::CollisionObjectd>(
        std::move(geometry), ToTransform(source.center));
    object->computeAABB();
    environment->objects.push_back({source.id, std::move(object)});
  }
  return {true, ""};
}

ValidationResult FclCollisionChecker::ValidateSingleObstacle(
    const data::Object& obstacle) const {
  if (obstacle.id < 0) {
    return {false, "obstacle id must be non-negative"};
  }
  return {true, ""};
}

CollisionCheckResult FclCollisionChecker::CheckPositions(
    const JointPositions& positions, const std::vector<data::Object>& obstacles,
    const CollisionQueryOptions& options, bool check_self_collision,
    bool check_environment_collision) const {
  CollisionCheckResult result;
  FclEnvironment environment;
  if (check_environment_collision) {
    const ValidationResult environment_result =
        BuildEnvironment(obstacles, options, &environment);
    if (!environment_result.ok) {
      result.message = environment_result.message;
      return result;
    }
  }

  std::vector<LinkCollisionInfo> links;
  const ValidationResult link_result =
      fcl_model_->ComputeLinkCollisionObjects(positions, &links);
  if (!link_result.ok) {
    result.message = link_result.message;
    return result;
  }

  return CheckCollisionObjects(links, environment, allowed_pairs_, positions,
                               check_self_collision,
                               check_environment_collision,
                               options.collect_all_collisions);
}

CollisionCheckResult FclCollisionChecker::CheckPathCore(
    const JointPath& path, const std::vector<data::Object>& obstacles,
    double max_joint_step, const CollisionQueryOptions& options,
    bool check_self_collision, bool check_environment_collision) const {
  CollisionCheckResult result;
  const std::vector<std::string>& joint_names = fcl_model_->joint_names();
  if (!initialized_) {
    result.message = "collision checker has not been initialized";
    return result;
  }
  if (path.points.empty() || path.joint_names != joint_names ||
      !std::isfinite(max_joint_step) || max_joint_step <= 0.0) {
    result.message = "invalid path, joint names, or max_joint_step";
    return result;
  }

  // Build environment objects once so point clouds are voxelized only once
  // for the whole path.
  FclEnvironment environment;
  if (check_environment_collision) {
    const ValidationResult environment_result =
        BuildEnvironment(obstacles, options, &environment);
    if (!environment_result.ok) {
      result.message = environment_result.message;
      return result;
    }
  }

  std::size_t checked_states = 0;
  for (std::size_t segment = 0; segment + 1 < path.points.size(); ++segment) {
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
      const double alpha =
          static_cast<double>(sample) / static_cast<double>(subdivisions);
      JointPositions positions(start.size());
      for (std::size_t joint = 0; joint < start.size(); ++joint) {
        positions[joint] =
            start[joint] + alpha * (end[joint] - start[joint]);
      }

      std::vector<LinkCollisionInfo> links;
      const ValidationResult link_result =
          fcl_model_->ComputeLinkCollisionObjects(positions, &links);
      if (!link_result.ok) {
        result.message = link_result.message;
        return result;
      }
      result = CheckCollisionObjects(
          links, environment, allowed_pairs_, positions, check_self_collision,
          check_environment_collision, options.collect_all_collisions);
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
  std::vector<LinkCollisionInfo> final_links;
  const ValidationResult final_result =
      fcl_model_->ComputeLinkCollisionObjects(final_positions, &final_links);
  if (!final_result.ok) {
    result.message = final_result.message;
    return result;
  }
  result = CheckCollisionObjects(
      final_links, environment, allowed_pairs_, final_positions,
      check_self_collision, check_environment_collision,
      options.collect_all_collisions);
  ++checked_states;
  result.checked_states = static_cast<double>(checked_states);
  if (result.valid && !result.collision) {
    result.message = "no collision detected at sampled path states";
  }
  return result;
}

CollisionCheckResult FclCollisionChecker::CheckCollision(
    const JointPositions& positions, const std::vector<data::Object>& obstacles,
    const CollisionQueryOptions& options) const {
  return CheckPositions(positions, obstacles, options, true, true);
}

CollisionCheckResult FclCollisionChecker::CheckSelfCollision(
    const JointPositions& positions) const {
  return CheckPositions(positions, {}, CollisionQueryOptions{}, true, false);
}

CollisionCheckResult FclCollisionChecker::CheckEnvironmentCollision(
    const JointPositions& positions, const std::vector<data::Object>& obstacles,
    const CollisionQueryOptions& options) const {
  return CheckPositions(positions, obstacles, options, false, true);
}

CollisionCheckResult FclCollisionChecker::CheckPathCollision(
    const JointPath& path, const std::vector<data::Object>& obstacles,
    double max_joint_step, const CollisionQueryOptions& options) const {
  return CheckPathCore(path, obstacles, max_joint_step, options, true, true);
}

CollisionCheckResult FclCollisionChecker::CheckPathCollision(
    const JointPath& path, const data::Object& obstacle, double max_joint_step,
    const CollisionQueryOptions& options) const {
  CollisionCheckResult result;
  const ValidationResult obstacle_result = ValidateSingleObstacle(obstacle);
  if (!obstacle_result.ok) {
    result.message = obstacle_result.message;
    return result;
  }
  return CheckPathCore(path, {obstacle}, max_joint_step, options, false, true);
}

CollisionCheckResult FclCollisionChecker::CheckTrajectory(
    const JointTrajectory& trajectory, const std::vector<data::Object>& obstacles,
    double max_joint_step, const CollisionQueryOptions& options) const {
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
  return CheckPathCollision(path, obstacles, max_joint_step, options);
}

CollisionDistanceResult FclCollisionChecker::ComputeDistance(
    const JointPositions& positions, const data::Object& obstacle,
    const CollisionQueryOptions& options) const {
  CollisionDistanceResult result;
  const ValidationResult obstacle_result = ValidateSingleObstacle(obstacle);
  if (!obstacle_result.ok) {
    result.message = obstacle_result.message;
    return result;
  }

  FclEnvironment environment;
  const ValidationResult environment_result =
      BuildEnvironment({obstacle}, options, &environment);
  if (!environment_result.ok) {
    result.message = environment_result.message;
    return result;
  }
  if (environment.objects.empty()) {
    result.message = "obstacle is not present: " + std::to_string(obstacle.id);
    return result;
  }
  const FclEnvironment::Object* target = nullptr;
  for (const FclEnvironment::Object& candidate : environment.objects) {
    if (candidate.id == obstacle.id) {
      target = &candidate;
      break;
    }
  }
  if (target == nullptr) {
    result.message = "obstacle not found: " + std::to_string(obstacle.id);
    return result;
  }

  std::vector<LinkCollisionInfo> links;
  const ValidationResult link_result =
      fcl_model_->ComputeLinkCollisionObjects(positions, &links);
  if (!link_result.ok) {
    result.message = link_result.message;
    return result;
  }

  fcl::DistanceRequestd request;
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (const LinkCollisionInfo& link : links) {
    fcl::DistanceResultd distance_result;
    const double distance = fcl::distance(
        link.object.get(), target->object.get(), request, distance_result);
    if (!std::isfinite(distance)) {
      result.message = "FCL returned a non-finite distance";
      return result;
    }
    if (distance < minimum_distance) {
      minimum_distance = distance;
      result.robot_link = link.link_name;
    }
  }

  result.valid = true;
  result.distance = minimum_distance;
  result.obstacle_id = std::to_string(obstacle.id);
  result.joint_positions = positions;
  result.message = "minimum distance computed";
  return result;
}

}  // namespace planning
}  // namespace excavator
