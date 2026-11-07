#include "fcl_collision_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace tramcar {
namespace planning {
namespace {

using Transform = Eigen::Isometry3d;
using Quaternion = Eigen::Quaterniond;

bool IsFinite(const Transform& transform) {
  return transform.matrix().allFinite();
}

bool IsFinitePose(const geometry_msgs::msg::Pose& pose) {
  const Quaternion quaternion(pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z);
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && quaternion.coeffs().allFinite() &&
         std::isfinite(quaternion.norm()) && quaternion.norm() > 1.0e-12;
}

Transform ToTransform(const geometry_msgs::msg::Pose& pose) {
  Transform transform = Transform::Identity();
  transform.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Quaternion quaternion(pose.orientation.w, pose.orientation.x,
                        pose.orientation.y, pose.orientation.z);
  quaternion.normalize();
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

std::string PairKey(std::string first, std::string second) {
  if (second < first) {
    std::swap(first, second);
  }
  return first + "|" + second;
}

std::shared_ptr<fcl::CollisionGeometryd> CreateFclGeometry(
    const CollisionGeometry& geometry, std::string* error) {
  switch (geometry.type) {
    case CollisionGeometryType::kBox:
      return std::make_shared<fcl::Boxd>(
          geometry.dimensions.x(), geometry.dimensions.y(),
          geometry.dimensions.z());
    case CollisionGeometryType::kSphere:
      return std::make_shared<fcl::Sphered>(geometry.dimensions.x());
    case CollisionGeometryType::kCylinder:
      return std::make_shared<fcl::Cylinderd>(geometry.dimensions.x(),
                                               geometry.dimensions.y());
    case CollisionGeometryType::kTriangleMesh: {
      if (geometry.mesh_vertices.empty() ||
          geometry.mesh_triangles.empty()) {
        *error = "triangle mesh contains no vertices or triangles";
        return {};
      }
      if (geometry.mesh_vertices.size() >
          static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        *error = "triangle mesh has too many vertices for FCL";
        return {};
      }

      std::vector<fcl::Vector3d> vertices;
      vertices.reserve(geometry.mesh_vertices.size());
      for (const Eigen::Vector3d& vertex : geometry.mesh_vertices) {
        if (!vertex.allFinite()) {
          *error = "triangle mesh contains non-finite vertex coordinates";
          return {};
        }
        vertices.emplace_back(vertex.x(), vertex.y(), vertex.z());
      }

      std::vector<fcl::Triangle> triangles;
      triangles.reserve(geometry.mesh_triangles.size());
      for (const auto& triangle : geometry.mesh_triangles) {
        if (triangle[0] >= vertices.size() || triangle[1] >= vertices.size() ||
            triangle[2] >= vertices.size()) {
          *error = "triangle mesh contains an out-of-range vertex index";
          return {};
        }
        triangles.emplace_back(static_cast<int>(triangle[0]),
                               static_cast<int>(triangle[1]),
                               static_cast<int>(triangle[2]));
      }

      auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
      if (model->beginModel() != fcl::BVH_OK ||
          model->addSubModel(vertices, triangles) != fcl::BVH_OK ||
          model->endModel() != fcl::BVH_OK) {
        *error = "FCL failed to build the triangle-mesh BVH";
        return {};
      }
      return model;
    }
  }

  *error = "unsupported project collision geometry type";
  return {};
}

std::shared_ptr<fcl::CollisionGeometryd> CreateEnvironmentGeometry(
    const shape_msgs::msg::SolidPrimitive& primitive,
    const std::string& object_id, std::string* error) {
  const auto valid_dimension = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };

  switch (primitive.type) {
    case shape_msgs::msg::SolidPrimitive::BOX:
      if (primitive.dimensions.size() != 3 ||
          !valid_dimension(primitive.dimensions[0]) ||
          !valid_dimension(primitive.dimensions[1]) ||
          !valid_dimension(primitive.dimensions[2])) {
        *error = "environment box requires three positive dimensions: " +
                 object_id;
        return {};
      }
      return std::make_shared<fcl::Boxd>(
          primitive.dimensions[0], primitive.dimensions[1],
          primitive.dimensions[2]);
    case shape_msgs::msg::SolidPrimitive::SPHERE:
      if (primitive.dimensions.size() != 1 ||
          !valid_dimension(primitive.dimensions[0])) {
        *error = "environment sphere requires one positive radius: " +
                 object_id;
        return {};
      }
      return std::make_shared<fcl::Sphered>(primitive.dimensions[0]);
    case shape_msgs::msg::SolidPrimitive::CYLINDER:
      if (primitive.dimensions.size() != 2 ||
          !valid_dimension(
              primitive.dimensions[
                  shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS]) ||
          !valid_dimension(
              primitive.dimensions[
                  shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT])) {
        *error =
            "environment cylinder requires positive radius and height: " +
            object_id;
        return {};
      }
      return std::make_shared<fcl::Cylinderd>(
          primitive.dimensions[
              shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
          primitive.dimensions[
              shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT]);
    default:
      *error = "unsupported environment primitive type: " + object_id;
      return {};
  }
}

}  // namespace

ValidationResult FclCollisionModel::Initialize(
    const RobotCollisionModel& robot_model) {
  root_frame_ = robot_model.root_frame();
  robot_geometries_.clear();

  if (root_frame_.empty() || robot_model.links().empty()) {
    return {false, "robot collision model is empty or has no root frame"};
  }

  for (const LinkCollisionGeometry& link : robot_model.links()) {
    for (const CollisionGeometryInstance& instance : link.geometries) {
      std::string error;
      GeometryPtr geometry = CreateFclGeometry(instance.geometry, &error);
      if (!geometry) {
        return {false, "failed to create FCL geometry for link " +
                           link.link_name + ": " + error};
      }

      RobotGeometry robot_geometry;
      robot_geometry.link_name = link.link_name;
      robot_geometry.parent_link_name = link.parent_link_name;
      robot_geometry.link_to_collision = instance.link_to_collision;
      robot_geometry.geometry = std::move(geometry);
      robot_geometries_.push_back(std::move(robot_geometry));
    }
  }

  if (robot_geometries_.empty()) {
    return {false, "robot collision model contains no FCL geometry"};
  }
  return {true, ""};
}

bool FclCollisionModel::BuildRobotObjects(
    const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses,
    std::vector<WorldGeometry>* objects, std::string* error) const {
  if (objects == nullptr) {
    *error = "robot collision object output is null";
    return false;
  }
  objects->clear();
  objects->reserve(robot_geometries_.size());

  for (const RobotGeometry& geometry : robot_geometries_) {
    const auto pose = link_poses.find(geometry.link_name);
    if (pose == link_poses.end()) {
      *error = "FK result is missing collision link: " + geometry.link_name;
      return false;
    }
    const Transform world_collision =
        pose->second * geometry.link_to_collision;
    if (!IsFinite(world_collision)) {
      *error = "non-finite world collision transform for link: " +
               geometry.link_name;
      return false;
    }

    // T_world_collision = T_world_link * T_link_collision.
    // Build per-query collision objects to avoid shared mutable FCL state.
    auto object = std::make_shared<fcl::CollisionObjectd>(
        geometry.geometry, world_collision);
    object->computeAABB();
    objects->push_back(
        {geometry.link_name, geometry.parent_link_name, std::move(object)});
  }
  return true;
}

bool FclCollisionModel::BuildEnvironmentObjects(
    const excavator_interfaces::msg::PlanningScene& scene,
    const std::string& obstacle_id_filter,
    std::vector<EnvironmentGeometry>* objects, std::string* error) const {
  if (objects == nullptr) {
    *error = "environment collision object output is null";
    return false;
  }
  objects->clear();

  if (!scene.header.frame_id.empty() && scene.header.frame_id != root_frame_) {
    *error = "planning scene frame_id must be empty or match the robot root "
             "frame '" +
             root_frame_ + "'";
    return false;
  }

  std::unordered_set<std::string> ids;
  bool target_found = false;
  for (const auto& source : scene.objects) {
    if (source.id.empty() || !ids.insert(source.id).second) {
      *error = "planning scene object IDs must be non-empty and unique";
      return false;
    }
    if (!obstacle_id_filter.empty() && source.id != obstacle_id_filter) {
      continue;
    }
    if (!source.enabled) {
      continue;
    }

    target_found = true;
    if (!IsFinitePose(source.pose)) {
      *error = "invalid environment object pose: " + source.id;
      return false;
    }

    GeometryPtr geometry =
        CreateEnvironmentGeometry(source.primitive, source.id, error);
    if (!geometry) {
      return false;
    }

    auto object = std::make_shared<fcl::CollisionObjectd>(
        std::move(geometry), ToTransform(source.pose));
    object->computeAABB();
    objects->push_back({source.id, std::move(object)});
  }

  if (!obstacle_id_filter.empty() && !target_found) {
    *error = "enabled obstacle not found in planning scene: " +
             obstacle_id_filter;
    return false;
  }
  return true;
}

CollisionCheckResult FclCollisionModel::Check(
    const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses,
    const excavator_interfaces::msg::PlanningScene& scene,
    const std::vector<std::pair<std::string, std::string>>& allowed_pairs,
    const JointPositions& positions, bool check_self_collision,
    bool check_environment_collision,
    const std::string& obstacle_id_filter) const {
  CollisionCheckResult result;
  if (robot_geometries_.empty()) {
    result.message = "FCL collision model has not been initialized";
    return result;
  }
  if (!check_self_collision && !check_environment_collision) {
    result.message = "at least one collision query scope must be enabled";
    return result;
  }

  std::vector<WorldGeometry> robot_objects;
  if (!BuildRobotObjects(link_poses, &robot_objects, &result.message)) {
    return result;
  }

  std::vector<EnvironmentGeometry> environment_objects;
  if (check_environment_collision &&
      !BuildEnvironmentObjects(scene, obstacle_id_filter,
                               &environment_objects, &result.message)) {
    return result;
  }

  if (check_environment_collision) {
    for (const WorldGeometry& robot : robot_objects) {
      for (const EnvironmentGeometry& obstacle : environment_objects) {
        fcl::CollisionRequestd request;
        request.num_max_contacts = 1;
        fcl::CollisionResultd collision_result;
        fcl::collide(robot.object.get(), obstacle.object.get(), request,
                     collision_result);
        if (collision_result.isCollision()) {
          result.valid = true;
          result.collision = true;
          result.first_link = robot.link_name;
          result.second_object = obstacle.id;
          result.collision_positions = positions;
          result.message = "environment collision: " + robot.link_name +
                           " with " + obstacle.id;
          return result;
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

    for (std::size_t first_index = 0; first_index < robot_objects.size();
         ++first_index) {
      for (std::size_t second_index = first_index + 1;
           second_index < robot_objects.size(); ++second_index) {
        const WorldGeometry& first = robot_objects[first_index];
        const WorldGeometry& second = robot_objects[second_index];
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
          result.valid = true;
          result.collision = true;
          result.first_link = first.link_name;
          result.second_object = second.link_name;
          result.collision_positions = positions;
          result.message = "self collision: " + first.link_name + " with " +
                           second.link_name;
          return result;
        }
      }
    }
  }

  result.valid = true;
  result.collision = false;
  result.collision_positions = positions;
  result.checked_states = 1.0;
  result.message = "no collision detected";
  return result;
}

CollisionDistanceResult FclCollisionModel::ComputeDistanceToObstacle(
    const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses,
    const excavator_interfaces::msg::PlanningScene& scene,
    const JointPositions& positions, const std::string& obstacle_id) const {
  CollisionDistanceResult result;
  if (obstacle_id.empty()) {
    result.message = "obstacle_id must not be empty";
    return result;
  }

  std::vector<WorldGeometry> robot_objects;
  if (!BuildRobotObjects(link_poses, &robot_objects, &result.message)) {
    return result;
  }

  std::vector<EnvironmentGeometry> environment_objects;
  if (!BuildEnvironmentObjects(scene, obstacle_id, &environment_objects,
                               &result.message)) {
    return result;
  }
  if (environment_objects.empty()) {
    result.message = "no enabled geometry found for obstacle: " + obstacle_id;
    return result;
  }

  fcl::DistanceRequestd request;
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (const WorldGeometry& robot : robot_objects) {
    fcl::DistanceResultd distance_result;
    const double distance = fcl::distance(
        robot.object.get(), environment_objects.front().object.get(), request,
        distance_result);
    if (!std::isfinite(distance)) {
      result.message = "FCL returned a non-finite distance";
      return result;
    }
    if (distance < minimum_distance) {
      minimum_distance = distance;
      result.robot_link = robot.link_name;
    }
  }

  result.valid = true;
  result.distance = minimum_distance;
  result.obstacle_id = obstacle_id;
  result.joint_positions = positions;
  result.message = "minimum distance computed";
  return result;
}

}  // namespace planning
}  // namespace tramcar
