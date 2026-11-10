#include "fcl_collision_model_builder.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace excavator {
namespace planning {
namespace {

using Transform = Eigen::Isometry3d;
using Quaternion = Eigen::Quaterniond;

bool IsFinite(const Transform& transform) {
  return transform.matrix().allFinite();
}

Transform ToTransform(const Pose3& pose, std::string* error) {
  Transform transform = Transform::Identity();
  const Eigen::Vector3d translation(pose.position.x, pose.position.y,
                                    pose.position.z);
  Quaternion quaternion(pose.orientation.w, pose.orientation.x,
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
    case CollisionGeometryType::kCapsule:
      return std::make_shared<fcl::Capsuled>(geometry.dimensions.x(),
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

}  // namespace

ValidationResult FclCollisionModelBuilder::Initialize(
    const CollisionModelConfig& config,
    const KinematicsModel* kinematics_model) {
  kinematics_model_ = nullptr;
  joint_names_.clear();
  robot_geometries_.clear();

  if (!config.model_format.empty() && config.model_format != "yaml") {
    return {false, "fcl backend expects a YAML collision model"};
  }
  if (config.model_source.empty() || config.joint_names.empty()) {
    return {false, "collision YAML source and planning joints must not be "
                   "empty"};
  }
  if (kinematics_model == nullptr) {
    return {false, "the fcl backend requires an initialized kinematics model"};
  }
  if (kinematics_model->joint_names() != config.joint_names) {
    return {false, "kinematics joint ordering does not match collision "
                   "configuration"};
  }

  const ValidationResult model_result = robot_model_.InitializeFromYaml(
      config.model_source, config.joint_names,
      config.kinematics_reference_frame, config.model_base_directory);
  if (!model_result.ok) {
    return model_result;
  }
  if (robot_model_.links().empty()) {
    return {false, "robot collision model contains no links"};
  }

  std::string error;
  if (!BuildRobotGeometries(&error)) {
    return {false, error};
  }

  kinematics_model_ = kinematics_model;
  joint_names_ = config.joint_names;
  return {true, ""};
}

bool FclCollisionModelBuilder::BuildRobotGeometries(std::string* error) {
  robot_geometries_.clear();

  for (const LinkCollisionGeometry& link : robot_model_.links()) {
    for (const CollisionGeometryInstance& instance : link.geometries) {
      std::string geometry_error;
      GeometryPtr geometry = CreateFclGeometry(instance.geometry,
                                               &geometry_error);
      if (!geometry) {
        *error = "failed to create FCL geometry for link " + link.link_name +
                 ": " + geometry_error;
        return false;
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
    *error = "robot collision model contains no FCL geometry";
    return false;
  }
  return true;
}

ValidationResult FclCollisionModelBuilder::ComputeLinkCollisionObjects(
    const JointPositions& positions,
    std::vector<LinkCollisionInfo>* links) const {
  if (links == nullptr) {
    return {false, "link collision output is null"};
  }
  if (kinematics_model_ == nullptr || robot_geometries_.empty()) {
    return {false, "FCL collision model has not been initialized"};
  }
  if (positions.size() != joint_names_.size()) {
    return {false, "joint position count does not match configured group"};
  }
  for (double position : positions) {
    if (!std::isfinite(position)) {
      return {false, "joint positions must be finite"};
    }
  }

  links->clear();
  links->reserve(robot_geometries_.size());
  for (const RobotGeometry& geometry : robot_geometries_) {
    Pose3 link_pose;
    const ValidationResult fk = kinematics_model_->GetLinkPose(
        positions, geometry.link_name, &link_pose);
    if (!fk.ok) {
      return {false, "forward kinematics failed for link " +
                         geometry.link_name + ": " + fk.message};
    }

    std::string error;
    // YAML root_frame is defined to be the frame returned by the kinematics
    // model, so no additional frame transform is applied here.
    const Transform world_link = ToTransform(link_pose, &error);
    if (!error.empty()) {
      return {false, error + ": " + geometry.link_name};
    }

    const Transform world_collision =
        world_link * geometry.link_to_collision;
    if (!IsFinite(world_collision)) {
      return {false, "non-finite world collision transform for link: " +
                         geometry.link_name};
    }

    auto object = std::make_shared<fcl::CollisionObjectd>(
        geometry.geometry, world_collision);
    object->computeAABB();
    links->push_back(
        {geometry.link_name, geometry.parent_link_name, std::move(object)});
  }
  return {true, ""};
}

}  // namespace planning
}  // namespace excavator
