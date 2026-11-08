#ifndef EXCAVATOR_COLLISION__ROBOT_COLLISION_MODEL_HPP_
#define EXCAVATOR_COLLISION__ROBOT_COLLISION_MODEL_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

enum class CollisionGeometryType {
  kBox,
  kSphere,
  kCylinder,
  kCapsule,
  kTriangleMesh,
};

struct CollisionGeometry {
  CollisionGeometryType type{CollisionGeometryType::kBox};

  // Box: x/y/z size. Sphere: x is radius. Cylinder and capsule: x is radius
  // and y is length. Triangle mesh data is stored in
  // mesh_vertices/mesh_triangles.
  Eigen::Vector3d dimensions{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> mesh_vertices;
  std::vector<std::array<std::size_t, 3>> mesh_triangles;
};

struct CollisionGeometryInstance {
  // Fixed transform from the parent link frame to this geometry frame.
  Eigen::Isometry3d link_to_collision{Eigen::Isometry3d::Identity()};
  CollisionGeometry geometry;
};

struct LinkCollisionGeometry {
  std::string link_name;
  // Optional kinematic parent, used only to skip adjacent link pairs in
  // self-collision checks.
  std::string parent_link_name;
  std::vector<CollisionGeometryInstance> geometries;
};

// Project-owned collision description. It deliberately contains no FCL types
// and does not calculate link poses. KinematicsModel and RobotCollisionModel
// remain independent and are joined by link_name at query time.
//
// The robot collision geometry is loaded from YAML; no URDF parsing happens
// here. The referenced link frames must be the frames produced by the selected
// KinematicsModel (for example the physical link frames of the DH model).
class RobotCollisionModel {
 public:
  ValidationResult InitializeFromYaml(
      const std::string& yaml,
      const std::vector<std::string>& planning_joint_names,
      const std::string& reference_frame = "base_link",
      const std::string& base_directory = "");

  ValidationResult InitializeFromYamlFile(
      const std::string& path,
      const std::vector<std::string>& planning_joint_names,
      const std::string& reference_frame = "base_link");

  // Frame in which link poses and environment obstacle poses are expressed.
  // KinematicsModel must return poses in this frame.
  const std::string& root_frame() const { return root_frame_; }
  const std::string& reference_frame() const { return reference_frame_; }
  const std::vector<std::string>& joint_names() const { return joint_names_; }
  const std::vector<LinkCollisionGeometry>& links() const { return links_; }

 private:
  std::string root_frame_;
  std::string reference_frame_;
  std::vector<std::string> joint_names_;
  std::vector<LinkCollisionGeometry> links_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__ROBOT_COLLISION_MODEL_HPP_
