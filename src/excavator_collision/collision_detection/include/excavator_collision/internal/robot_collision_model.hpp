#ifndef EXCAVATOR_COLLISION__INTERNAL__ROBOT_COLLISION_MODEL_HPP_
#define EXCAVATOR_COLLISION__INTERNAL__ROBOT_COLLISION_MODEL_HPP_

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
  kTriangleMesh,
};

struct CollisionGeometry {
  CollisionGeometryType type{CollisionGeometryType::kBox};

  // Box: x/y/z size. Sphere: x is radius. Cylinder: x is radius and y
  // is length. Triangle mesh data is stored in mesh_vertices/mesh_triangles.
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
  std::string parent_link_name;
  std::vector<CollisionGeometryInstance> geometries;
};

// Project-owned collision description. It deliberately contains no FCL types
// and does not calculate link poses. KinematicsModel and RobotCollisionModel
// remain independent and are joined by link_name at query time.
class RobotCollisionModel {
 public:
  ValidationResult InitializeFromUrdf(
      const std::string& urdf_xml,
      const std::vector<std::string>& planning_joint_names);

  const std::string& root_frame() const { return root_frame_; }
  const std::vector<std::string>& joint_names() const { return joint_names_; }
  const std::vector<LinkCollisionGeometry>& links() const { return links_; }

 private:
  std::string root_frame_;
  std::vector<std::string> joint_names_;
  std::vector<LinkCollisionGeometry> links_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__INTERNAL__ROBOT_COLLISION_MODEL_HPP_
