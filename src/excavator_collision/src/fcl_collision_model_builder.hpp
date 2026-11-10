#ifndef EXCAVATOR_COLLISION__THIRD_LIBRARY__FCL_COLLISION_MODEL_BUILDER_HPP_
#define EXCAVATOR_COLLISION__THIRD_LIBRARY__FCL_COLLISION_MODEL_BUILDER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <fcl/fcl.h>

#include "excavator_collision/collision_checker_interface.hpp"
#include "excavator_collision/robot_collision_geometry_model.hpp"

namespace excavator {
namespace planning {

// One robot link prepared for collision queries. The world transform and the
// FCL collision object are both derived from the joint positions supplied to
// ComputeLinkCollisionObjects().
struct LinkCollisionInfo {
  std::string link_name;
  std::string parent_link_name;
  std::shared_ptr<fcl::CollisionObjectd> object;
};

// FCL adapter for the robot model. This class owns the YAML collision model and
// the borrowed kinematics model, resolves FK link poses from joint positions,
// and returns link collision information. It deliberately contains no
// collision or distance query logic; those live in FclCollisionChecker.
class FclCollisionModelBuilder {
 public:
  ValidationResult Initialize(const CollisionModelConfig& config,
                              const KinematicsModel* kinematics_model);

  const std::vector<std::string>& joint_names() const { return joint_names_; }

  // Computes FK, applies each geometry's link-local transform, and returns one
  // FCL collision object per collision geometry.
  ValidationResult ComputeLinkCollisionObjects(
      const JointPositions& positions,
      std::vector<LinkCollisionInfo>* links) const;

 private:
  using GeometryPtr = std::shared_ptr<fcl::CollisionGeometryd>;

  struct RobotGeometry {
    std::string link_name;
    std::string parent_link_name;
    Eigen::Isometry3d link_to_collision{Eigen::Isometry3d::Identity()};
    GeometryPtr geometry;
  };

  bool BuildRobotGeometries(std::string* error);

  RobotCollisionGeometryModel robot_model_;
  const KinematicsModel* kinematics_model_{nullptr};
  std::vector<std::string> joint_names_;
  std::vector<RobotGeometry> robot_geometries_;
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_COLLISION__THIRD_LIBRARY__FCL_COLLISION_MODEL_BUILDER_HPP_
