#ifndef EXCAVATOR_COLLISION__THIRD_LIBRARY__FCL_COLLISION_MODEL_HPP_
#define EXCAVATOR_COLLISION__THIRD_LIBRARY__FCL_COLLISION_MODEL_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <fcl/fcl.h>

#include "excavator_collision/collision_checker_interface.hpp"
#include "excavator_collision/internal/robot_collision_model.hpp"
#include "excavator_interfaces/msg/planning_scene.hpp"

namespace tramcar {
namespace planning {

// FCL-only adapter. All FCL types are confined to this third_library folder.
// It owns FCL geometry objects and combines FK link poses with each geometry's
// link-local transform before issuing collision/distance queries.
class FclCollisionModel {
 public:
  ValidationResult Initialize(const RobotCollisionModel& robot_model);

  CollisionCheckResult Check(
      const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses,
      const excavator_interfaces::msg::PlanningScene& scene,
      const std::vector<std::pair<std::string, std::string>>& allowed_pairs,
      const JointPositions& positions, bool check_self_collision,
      bool check_environment_collision,
      const std::string& obstacle_id_filter = {}) const;

  CollisionDistanceResult ComputeDistanceToObstacle(
      const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses,
      const excavator_interfaces::msg::PlanningScene& scene,
      const JointPositions& positions, const std::string& obstacle_id) const;

 private:
  using GeometryPtr = std::shared_ptr<fcl::CollisionGeometryd>;

  struct RobotGeometry {
    std::string link_name;
    std::string parent_link_name;
    Eigen::Isometry3d link_to_collision{Eigen::Isometry3d::Identity()};
    GeometryPtr geometry;
  };

  struct WorldGeometry {
    std::string link_name;
    std::string parent_link_name;
    std::shared_ptr<fcl::CollisionObjectd> object;
  };

  struct EnvironmentGeometry {
    std::string id;
    std::shared_ptr<fcl::CollisionObjectd> object;
  };

  bool BuildRobotObjects(
      const std::unordered_map<std::string, Eigen::Isometry3d>& link_poses,
      std::vector<WorldGeometry>* objects, std::string* error) const;

  bool BuildEnvironmentObjects(
      const excavator_interfaces::msg::PlanningScene& scene,
      const std::string& obstacle_id_filter,
      std::vector<EnvironmentGeometry>* objects, std::string* error) const;

  std::string root_frame_;
  std::vector<RobotGeometry> robot_geometries_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__THIRD_LIBRARY__FCL_COLLISION_MODEL_HPP_
