#ifndef EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_
#define EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_interfaces/msg/planning_scene.hpp"
#include "kinematics/kinematics_interface.hpp"

namespace tramcar {
namespace planning {

struct CollisionCheckResult {
  bool valid{false};
  bool collision{false};
  std::string message;
  std::string first_link;
  std::string second_object;
  JointPositions collision_positions;
  double checked_states{0.0};
};

struct CollisionDistanceResult {
  bool valid{false};
  double distance{std::numeric_limits<double>::infinity()};
  std::string robot_link;
  std::string obstacle_id;
  JointPositions joint_positions;
  std::string message;
};

// Backend-independent collision model description. `model_source` contains
// the robot collision YAML and is parsed into RobotCollisionModel; it never
// contains FCL types. Kinematics comes from the injected KinematicsModel, so
// no URDF is required by the collision backend.
struct CollisionModelConfig {
  std::vector<std::string> joint_names;
  std::string model_source;
  std::string model_format{"yaml"};
  // Directory used to resolve relative mesh resources in the YAML. Package
  // URIs ("package://...") and absolute paths are resolved independently.
  std::string model_base_directory;
  // Frame returned by the selected KinematicsModel and used by PlanningScene.
  // With no URDF frame tree there is no implicit transform, so the YAML
  // robot.root_frame must equal this value.
  std::string kinematics_reference_frame{"base_link"};
};

// Planning and application code depends on this interface, not a concrete
// collision backend. The kinematics model is borrowed and must outlive this
// checker. A kinematics model is required: the collision backend does not
// parse URDF or construct a fallback FK model.
class CollisionChecker {
 public:
  virtual ~CollisionChecker() = default;

  virtual ValidationResult Initialize(
      const CollisionModelConfig& config,
      const KinematicsModel* kinematics_model) = 0;

  virtual ValidationResult Initialize(const CollisionModelConfig& config) {
    return Initialize(config, nullptr);
  }

  virtual void SetAllowedCollisionPairs(
      const std::vector<std::pair<std::string, std::string>>& pairs) = 0;

  virtual CollisionCheckResult CheckState(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene) const = 0;

  virtual CollisionCheckResult CheckSelfCollision(
      const JointPositions& positions) const = 0;

  virtual CollisionCheckResult CheckEnvironmentCollision(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene) const = 0;

  // Discrete joint-space path sampling. This does not prove continuous
  // collision freedom between samples.
  virtual CollisionCheckResult CheckPath(
      const JointPath& path,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const = 0;

  virtual CollisionCheckResult CheckPathCollisionWithObstacle(
      const JointPath& path,
      const excavator_interfaces::msg::PlanningScene& scene,
      const std::string& obstacle_id, double max_joint_step) const = 0;

  virtual CollisionCheckResult CheckTrajectory(
      const JointTrajectory& trajectory,
      const excavator_interfaces::msg::PlanningScene& scene,
      double max_joint_step) const = 0;

  virtual CollisionDistanceResult ComputeDistanceToObstacle(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene,
      const std::string& obstacle_id) const = 0;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_
