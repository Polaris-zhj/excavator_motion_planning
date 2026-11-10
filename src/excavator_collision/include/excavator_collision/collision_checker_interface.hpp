#ifndef EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_
#define EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "data/object/object.h"
#include "excavator_common/joint_trajectory.hpp"
#include "kinematics/kinematics_interface.hpp"

namespace excavator {
namespace planning {

// Obstacle representation selected for one collision query. A single query
// uses one representation for all supplied obstacles.
enum class ObstacleType {
  kPointCloud,
  kGeometry,
};

struct CollisionQueryOptions {
  ObstacleType obstacle_type{ObstacleType::kGeometry};
  // Voxel edge length in meters when obstacle_type == kPointCloud.
  double point_cloud_resolution{0.05};
  // When true, the backend keeps scanning after the first contact and fills
  // CollisionCheckResult::colliding_links with every colliding robot link.
  // Planning keeps the default (false) so it can stop at the first contact.
  bool collect_all_collisions{false};
};

// Project-owned obstacle model. Geometry representation is selected by
// CollisionQueryOptions: kPointCloud uses cloud_points, while kGeometry uses
// an axis-aligned box described by length/width/height and center. Message
// based inputs must be converted by ObstacleAdapter before reaching a backend.
using Obstacle = data::Object;
using ObstacleList = std::vector<Obstacle>;

struct CollisionCheckResult {
  bool valid{false};
  bool collision{false};
  std::string message;
  std::string first_link;
  std::string second_object;
  // All colliding robot links (unique). Filled when
  // CollisionQueryOptions::collect_all_collisions is true; otherwise empty.
  std::vector<std::string> colliding_links;
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
// the robot collision YAML and is parsed into RobotCollisionGeometryModel; it never
// contains FCL types. Kinematics comes from the injected KinematicsModel, so
// no URDF is required by the collision backend.
struct CollisionModelConfig {
  std::vector<std::string> joint_names;
  std::string model_source;
  std::string model_format{"yaml"};
  // Directory used to resolve relative mesh resources in the YAML. Package
  // URIs ("package://...") and absolute paths are resolved independently.
  std::string model_base_directory;
  // Frame returned by the selected KinematicsModel. Obstacle poses are assumed
  // to be expressed in this frame; no frame validation or TF lookup is done.
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

  // Enables or disables one unordered link pair. Allowed pairs are skipped by
  // self-collision queries.
  virtual void SetAllowedCollision(const std::string& link_a,
                                   const std::string& link_b,
                                   bool allowed) = 0;

  virtual CollisionCheckResult CheckCollision(
      const JointPositions& positions, const ObstacleList& obstacles,
      const CollisionQueryOptions& options = {}) const = 0;

  virtual CollisionCheckResult CheckSelfCollision(
      const JointPositions& positions) const = 0;

  virtual CollisionCheckResult CheckEnvironmentCollision(
      const JointPositions& positions, const ObstacleList& obstacles,
      const CollisionQueryOptions& options = {}) const = 0;

  // Discrete joint-space path sampling. This does not prove continuous
  // collision freedom between samples.
  virtual CollisionCheckResult CheckPathCollision(
      const JointPath& path, const ObstacleList& obstacles,
      double max_joint_step,
      const CollisionQueryOptions& options = {}) const = 0;

  // Path query against a single obstacle.
  virtual CollisionCheckResult CheckPathCollision(
      const JointPath& path, const Obstacle& obstacle,
      double max_joint_step,
      const CollisionQueryOptions& options = {}) const = 0;

  virtual CollisionCheckResult CheckTrajectory(
      const JointTrajectory& trajectory, const ObstacleList& obstacles,
      double max_joint_step,
      const CollisionQueryOptions& options = {}) const = 0;

  virtual CollisionDistanceResult ComputeDistance(
      const JointPositions& positions, const Obstacle& obstacle,
      const CollisionQueryOptions& options = {}) const = 0;
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_COLLISION__COLLISION_CHECKER_INTERFACE_HPP_
