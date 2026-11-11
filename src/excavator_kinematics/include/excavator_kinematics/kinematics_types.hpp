#ifndef EXCAVATOR_KINEMATICS__KINEMATICS_TYPES_HPP_
#define EXCAVATOR_KINEMATICS__KINEMATICS_TYPES_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace tramcar::planning {

enum class KinematicsStatus {
  kSuccess = 0,
  kInvalidArgument,
  kDimensionMismatch,
  kUnknownLink,
  kUnknownJoint,
  kModelNotReady,
  kUnsupported,
  kJointLimitViolation,
  kSingular,
  kNoConvergence,
  kBackendFailure,
};

struct KinematicsResult {
  KinematicsStatus status{KinematicsStatus::kSuccess};
  std::string message;

  bool ok() const { return status == KinematicsStatus::kSuccess; }
  static KinematicsResult Success() { return {}; }
  static KinematicsResult Failure(KinematicsStatus status,
                                  std::string message) {
    return {status, std::move(message)};
  }
};

struct Pose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct Transform {
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};

using JointVector = Eigen::VectorXd;
using JacobianMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

enum class JointType { kFixed = 0, kRevolute, kContinuous, kPrismatic };

struct JointLimit {
  bool has_position_limits{false};
  double lower{0.0};
  double upper{0.0};
  bool has_velocity_limit{false};
  double max_velocity{0.0};
};

struct LinkModel {
  std::string name;
};

struct JointModel {
  std::string name;
  JointType type{JointType::kFixed};
  std::string parent_link;
  std::string child_link;
  Transform origin;
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  JointLimit limit;
};

struct ModelDescription {
  std::string name;
  std::string root_link;
  std::vector<LinkModel> links;
  std::vector<JointModel> joints;
  std::vector<std::string> joint_order;
};

struct IkOptions {
  bool enforce_joint_limits{true};
  unsigned int max_iterations{150};
  double position_tolerance{1.0e-4};
  double orientation_tolerance{1.0e-4};
};

struct IkRequest {
  std::string target_link;
  Pose target_pose;       // Expressed in model root_link frame.
  JointVector initial_q;  // Ordered according to ModelDescription::joint_order.
  IkOptions options;
};

struct IkResult {
  KinematicsResult result;
  JointVector solution;
  double position_error{0.0};
  double orientation_error{0.0};
};

struct KinematicsSolverConfig {
  std::string backend{"kdl"};
  std::string ik_solver{"nr_jl"};
  unsigned int max_iterations{150};
  double position_tolerance{1.0e-4};
  double orientation_tolerance{1.0e-4};
};

}  // namespace tramcar::planning

#endif  // EXCAVATOR_KINEMATICS__KINEMATICS_TYPES_HPP_
