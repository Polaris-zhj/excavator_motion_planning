#include "kinematics/urdf_kinematics_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <urdf/model.h>

namespace tramcar {
namespace planning {
namespace {

using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

struct JointMotion {
  std::string name;
  urdf::JointConstSharedPtr joint;
  Isometry3d parent_to_joint{Isometry3d::Identity()};
  Vector3d axis{Vector3d::Zero()};
  std::size_t group_index{0};
  bool in_group{false};
};

Isometry3d ToEigen(const urdf::Pose& pose) {
  Isometry3d result = Isometry3d::Identity();
  result.translation() =
      Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Quaterniond(pose.rotation.w, pose.rotation.x,
                                pose.rotation.y, pose.rotation.z)
                        .normalized()
                        .toRotationMatrix();
  return result;
}

Vector3d ToEigen(const urdf::Vector3& vector) {
  return Vector3d(vector.x, vector.y, vector.z);
}

Pose3 ToPose3(const Isometry3d& transform) {
  const Quaterniond q(transform.rotation());
  Pose3 pose;
  pose.position = {transform.translation().x(), transform.translation().y(),
                   transform.translation().z()};
  pose.orientation = {q.x(), q.y(), q.z(), q.w()};
  return pose;
}

bool IsFinite(const Pose3& pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

ValidationResult ValidateJointVector(const JointPositions& q,
                                     std::size_t expected_size) {
  if (q.size() != expected_size) {
    return {false, "joint position count does not match configured joint group"};
  }
  for (double value : q) {
    if (!std::isfinite(value)) {
      return {false, "joint positions must be finite"};
    }
  }
  return {true, ""};
}

}  // namespace

struct UrdfKinematicsModel::Impl {
  urdf::Model model;
  bool initialized{false};
  std::unordered_map<std::string, std::pair<double, double>> limits;
  std::unordered_map<std::string, double> velocity_limits;

  ValidationResult BuildChain(const std::string& link_name,
                              std::vector<JointMotion>* chain) const {
    if (!initialized) {
      return {false, "URDF model has not been initialized"};
    }
    if (chain == nullptr) {
      return {false, "chain output pointer is null"};
    }
    auto link = model.getLink(link_name);
    if (!link) {
      return {false, "unknown target link: " + link_name};
    }

    std::vector<JointMotion> reversed;
    while (link && link->name != "base_link" && link->parent_joint) {
      JointMotion motion;
      motion.name = link->parent_joint->name;
      motion.joint = link->parent_joint;
      motion.parent_to_joint = ToEigen(
          link->parent_joint->parent_to_joint_origin_transform);
      motion.axis = ToEigen(link->parent_joint->axis);
      reversed.push_back(std::move(motion));
      link = model.getLink(link->parent_joint->parent_link_name);
    }
    chain->assign(reversed.rbegin(), reversed.rend());
    return {true, ""};
  }
};

UrdfKinematicsModel::UrdfKinematicsModel(
    std::vector<std::string> joint_names)
    : impl_(std::make_unique<Impl>()), joint_names_(std::move(joint_names)) {}

UrdfKinematicsModel::~UrdfKinematicsModel() = default;

ValidationResult UrdfKinematicsModel::InitializeFromFile(
    const std::string& urdf_path) {
  if (urdf_path.empty()) {
    return {false, "URDF path must not be empty"};
  }
  if (!impl_->model.initFile(urdf_path)) {
    return {false, "failed to load URDF file: " + urdf_path};
  }
  if (joint_names_.empty()) {
    return {false, "configured joint group must not be empty"};
  }

  impl_->limits.clear();
  impl_->velocity_limits.clear();
  for (const std::string& joint_name : joint_names_) {
    const auto joint = impl_->model.getJoint(joint_name);
    if (!joint) {
      return {false, "configured joint is missing from URDF: " + joint_name};
    }
    if (joint->type != urdf::Joint::REVOLUTE &&
        joint->type != urdf::Joint::CONTINUOUS &&
        joint->type != urdf::Joint::PRISMATIC) {
      return {false, "configured planning joint is not movable: " +
                         joint_name};
    }
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
    if (joint->limits && joint->type != urdf::Joint::CONTINUOUS) {
      lower = joint->limits->lower;
      upper = joint->limits->upper;
    }
    impl_->limits[joint_name] = {lower, upper};
    impl_->velocity_limits[joint_name] =
        joint->limits ? joint->limits->velocity
                      : std::numeric_limits<double>::infinity();
  }
  impl_->initialized = true;
  return {true, ""};
}

ValidationResult UrdfKinematicsModel::InitializeFromString(
    const std::string& urdf_xml) {
  if (urdf_xml.empty()) {
    return {false, "URDF XML string must not be empty"};
  }
  if (!impl_->model.initString(urdf_xml)) {
    return {false, "failed to parse URDF XML string"};
  }
  if (joint_names_.empty()) {
    return {false, "configured joint group must not be empty"};
  }

  impl_->limits.clear();
  impl_->velocity_limits.clear();
  for (const std::string& joint_name : joint_names_) {
    const auto joint = impl_->model.getJoint(joint_name);
    if (!joint) {
      return {false, "configured joint is missing from URDF: " + joint_name};
    }
    if (joint->type != urdf::Joint::REVOLUTE &&
        joint->type != urdf::Joint::CONTINUOUS &&
        joint->type != urdf::Joint::PRISMATIC) {
      return {false, "configured planning joint is not movable: " +
                         joint_name};
    }
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
    if (joint->limits && joint->type != urdf::Joint::CONTINUOUS) {
      lower = joint->limits->lower;
      upper = joint->limits->upper;
    }
    impl_->limits[joint_name] = {lower, upper};
    impl_->velocity_limits[joint_name] =
        joint->limits ? joint->limits->velocity
                      : std::numeric_limits<double>::infinity();
  }
  impl_->initialized = true;
  return {true, ""};
}

std::vector<std::pair<double, double>>
UrdfKinematicsModel::GetPositionLimits() const {
  std::vector<std::pair<double, double>> limits;
  limits.reserve(joint_names_.size());
  for (const std::string& joint_name : joint_names_) {
    const auto iter = impl_->limits.find(joint_name);
    if (iter == impl_->limits.end()) {
      return {};
    }
    limits.push_back(iter->second);
  }
  return limits;
}

std::vector<double> UrdfKinematicsModel::GetVelocityLimits() const {
  std::vector<double> limits;
  limits.reserve(joint_names_.size());
  for (const std::string& joint_name : joint_names_) {
    const auto iter = impl_->velocity_limits.find(joint_name);
    if (iter == impl_->velocity_limits.end()) {
      return {};
    }
    limits.push_back(iter->second);
  }
  return limits;
}

const std::vector<std::string>& UrdfKinematicsModel::joint_names() const {
  return joint_names_;
}

ValidationResult UrdfKinematicsModel::GetLinkPose(
    const JointPositions& joint_positions, const std::string& link_name,
    Pose3* pose) const {
  const auto input_result =
      ValidateJointVector(joint_positions, joint_names_.size());
  if (!input_result.ok) {
    return input_result;
  }
  if (pose == nullptr) {
    return {false, "pose output pointer is null"};
  }

  std::vector<JointMotion> chain;
  const auto chain_result = impl_->BuildChain(link_name, &chain);
  if (!chain_result.ok) {
    return chain_result;
  }
  std::unordered_map<std::string, std::size_t> indices;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    indices[joint_names_[i]] = i;
  }

  Isometry3d transform = Isometry3d::Identity();
  for (const JointMotion& motion : chain) {
    transform = transform * motion.parent_to_joint;
    const auto index = indices.find(motion.name);
    if (index == indices.end()) {
      continue;
    }
    const double q = joint_positions[index->second];
    if (motion.joint->type == urdf::Joint::REVOLUTE ||
        motion.joint->type == urdf::Joint::CONTINUOUS) {
      transform.rotate(Eigen::AngleAxisd(q, motion.axis.normalized()));
    } else if (motion.joint->type == urdf::Joint::PRISMATIC) {
      transform.translate(motion.axis.normalized() * q);
    }
  }
  *pose = ToPose3(transform);
  return IsFinite(*pose) ? ValidationResult{true, ""}
                         : ValidationResult{false, "FK produced a non-finite pose"};
}

ValidationResult UrdfKinematicsModel::GetJacobian(
    const JointPositions& joint_positions, const std::string& link_name,
    Jacobian* jacobian) const {
  const auto input_result =
      ValidateJointVector(joint_positions, joint_names_.size());
  if (!input_result.ok) {
    return input_result;
  }
  if (jacobian == nullptr) {
    return {false, "Jacobian output pointer is null"};
  }

  std::vector<JointMotion> chain;
  const auto chain_result = impl_->BuildChain(link_name, &chain);
  if (!chain_result.ok) {
    return chain_result;
  }
  std::unordered_map<std::string, std::size_t> indices;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    indices[joint_names_[i]] = i;
  }

  struct ActiveAxis {
    std::size_t index;
    Vector3d origin;
    Vector3d axis;
    bool prismatic;
  };
  std::vector<ActiveAxis> active_axes;
  Isometry3d transform = Isometry3d::Identity();
  for (const JointMotion& motion : chain) {
    transform = transform * motion.parent_to_joint;
    const auto index = indices.find(motion.name);
    if (index == indices.end()) {
      continue;
    }
    const Vector3d axis_world =
        (transform.rotation() * motion.axis.normalized()).normalized();
    active_axes.push_back(
        {index->second, transform.translation(), axis_world,
         motion.joint->type == urdf::Joint::PRISMATIC});

    const double q = joint_positions[index->second];
    if (motion.joint->type == urdf::Joint::REVOLUTE ||
        motion.joint->type == urdf::Joint::CONTINUOUS) {
      transform.rotate(Eigen::AngleAxisd(q, motion.axis.normalized()));
    } else if (motion.joint->type == urdf::Joint::PRISMATIC) {
      transform.translate(motion.axis.normalized() * q);
    }
  }

  MatrixXd matrix = MatrixXd::Zero(6, static_cast<Eigen::Index>(joint_names_.size()));
  const Vector3d end_position = transform.translation();
  for (const ActiveAxis& active : active_axes) {
    const Eigen::Index column = static_cast<Eigen::Index>(active.index);
    if (active.prismatic) {
      matrix.block<3, 1>(0, column) = active.axis;
    } else {
      matrix.block<3, 1>(0, column) =
          active.axis.cross(end_position - active.origin);
      matrix.block<3, 1>(3, column) = active.axis;
    }
  }

  jacobian->rows.assign(6, std::vector<double>(joint_names_.size(), 0.0));
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      jacobian->rows[static_cast<std::size_t>(row)]
                    [static_cast<std::size_t>(col)] = matrix(row, col);
    }
  }
  return {matrix.allFinite(), matrix.allFinite() ? "" : "Jacobian is non-finite"};
}

InverseKinematicsResult UrdfKinematicsModel::SolveIK(
    const Pose3& target_pose, const std::string& link_name,
    const JointPositions& seed, const InverseKinematicsOptions& options) const {
  InverseKinematicsResult result;
  result.joint_positions = seed;
  const auto seed_result = ValidateJointVector(seed, joint_names_.size());
  if (!seed_result.ok) {
    result.message = seed_result.message;
    return result;
  }
  if (!IsFinite(target_pose) || options.max_iterations == 0 ||
      options.position_tolerance <= 0.0 ||
      options.orientation_tolerance <= 0.0 || options.damping <= 0.0) {
    result.message = "invalid IK target or solver options";
    return result;
  }

  Quaterniond target_orientation(target_pose.orientation.w,
                                 target_pose.orientation.x,
                                 target_pose.orientation.y,
                                 target_pose.orientation.z);
  if (target_orientation.norm() < 1.0e-12) {
    result.message = "target orientation quaternion has zero norm";
    return result;
  }
  target_orientation.normalize();
  const Vector3d target_position(target_pose.position.x,
                                 target_pose.position.y,
                                 target_pose.position.z);
  VectorXd q = Eigen::Map<const VectorXd>(seed.data(),
      static_cast<Eigen::Index>(seed.size()));

  for (std::size_t iteration = 0; iteration < options.max_iterations;
       ++iteration) {
    const JointPositions current_q(q.data(), q.data() + q.size());
    Pose3 current_pose;
    const auto fk_result = GetLinkPose(current_q, link_name, &current_pose);
    if (!fk_result.ok) {
      result.message = fk_result.message;
      return result;
    }
    const Vector3d current_position(current_pose.position.x,
                                    current_pose.position.y,
                                    current_pose.position.z);
    Quaterniond current_orientation(current_pose.orientation.w,
                                    current_pose.orientation.x,
                                    current_pose.orientation.y,
                                    current_pose.orientation.z);
    Quaterniond delta_orientation =
        (target_orientation * current_orientation.conjugate()).normalized();
    if (delta_orientation.w() < 0.0) {
      delta_orientation.coeffs() *= -1.0;
    }
    const Eigen::AngleAxisd angle_axis(delta_orientation);
    const Vector3d orientation_error = angle_axis.axis() * angle_axis.angle();

    const Vector3d position_error = target_position - current_position;
    if (position_error.norm() <= options.position_tolerance &&
        orientation_error.norm() <= options.orientation_tolerance) {
      result.success = true;
      result.joint_positions = current_q;
      result.iterations = iteration;
      result.message = "IK converged";
      return result;
    }

    Jacobian jacobian_data;
    const auto jacobian_result =
        GetJacobian(current_q, link_name, &jacobian_data);
    if (!jacobian_result.ok) {
      result.message = jacobian_result.message;
      return result;
    }
    MatrixXd j(6, static_cast<Eigen::Index>(joint_names_.size()));
    for (Eigen::Index row = 0; row < 6; ++row) {
      for (Eigen::Index col = 0; col < j.cols(); ++col) {
        j(row, col) = jacobian_data.rows[static_cast<std::size_t>(row)]
                                       [static_cast<std::size_t>(col)];
      }
    }
    VectorXd error(6);
    error.head<3>() = position_error;
    error.tail<3>() = orientation_error;
    const MatrixXd normal =
        j.transpose() * j +
        options.damping * options.damping *
            MatrixXd::Identity(j.cols(), j.cols());
    const VectorXd delta = normal.ldlt().solve(j.transpose() * error);
    if (!delta.allFinite()) {
      result.message = "IK solver produced a non-finite update";
      return result;
    }
    q += delta;
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto limits = impl_->limits.at(joint_names_[i]);
      q[static_cast<Eigen::Index>(i)] =
          std::clamp(q[static_cast<Eigen::Index>(i)], limits.first, limits.second);
    }
    result.iterations = iteration + 1;
  }
  result.joint_positions.assign(q.data(), q.data() + q.size());
  result.message = "IK did not converge within the iteration limit";
  return result;
}

}  // namespace planning
}  // namespace tramcar
