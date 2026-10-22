#include "kinematics/dh_kinematics_model.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

namespace tramcar {
namespace planning {
namespace {

using Clock = std::chrono::steady_clock;

Eigen::Isometry3d Rx(double angle) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX())
                           .toRotationMatrix();
  return transform;
}

Eigen::Isometry3d Rz(double angle) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
                           .toRotationMatrix();
  return transform;
}

Eigen::Isometry3d Tx(double distance) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = distance;
  return transform;
}

Eigen::Isometry3d Tz(double distance) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = distance;
  return transform;
}

bool IsFinite(const Eigen::Isometry3d& transform) {
  return transform.matrix().allFinite();
}

Pose3 ToPose(const Eigen::Isometry3d& transform) {
  Pose3 pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

double OrientationAngle(const Eigen::Quaterniond& q_target,
                        const Eigen::Quaterniond& q_current) {
  Eigen::Quaterniond error = q_target * q_current.inverse();
  error.normalize();
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }
  const double w = std::clamp(error.w(), -1.0, 1.0);
  return 2.0 * std::acos(w);
}

Eigen::Vector3d OrientationError(const Eigen::Quaterniond& q_target,
                                 const Eigen::Quaterniond& q_current) {
  Eigen::Quaterniond error = q_target * q_current.inverse();
  error.normalize();
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }
  const double angle = OrientationAngle(q_target, q_current);
  const Eigen::Vector3d vector = error.vec();
  if (vector.norm() < 1.0e-12) {
    return Eigen::Vector3d::Zero();
  }
  return vector.normalized() * angle;
}

}  // namespace

DhKinematicsModel::DhKinematicsModel(std::vector<std::string> joint_names)
    : joint_names_(std::move(joint_names)) {}

DhKinematicsModel::~DhKinematicsModel() = default;

ValidationResult DhKinematicsModel::InitializeFromFile(
    const std::string& path) {
  const ValidationResult loaded = parameters_.LoadFromFile(path);
  if (!loaded.ok) {
    return loaded;
  }
  const std::vector<std::string> names = parameters_.JointNames();
  if (joint_names_.empty()) {
    joint_names_ = names;
  } else if (joint_names_ != names) {
    return {false, "DH joint order does not match the configured joint_names"};
  }
  initialized_ = true;
  return {true, ""};
}

ValidationResult DhKinematicsModel::InitializeFromString(
    const std::string& yaml) {
  const ValidationResult loaded = parameters_.LoadFromString(yaml);
  if (!loaded.ok) {
    return loaded;
  }
  const std::vector<std::string> names = parameters_.JointNames();
  if (joint_names_.empty()) {
    joint_names_ = names;
  } else if (joint_names_ != names) {
    return {false, "DH joint order does not match the configured joint_names"};
  }
  initialized_ = true;
  return {true, ""};
}

const std::vector<std::string>& DhKinematicsModel::joint_names() const {
  return joint_names_;
}

std::vector<std::pair<double, double>>
DhKinematicsModel::GetPositionLimits() const {
  std::vector<std::pair<double, double>> limits;
  limits.reserve(parameters_.joints.size());
  for (const DhJoint& joint : parameters_.joints) {
    limits.emplace_back(joint.lower, joint.upper);
  }
  return limits;
}

std::vector<double> DhKinematicsModel::GetVelocityLimits() const {
  std::vector<double> limits;
  limits.reserve(parameters_.joints.size());
  for (const DhJoint& joint : parameters_.joints) {
    limits.push_back(joint.velocity);
  }
  return limits;
}

ValidationResult DhKinematicsModel::BuildFrames(
    const JointPositions& positions,
    std::vector<Eigen::Isometry3d>* frames) const {
  if (!initialized_) {
    return {false, "DH model is not initialized"};
  }
  if (frames == nullptr) {
    return {false, "frame output pointer is null"};
  }
  if (positions.size() != parameters_.joints.size()) {
    return {false, "joint position count does not match the DH model"};
  }
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (!std::isfinite(positions[index])) {
      return {false, "joint positions must be finite"};
    }
    if (positions[index] < parameters_.joints[index].lower ||
        positions[index] > parameters_.joints[index].upper) {
      return {false, "joint position violates the DH joint limit"};
    }
  }

  frames->resize(parameters_.joints.size() + 1);
  (*frames)[0] = parameters_.base;
  for (std::size_t index = 0; index < parameters_.joints.size(); ++index) {
    const DhJoint& joint = parameters_.joints[index];
    const Eigen::Isometry3d pre =
        (*frames)[index] * Rx(joint.alpha_prev) * Tx(joint.a_prev);
    (*frames)[index + 1] =
        pre * Rz(joint.theta_sign * positions[index] +
                 joint.theta_offset) * Tz(joint.d);
    if (!IsFinite((*frames)[index + 1])) {
      return {false, "DH forward kinematics produced a non-finite frame"};
    }
  }
  return {true, ""};
}

int DhKinematicsModel::LinkJointCount(const std::string& link_name) const {
  if (link_name == "base_link") {
    return 0;
  }
  if (link_name == "bucket_tip") {
    return static_cast<int>(parameters_.joints.size());
  }
  for (std::size_t index = 0; index < parameters_.joints.size(); ++index) {
    if (parameters_.joints[index].link == link_name) {
      return static_cast<int>(index + 1);
    }
  }
  return -1;
}

ValidationResult DhKinematicsModel::LinkPose(
    const JointPositions& positions, const std::string& link_name,
    Eigen::Isometry3d* transform) const {
  if (transform == nullptr) {
    return {false, "pose output pointer is null"};
  }
  const int count = LinkJointCount(link_name);
  if (count < 0) {
    return {false, "link is not part of the DH model: " + link_name};
  }
  std::vector<Eigen::Isometry3d> frames;
  const ValidationResult built = BuildFrames(positions, &frames);
  if (!built.ok) {
    return built;
  }
  if (link_name == "base_link") {
    *transform = Eigen::Isometry3d::Identity();
    return {true, ""};
  }
  const auto offset = parameters_.link_offsets.find(link_name);
  const Eigen::Isometry3d link_offset =
      offset == parameters_.link_offsets.end()
          ? Eigen::Isometry3d::Identity()
          : offset->second;
  *transform = frames[static_cast<std::size_t>(count)] * link_offset;
  if (!IsFinite(*transform)) {
    return {false, "DH link pose is non-finite"};
  }
  return {true, ""};
}

ValidationResult DhKinematicsModel::GetLinkPose(
    const JointPositions& joint_positions, const std::string& link_name,
    Pose3* pose) const {
  if (pose == nullptr) {
    return {false, "pose output pointer is null"};
  }
  Eigen::Isometry3d transform;
  const ValidationResult result =
      LinkPose(joint_positions, link_name, &transform);
  if (!result.ok) {
    return result;
  }
  *pose = ToPose(transform);
  return {true, ""};
}

ValidationResult DhKinematicsModel::GetJacobian(
    const JointPositions& joint_positions, const std::string& link_name,
    Jacobian* jacobian) const {
  if (jacobian == nullptr) {
    return {false, "Jacobian output pointer is null"};
  }
  const int count = LinkJointCount(link_name);
  if (count < 0) {
    return {false, "link is not part of the DH model: " + link_name};
  }
  std::vector<Eigen::Isometry3d> frames;
  const ValidationResult built = BuildFrames(joint_positions, &frames);
  if (!built.ok) {
    return built;
  }
  Eigen::Isometry3d target;
  const ValidationResult target_result =
      LinkPose(joint_positions, link_name, &target);
  if (!target_result.ok) {
    return target_result;
  }
  const Eigen::Vector3d target_position = target.translation();

  const std::size_t joints = parameters_.joints.size();
  jacobian->rows.assign(6, std::vector<double>(joints, 0.0));
  for (int index = 0; index < count; ++index) {
    const DhJoint& joint =
        parameters_.joints[static_cast<std::size_t>(index)];
    const Eigen::Isometry3d pre =
        frames[static_cast<std::size_t>(index)] *
        Rx(joint.alpha_prev) * Tx(joint.a_prev);
    const Eigen::Vector3d axis_line = pre.linear().col(2).normalized();
    const Eigen::Vector3d axis = joint.theta_sign * axis_line;
    const Eigen::Vector3d point =
        pre.translation() + axis_line * joint.d;
    const Eigen::Vector3d linear =
        axis.cross(target_position - point);
    jacobian->rows[0][static_cast<std::size_t>(index)] = linear.x();
    jacobian->rows[1][static_cast<std::size_t>(index)] = linear.y();
    jacobian->rows[2][static_cast<std::size_t>(index)] = linear.z();
    jacobian->rows[3][static_cast<std::size_t>(index)] = axis.x();
    jacobian->rows[4][static_cast<std::size_t>(index)] = axis.y();
    jacobian->rows[5][static_cast<std::size_t>(index)] = axis.z();
  }
  return {true, ""};
}

InverseKinematicsResult DhKinematicsModel::SolveIK(
    const Pose3& target_pose, const std::string& link_name,
    const JointPositions& seed,
    const InverseKinematicsOptions& options) const {
  InverseKinematicsResult result;
  if (!initialized_) {
    result.message = "DH model is not initialized";
    return result;
  }
  if (seed.size() != parameters_.joints.size()) {
    result.message = "IK seed size does not match the DH model";
    return result;
  }
  const int count = LinkJointCount(link_name);
  if (count < 0) {
    result.message = "link is not part of the DH model: " + link_name;
    return result;
  }
  const Eigen::Vector3d target_position(
      target_pose.position.x, target_pose.position.y,
      target_pose.position.z);
  Eigen::Quaterniond target_orientation(
      target_pose.orientation.w, target_pose.orientation.x,
      target_pose.orientation.y, target_pose.orientation.z);
  target_orientation.normalize();

  JointPositions positions = seed;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    positions[index] = std::clamp(
        positions[index], parameters_.joints[index].lower,
        parameters_.joints[index].upper);
  }

  const std::size_t max_iterations =
      std::max<std::size_t>(1, options.max_iterations);
  const double position_tolerance =
      std::max(1.0e-9, options.position_tolerance);
  const double orientation_tolerance =
      std::max(1.0e-9, options.orientation_tolerance);
  const double orientation_weight =
      std::max(0.0, options.orientation_weight);
  const double damping = std::max(1.0e-9, options.damping);
  const double max_step =
      options.max_joint_step > 0.0 ? options.max_joint_step : 0.2;
  const auto deadline =
      Clock::now() + std::chrono::duration<double>(
                         std::max(0.001, options.time_budget_seconds));

  const std::size_t joint_count = parameters_.joints.size();
  for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
    if (Clock::now() >= deadline) {
      result.message = "IK time budget exhausted";
      result.joint_positions = positions;
      result.iterations = iteration;
      return result;
    }

    Eigen::Isometry3d current;
    const ValidationResult pose_result =
        LinkPose(positions, link_name, &current);
    if (!pose_result.ok) {
      result.message = pose_result.message;
      return result;
    }
    const Eigen::Quaterniond current_orientation(current.linear());
    const Eigen::Vector3d position_error =
        target_position - current.translation();
    const Eigen::Vector3d orientation_error =
        OrientationError(target_orientation, current_orientation);
    const double orientation_angle =
        OrientationAngle(target_orientation, current_orientation);

    if (position_error.norm() <= position_tolerance &&
        (orientation_weight <= 0.0 ||
         orientation_angle <= orientation_tolerance)) {
      result.success = true;
      result.joint_positions = positions;
      result.iterations = iteration;
      result.message = "IK converged";
      return result;
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, joint_count);
    std::vector<Eigen::Isometry3d> frames;
    const ValidationResult frames_result = BuildFrames(positions, &frames);
    if (!frames_result.ok) {
      result.message = frames_result.message;
      return result;
    }
    jacobian.setZero();
    for (int joint_index = 0; joint_index < count; ++joint_index) {
      const DhJoint& joint =
          parameters_.joints[static_cast<std::size_t>(joint_index)];
      const Eigen::Isometry3d pre =
          frames[static_cast<std::size_t>(joint_index)] *
          Rx(joint.alpha_prev) * Tx(joint.a_prev);
      const Eigen::Vector3d axis_line = pre.linear().col(2).normalized();
      const Eigen::Vector3d axis = joint.theta_sign * axis_line;
      const Eigen::Vector3d point =
          pre.translation() + axis_line * joint.d;
      const Eigen::Vector3d linear =
          axis.cross(target_position - point);
      jacobian.block<3, 1>(0, joint_index) = linear;
      jacobian.block<3, 1>(3, joint_index) = axis;
    }
    jacobian.block(3, 0, 3, joint_count) *= orientation_weight;

    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = position_error;
    error.tail<3>() = orientation_error * orientation_weight;
    const Eigen::MatrixXd normal =
        jacobian.transpose() * jacobian +
        damping * damping * Eigen::MatrixXd::Identity(joint_count,
                                                      joint_count);
    const Eigen::VectorXd rhs = jacobian.transpose() * error;
    Eigen::VectorXd delta = normal.ldlt().solve(rhs);
    if (!delta.allFinite()) {
      result.message = "IK linear solve produced non-finite values";
      result.joint_positions = positions;
      result.iterations = iteration;
      return result;
    }
    const double delta_norm = delta.norm();
    if (delta_norm > max_step) {
      delta *= max_step / delta_norm;
    }

    bool improved = false;
    double step = 1.0;
    for (int attempt = 0; attempt < 8; ++attempt) {
      JointPositions candidate = positions;
      for (std::size_t index = 0; index < joint_count; ++index) {
        candidate[index] = std::clamp(
            positions[index] + step * delta[static_cast<Eigen::Index>(index)],
            parameters_.joints[index].lower,
            parameters_.joints[index].upper);
      }
      Eigen::Isometry3d candidate_pose;
      const ValidationResult candidate_result =
          LinkPose(candidate, link_name, &candidate_pose);
      if (candidate_result.ok) {
        const Eigen::Quaterniond candidate_q(candidate_pose.linear());
        const double candidate_position =
            (target_position - candidate_pose.translation()).norm();
        const double candidate_orientation_error =
            OrientationAngle(target_orientation, candidate_q);
        const double current_cost =
            position_error.squaredNorm() +
            orientation_weight * orientation_weight *
                orientation_angle * orientation_angle;
        const double candidate_cost =
            candidate_position * candidate_position +
            orientation_weight * orientation_weight *
                candidate_orientation_error * candidate_orientation_error;
        if (candidate_cost < current_cost) {
          positions = std::move(candidate);
          improved = true;
          break;
        }
      }
      step *= 0.5;
    }
    if (!improved) {
      result.message = "IK stalled";
      result.joint_positions = positions;
      result.iterations = iteration;
      return result;
    }
  }

  result.message = "IK iteration limit reached";
  result.joint_positions = positions;
  result.iterations = max_iterations;
  return result;
}

}  // namespace planning
}  // namespace tramcar
