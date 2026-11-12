#include "excavator_kinematics/backend/kdl/kdl_kinematics_solver.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "chainiksolverpos_nr_jl.hpp"
#include "chainiksolvervel_wdls.hpp"

#include "excavator_kinematics/backend/kdl/kdl_type_converter.hpp"

namespace excavator {
namespace planning {
namespace {

bool IsFinite(const JointVector& q) {
  return q.array().isFinite().all();
}

bool IsFinite(const Pose& pose) {
  return pose.position.array().isFinite().all() &&
         pose.orientation.coeffs().array().isFinite().all() &&
         pose.orientation.norm() > 1.0e-12;
}

Eigen::Isometry3d ToIsometry(const Pose& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = pose.position;
  transform.linear() = pose.orientation.normalized().toRotationMatrix();
  return transform;
}

Pose FromIsometry(const Eigen::Isometry3d& transform) {
  Pose pose;
  pose.position = transform.translation();
  pose.orientation = Eigen::Quaterniond(transform.linear()).normalized();
  return pose;
}

double OrientationError(const Eigen::Quaterniond& target,
                        const Eigen::Quaterniond& actual) {
  const double dot = std::clamp(
      std::abs(target.normalized().dot(actual.normalized())), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

}  // namespace

KdlKinematicsSolver::KdlKinematicsSolver(KinematicsSolverConfig config)
    : config_(std::move(config)) {}

KinematicsResult KdlKinematicsSolver::Initialize(
    std::shared_ptr<const KinematicsModel> model) {
  if (!model || !model->initialized()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "an initialized KinematicsModel is required");
  }
  if (config_.ik_solver != "nr_jl") {
    return KinematicsResult::Failure(
        KinematicsStatus::kUnsupported,
        "KDL backend currently supports ik_solver: nr_jl only");
  }
  if (config_.max_iterations == 0 ||
      !std::isfinite(config_.position_tolerance) ||
      !std::isfinite(config_.orientation_tolerance) ||
      config_.position_tolerance <= 0.0 ||
      config_.orientation_tolerance <= 0.0) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "IK iterations and tolerances must be positive and finite");
  }

  KdlChainDescription description;
  KinematicsResult result =
      model_converter_.BuildFullChain(*model, &description);
  if (!result.ok()) return result;

  std::lock_guard<std::mutex> lock(mutex_);
  // KDL solvers keep references to their Chain, so the member Chain must be
  // fully assigned before constructing either solver.
  fk_solver_.reset();
  jacobian_solver_.reset();
  model_ = std::move(model);
  chain_ = std::move(description.chain);
  joint_indices_ = std::move(description.joint_indices);
  joint_limits_ = std::move(description.joint_limits);
  link_segment_indices_ = std::move(description.link_segment_indices);
  fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  last_error_message_.clear();
  return KinematicsResult::Success();
}

KinematicsResult KdlKinematicsSolver::ValidateJointVector(
    const JointVector& q) const {
  if (!model_ || !fk_solver_ || !jacobian_solver_) {
    return KinematicsResult::Failure(KinematicsStatus::kModelNotReady,
                                     "KDL solver has not been initialized");
  }
  if (q.size() !=
      static_cast<Eigen::Index>(model_->GetActiveJointNames().size())) {
    return KinematicsResult::Failure(
        KinematicsStatus::kDimensionMismatch,
        "joint vector dimension does not match model joint_order");
  }
  if (!IsFinite(q)) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "joint vector must contain finite values");
  }
  return KinematicsResult::Success();
}

KinematicsResult KdlKinematicsSolver::MapJointVector(
    const JointVector& q, KDL::JntArray* q_kdl) const {
  if (q_kdl == nullptr ||
      !KdlTypeConverter::ToKdlJntArray(q, joint_indices_, q_kdl)) {
    return KinematicsResult::Failure(
        KinematicsStatus::kDimensionMismatch,
        "failed to map project joint vector to full KDL Chain order");
  }
  return KinematicsResult::Success();
}

bool KdlKinematicsSolver::GetSegmentIndex(
    const std::string& link_name, unsigned int* segment_index) const {
  if (segment_index == nullptr) return false;
  const auto found = link_segment_indices_.find(link_name);
  if (found == link_segment_indices_.end()) return false;
  *segment_index = found->second;
  return true;
}

KinematicsResult KdlKinematicsSolver::GetLinkPoseLocked(
    const JointVector& q, const std::string& link_name, Pose* pose) const {
  if (pose == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "pose output pointer is null");
  }
  if (model_->GetLink(link_name) == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kUnknownLink,
                                     "unknown target Link: " + link_name);
  }
  unsigned int segment_index = 0;
  if (!GetSegmentIndex(link_name, &segment_index)) {
    return KinematicsResult::Failure(
        KinematicsStatus::kUnsupported,
        "Link is outside the selected single KDL Chain: " + link_name);
  }
  if (segment_index == 0U) {
    *pose = Pose{};
    return KinematicsResult::Success();
  }

  KDL::JntArray q_kdl;
  KinematicsResult result = MapJointVector(q, &q_kdl);
  if (!result.ok()) return result;

  KDL::Frame frame;
  const int code =
      fk_solver_->JntToCart(q_kdl, frame, static_cast<int>(segment_index));
  if (code != KDL::SolverI::E_NOERROR) {
    last_error_message_ = fk_solver_->strError(code);
    return KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "KDL FK failed: " + last_error_message_);
  }
  *pose = KdlTypeConverter::FromKdlFrame(frame);
  return KinematicsResult::Success();
}

KinematicsResult KdlKinematicsSolver::GetLinkPose(
    const JointVector& q, const std::string& link_name, Pose* pose) const {
  std::lock_guard<std::mutex> lock(mutex_);
  KinematicsResult result = ValidateJointVector(q);
  if (!result.ok()) return result;
  return GetLinkPoseLocked(q, link_name, pose);
}

KinematicsResult KdlKinematicsSolver::GetLinkPoses(
    const JointVector& q, const std::vector<std::string>& link_names,
    std::vector<Pose>* poses) const {
  if (poses == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "poses output pointer is null");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  KinematicsResult result = ValidateJointVector(q);
  if (!result.ok()) return result;

  std::vector<Pose> output;
  output.reserve(link_names.size());
  for (const std::string& link_name : link_names) {
    Pose pose;
    result = GetLinkPoseLocked(q, link_name, &pose);
    if (!result.ok()) return result;
    output.push_back(std::move(pose));
  }
  *poses = std::move(output);
  return KinematicsResult::Success();
}

KinematicsResult KdlKinematicsSolver::GetRelativePose(
    const JointVector& q, const std::string& reference_link,
    const std::string& target_link, Pose* pose) const {
  if (pose == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "pose output pointer is null");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  KinematicsResult result = ValidateJointVector(q);
  if (!result.ok()) return result;

  Pose reference_pose;
  Pose target_pose;
  result = GetLinkPoseLocked(q, reference_link, &reference_pose);
  if (!result.ok()) return result;
  result = GetLinkPoseLocked(q, target_link, &target_pose);
  if (!result.ok()) return result;
  *pose = FromIsometry(ToIsometry(reference_pose).inverse() *
                       ToIsometry(target_pose));
  return KinematicsResult::Success();
}

KinematicsResult KdlKinematicsSolver::GetJacobian(
    const JointVector& q, const std::string& link_name,
    JacobianMatrix* jacobian) const {
  if (jacobian == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "Jacobian output pointer is null");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  KinematicsResult result = ValidateJointVector(q);
  if (!result.ok()) return result;
  if (model_->GetLink(link_name) == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kUnknownLink,
                                     "unknown target Link: " + link_name);
  }

  const Eigen::Index joint_count =
      static_cast<Eigen::Index>(model_->GetActiveJointNames().size());
  JacobianMatrix output = JacobianMatrix::Zero(6, joint_count);
  unsigned int segment_index = 0;
  if (!GetSegmentIndex(link_name, &segment_index)) {
    return KinematicsResult::Failure(
        KinematicsStatus::kUnsupported,
        "Link is outside the selected single KDL Chain: " + link_name);
  }
  if (segment_index == 0U || chain_.getNrOfJoints() == 0U) {
    *jacobian = std::move(output);
    return KinematicsResult::Success();
  }

  KDL::JntArray q_kdl;
  result = MapJointVector(q, &q_kdl);
  if (!result.ok()) return result;
  KDL::Jacobian kdl_jacobian(chain_.getNrOfJoints());
  const int code = jacobian_solver_->JntToJac(
      q_kdl, kdl_jacobian, static_cast<int>(segment_index));
  if (code != KDL::SolverI::E_NOERROR) {
    last_error_message_ = jacobian_solver_->strError(code);
    return KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "KDL Jacobian failed: " + last_error_message_);
  }

  const JacobianMatrix chain_jacobian =
      KdlTypeConverter::FromKdlJacobian(kdl_jacobian);
  for (std::size_t column = 0; column < joint_indices_.size(); ++column) {
    output.col(static_cast<Eigen::Index>(joint_indices_[column])) =
        chain_jacobian.col(static_cast<Eigen::Index>(column));
  }
  *jacobian = std::move(output);
  return KinematicsResult::Success();
}

IkResult KdlKinematicsSolver::InverseKinematics(
    const IkRequest& request) const {
  IkResult output;
  std::lock_guard<std::mutex> lock(mutex_);
  IkOptions options = request.options;
  const IkOptions default_options;
  if (options.max_iterations == default_options.max_iterations) {
    options.max_iterations = config_.max_iterations;
  }
  if (options.position_tolerance == default_options.position_tolerance) {
    options.position_tolerance = config_.position_tolerance;
  }
  if (options.orientation_tolerance == default_options.orientation_tolerance) {
    options.orientation_tolerance = config_.orientation_tolerance;
  }

  output.result = ValidateJointVector(request.initial_q);
  if (!output.result.ok()) return output;
  if (request.target_link.empty() || !IsFinite(request.target_pose) ||
      options.max_iterations == 0 ||
      !std::isfinite(options.position_tolerance) ||
      !std::isfinite(options.orientation_tolerance) ||
      options.position_tolerance <= 0.0 ||
      options.orientation_tolerance <= 0.0) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "IK target, pose, iteration count and tolerances must be valid");
    return output;
  }
  if (model_->GetLink(request.target_link) == nullptr) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kUnknownLink,
        "unknown IK target Link: " + request.target_link);
    return output;
  }

  unsigned int target_segment = 0;
  if (!GetSegmentIndex(request.target_link, &target_segment)) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kUnsupported,
        "IK target is outside the selected single KDL Chain: " +
            request.target_link);
    return output;
  }

  if (options.enforce_joint_limits) {
    const auto& joint_names = model_->GetActiveJointNames();
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const JointModel* joint = model_->GetJoint(joint_names[i]);
      const double value = request.initial_q(static_cast<Eigen::Index>(i));
      if (joint != nullptr && joint->limit.has_position_limits &&
          (value < joint->limit.lower || value > joint->limit.upper)) {
        output.result = KinematicsResult::Failure(
            KinematicsStatus::kJointLimitViolation,
            "IK initial configuration violates a joint position limit");
        return output;
      }
    }
  }

  if (target_segment == 0U) {
    const double position_error = request.target_pose.position.norm();
    const double orientation_error = OrientationError(
        Eigen::Quaterniond::Identity(), request.target_pose.orientation);
    if (position_error > options.position_tolerance ||
        orientation_error > options.orientation_tolerance) {
      output.result = KinematicsResult::Failure(
          KinematicsStatus::kNoConvergence,
          "root_link is fixed and cannot reach a non-identity target pose");
      return output;
    }
    output.solution = request.initial_q;
    output.result = KinematicsResult::Success();
    return output;
  }

  KDL::Chain prefix_chain;
  for (unsigned int i = 0; i < target_segment; ++i) {
    prefix_chain.addSegment(chain_.getSegment(i));
  }
  const std::size_t prefix_joint_count = prefix_chain.getNrOfJoints();
  if (prefix_joint_count == 0U) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kNoConvergence,
        "target Link has no active joints relative to root_link");
    return output;
  }

  std::vector<std::size_t> prefix_indices;
  std::vector<JointLimit> prefix_limits;
  prefix_indices.reserve(prefix_joint_count);
  prefix_limits.reserve(prefix_joint_count);
  for (std::size_t i = 0; i < prefix_joint_count; ++i) {
    prefix_indices.push_back(joint_indices_[i]);
    prefix_limits.push_back(joint_limits_[i]);
  }

  KDL::JntArray seed;
  if (!KdlTypeConverter::ToKdlJntArray(request.initial_q,
                                       prefix_indices, &seed)) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kDimensionMismatch,
        "failed to map IK seed to target prefix Chain order");
    return output;
  }

  KDL::JntArray lower(static_cast<unsigned int>(prefix_joint_count));
  KDL::JntArray upper(static_cast<unsigned int>(prefix_joint_count));
  constexpr double kWideLimit = 1.0e9;
  for (unsigned int i = 0; i < prefix_joint_count; ++i) {
    const JointLimit& limit = prefix_limits[i];
    lower(i) = options.enforce_joint_limits && limit.has_position_limits
                   ? limit.lower
                   : -kWideLimit;
    upper(i) = options.enforce_joint_limits && limit.has_position_limits
                   ? limit.upper
                   : kWideLimit;
  }

  KDL::ChainFkSolverPos_recursive prefix_fk_solver(prefix_chain);
  KDL::ChainIkSolverVel_wdls velocity_solver(prefix_chain);
  const double epsilon =
      std::min(options.position_tolerance, options.orientation_tolerance);
  KDL::ChainIkSolverPos_NR_JL position_solver(
      prefix_chain, lower, upper, prefix_fk_solver, velocity_solver,
      options.max_iterations, epsilon);
  KDL::JntArray prefix_solution(static_cast<unsigned int>(prefix_joint_count));
  const KDL::Frame target =
      KdlTypeConverter::ToKdlFrame(request.target_pose);
  const int code = position_solver.CartToJnt(seed, target, prefix_solution);
  if (code != KDL::SolverI::E_NOERROR) {
    last_error_message_ = position_solver.strError(code);
    output.result = KinematicsResult::Failure(
        code == KDL::ChainIkSolverVel_wdls::E_CONVERGE_PINV_SINGULAR
            ? KinematicsStatus::kSingular
            : KinematicsStatus::kNoConvergence,
        "KDL position IK failed: " + last_error_message_);
    return output;
  }

  output.solution = request.initial_q;
  if (!KdlTypeConverter::FromKdlJntArray(
          prefix_solution, prefix_indices, &output.solution)) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "failed to map KDL IK solution to project joint order");
    return output;
  }

  if (options.enforce_joint_limits) {
    const auto& joint_names = model_->GetActiveJointNames();
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const JointModel* joint = model_->GetJoint(joint_names[i]);
      const double value = output.solution(static_cast<Eigen::Index>(i));
      if (joint != nullptr && joint->limit.has_position_limits &&
          (value < joint->limit.lower - 1.0e-9 ||
           value > joint->limit.upper + 1.0e-9)) {
        output.result = KinematicsResult::Failure(
            KinematicsStatus::kJointLimitViolation,
            "IK solution violates a joint position limit");
        return output;
      }
    }
  }

  Pose achieved_pose;
  output.result = GetLinkPoseLocked(output.solution, request.target_link,
                                    &achieved_pose);
  if (!output.result.ok()) return output;
  output.position_error =
      (achieved_pose.position - request.target_pose.position).norm();
  output.orientation_error = OrientationError(
      request.target_pose.orientation, achieved_pose.orientation);
  if (output.position_error > options.position_tolerance ||
      output.orientation_error > options.orientation_tolerance) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kNoConvergence,
        "IK candidate failed FK pose verification");
    return output;
  }
  output.result = KinematicsResult::Success();
  return output;
}

}  // namespace planning
}  // namespace excavator
