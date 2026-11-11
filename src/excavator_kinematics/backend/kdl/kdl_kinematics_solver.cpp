#include "excavator_kinematics/backend/kdl/kdl_kinematics_solver.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include "chainiksolvervel_wdls.hpp"

#include "excavator_kinematics/backend/kdl/kdl_type_converter.hpp"

namespace tramcar::planning {
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
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
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
  std::lock_guard<std::mutex> lock(mutex_);
  model_ = std::move(model);
  bundles_.clear();
  return KinematicsResult::Success();
}

KinematicsResult KdlKinematicsSolver::ValidateJointVector(
    const JointVector& q) const {
  if (!model_) {
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

std::string KdlKinematicsSolver::MakeCacheKey(
    const std::string& base_link, const std::string& tip_link) const {
  return base_link + "->" + tip_link;
}

KinematicsResult KdlKinematicsSolver::GetOrCreateBundleLocked(
    const std::string& base_link, const std::string& tip_link,
    KdlSolverBundle** bundle) const {
  if (bundle == nullptr || !model_) {
    return KinematicsResult::Failure(KinematicsStatus::kModelNotReady,
                                     "solver is not initialized");
  }
  const std::string key = MakeCacheKey(base_link, tip_link);
  const auto found = bundles_.find(key);
  if (found != bundles_.end()) {
    *bundle = found->second.get();
    return KinematicsResult::Success();
  }
  KdlChainDescription description;
  KinematicsResult result =
      model_converter_.BuildChain(*model_, base_link, tip_link, &description);
  if (!result.ok()) return result;

  auto created = std::make_unique<KdlSolverBundle>(std::move(description));
  result = created->Initialize();
  if (!result.ok()) return result;
  KdlSolverBundle* raw = created.get();
  bundles_.emplace(key, std::move(created));
  *bundle = raw;
  return KinematicsResult::Success();
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
  if (link_name == model_->root_link()) {
    *pose = Pose{};
    return KinematicsResult::Success();
  }

  KdlSolverBundle* bundle = nullptr;
  KinematicsResult result =
      GetOrCreateBundleLocked(model_->root_link(), link_name, &bundle);
  if (!result.ok()) return result;
  KDL::JntArray q_kdl;
  if (!KdlTypeConverter::ToKdlJntArray(q, bundle->joint_indices(), &q_kdl)) {
    return KinematicsResult::Failure(
        KinematicsStatus::kDimensionMismatch,
        "failed to map project joint vector to KDL Chain order");
  }
  KDL::Frame frame;
  const int code = bundle->ComputeFk(q_kdl, &frame);
  if (code != KDL::SolverI::E_NOERROR) {
    return KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "KDL FK failed: " + bundle->last_error_message());
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
  if (link_name == model_->root_link()) {
    *jacobian = std::move(output);
    return KinematicsResult::Success();
  }

  KdlSolverBundle* bundle = nullptr;
  result = GetOrCreateBundleLocked(model_->root_link(), link_name, &bundle);
  if (!result.ok()) return result;
  if (bundle->joint_indices().empty()) {
    *jacobian = std::move(output);
    return KinematicsResult::Success();
  }

  KDL::JntArray q_kdl;
  if (!KdlTypeConverter::ToKdlJntArray(q, bundle->joint_indices(), &q_kdl)) {
    return KinematicsResult::Failure(
        KinematicsStatus::kDimensionMismatch,
        "failed to map project joint vector to KDL Chain order");
  }
  KDL::Jacobian kdl_jacobian(
      static_cast<unsigned int>(bundle->joint_indices().size()));
  const int code = bundle->ComputeJacobian(q_kdl, &kdl_jacobian);
  if (code != KDL::SolverI::E_NOERROR) {
    return KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "KDL Jacobian failed: " + bundle->last_error_message());
  }
  const JacobianMatrix chain_jacobian =
      KdlTypeConverter::FromKdlJacobian(kdl_jacobian);
  for (std::size_t column = 0; column < bundle->joint_indices().size();
       ++column) {
    output.col(static_cast<Eigen::Index>(bundle->joint_indices()[column])) =
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
      request.options.position_tolerance <= 0.0 ||
      request.options.orientation_tolerance <= 0.0) {
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
  if (request.target_link == model_->root_link()) {
    const double position_error = request.target_pose.position.norm();
    const double orientation_error = OrientationError(
        Eigen::Quaterniond::Identity(), request.target_pose.orientation);
    if (position_error > request.options.position_tolerance ||
        orientation_error > request.options.orientation_tolerance) {
      output.result = KinematicsResult::Failure(
          KinematicsStatus::kNoConvergence,
          "root_link is fixed and cannot reach a non-identity target pose");
      return output;
    }
    output.solution = request.initial_q;
    output.result = KinematicsResult::Success();
    return output;
  }

  KdlSolverBundle* bundle = nullptr;
  output.result = GetOrCreateBundleLocked(model_->root_link(),
                                          request.target_link, &bundle);
  if (!output.result.ok()) return output;
  if (bundle->joint_indices().empty()) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kNoConvergence,
        "target Link has no movable joints relative to root_link");
    return output;
  }

  KDL::JntArray seed;
  if (!KdlTypeConverter::ToKdlJntArray(request.initial_q,
                                       bundle->joint_indices(), &seed)) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kDimensionMismatch,
        "failed to map IK seed to KDL Chain order");
    return output;
  }
  const KDL::Frame target = KdlTypeConverter::ToKdlFrame(request.target_pose);
  KDL::JntArray solution(seed.rows());
  const int code = bundle->SolveIk(seed, target, request.options,
                                   options.enforce_joint_limits,
                                   &solution);
  if (code != KDL::SolverI::E_NOERROR) {
    output.result = KinematicsResult::Failure(
        code == KDL::ChainIkSolverVel_wdls::E_CONVERGE_PINV_SINGULAR
            ? KinematicsStatus::kSingular
            : KinematicsStatus::kNoConvergence,
        "KDL position IK failed: " + bundle->last_error_message());
    return output;
  }

  output.solution = request.initial_q;
  if (!KdlTypeConverter::FromKdlJntArray(
          solution, bundle->joint_indices(), &output.solution)) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "failed to map KDL IK solution to project joint order");
    return output;
  }

  if (options.enforce_joint_limits) {
    for (std::size_t i = 0; i < bundle->joint_indices().size(); ++i) {
      const JointLimit& limit = bundle->joint_limits()[i];
      const double value = solution(static_cast<unsigned int>(i));
      if (limit.has_position_limits &&
          (value < limit.lower - 1.0e-9 || value > limit.upper + 1.0e-9)) {
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
  if (output.position_error > request.options.position_tolerance ||
      output.orientation_error > request.options.orientation_tolerance) {
    output.result = KinematicsResult::Failure(
        KinematicsStatus::kNoConvergence,
        "IK candidate failed FK pose verification");
    return output;
  }
  output.result = KinematicsResult::Success();
  return output;
}

}  // namespace tramcar::planning
