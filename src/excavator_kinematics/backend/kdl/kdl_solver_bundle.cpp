#include "excavator_kinematics/backend/kdl/kdl_solver_bundle.hpp"

#include <algorithm>
#include <utility>

namespace tramcar::planning {

KdlSolverBundle::KdlSolverBundle(
    KdlChainDescription description,
    const KinematicsSolverConfig& config)
    : chain_(std::move(description.chain)),
      joint_indices_(std::move(description.joint_indices)),
      joint_limits_(std::move(description.joint_limits)),
      config_(config) {}

KinematicsResult KdlSolverBundle::Initialize() {
  if (chain_.getNrOfJoints() != joint_indices_.size() ||
      joint_indices_.size() != joint_limits_.size()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "KDL Chain and joint index/limit mappings have different sizes");
  }
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  if (chain_.getNrOfJoints() == 0) return KinematicsResult::Success();

  ik_velocity_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(
      chain_, 1.0e-5, static_cast<int>(config_.max_iterations));

  KDL::JntArray lower(chain_.getNrOfJoints());
  KDL::JntArray upper(chain_.getNrOfJoints());
  KDL::JntArray unbounded_lower(chain_.getNrOfJoints());
  KDL::JntArray unbounded_upper(chain_.getNrOfJoints());
  constexpr double kWideLimit = 1.0e9;
  for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
    const JointLimit& limit = joint_limits_[i];
    lower(i) = limit.has_position_limits ? limit.lower : -kWideLimit;
    upper(i) = limit.has_position_limits ? limit.upper : kWideLimit;
    unbounded_lower(i) = -kWideLimit;
    unbounded_upper(i) = kWideLimit;
  }

  const unsigned int iterations = std::max(1U, config_.max_iterations);
  const double epsilon =
      std::min(config_.position_tolerance, config_.orientation_tolerance);
  ik_position_limited_solver_ =
      std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
          chain_, lower, upper, *fk_solver_, *ik_velocity_solver_, iterations,
          epsilon);
  ik_position_unbounded_solver_ =
      std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
          chain_, unbounded_lower, unbounded_upper, *fk_solver_,
          *ik_velocity_solver_, iterations, epsilon);
  return KinematicsResult::Success();
}

int KdlSolverBundle::ComputeFk(const KDL::JntArray& q, KDL::Frame* frame) {
  if (!fk_solver_ || frame == nullptr) {
    last_error_message_ = "FK solver is not initialized or output is null";
    return KDL::SolverI::E_NOT_UP_TO_DATE;
  }
  const int code = fk_solver_->JntToCart(q, *frame);
  last_error_message_ = code == KDL::SolverI::E_NOERROR
                            ? std::string()
                            : fk_solver_->strError(code);
  return code;
}

int KdlSolverBundle::ComputeJacobian(const KDL::JntArray& q,
                                     KDL::Jacobian* jacobian) {
  if (!jacobian_solver_ || jacobian == nullptr) {
    last_error_message_ = "Jacobian solver is not initialized or output is null";
    return KDL::SolverI::E_NOT_UP_TO_DATE;
  }
  const int code = jacobian_solver_->JntToJac(q, *jacobian);
  last_error_message_ = code == KDL::SolverI::E_NOERROR
                            ? std::string()
                            : jacobian_solver_->strError(code);
  return code;
}

int KdlSolverBundle::SolveIk(const KDL::JntArray& seed,
                             const KDL::Frame& target, bool enforce_limits,
                             KDL::JntArray* solution) {
  if (solution == nullptr || chain_.getNrOfJoints() == 0 ||
      !ik_velocity_solver_) {
    last_error_message_ = "IK requires a non-empty Chain and initialized solver";
    return KDL::SolverI::E_NOT_UP_TO_DATE;
  }
  KDL::ChainIkSolverPos_NR_JL* solver =
      enforce_limits ? ik_position_limited_solver_.get()
                     : ik_position_unbounded_solver_.get();
  if (solver == nullptr) {
    last_error_message_ = "position IK solver is not initialized";
    return KDL::SolverI::E_NOT_UP_TO_DATE;
  }
  const int code = solver->CartToJnt(seed, target, *solution);
  last_error_message_ =
      code == KDL::SolverI::E_NOERROR ? std::string() : solver->strError(code);
  return code;
}

}  // namespace tramcar::planning
