#include "excavator_kinematics/backend/kdl/kdl_solver_bundle.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "chainiksolverpos_nr_jl.hpp"
#include "chainiksolvervel_wdls.hpp"

namespace tramcar::planning {

KdlSolverBundle::KdlSolverBundle(KdlChainDescription description)
    : chain_(std::move(description.chain)),
      joint_indices_(std::move(description.joint_indices)),
      joint_limits_(std::move(description.joint_limits)) {}

KinematicsResult KdlSolverBundle::Initialize() {
  if (chain_.getNrOfJoints() != joint_indices_.size() ||
      joint_indices_.size() != joint_limits_.size()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "KDL Chain and joint index/limit mappings have different sizes");
  }
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
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
                             const KDL::Frame& target,
                             const IkOptions& options, bool enforce_limits,
                             KDL::JntArray* solution) {
  if (solution == nullptr || !fk_solver_ || chain_.getNrOfJoints() == 0 ||
      options.max_iterations == 0 ||
      !std::isfinite(options.position_tolerance) ||
      !std::isfinite(options.orientation_tolerance) ||
      options.position_tolerance <= 0.0 ||
      options.orientation_tolerance <= 0.0) {
    last_error_message_ = "IK inputs or solver state are invalid";
    return KDL::SolverI::E_NOT_UP_TO_DATE;
  }

  KDL::JntArray lower(chain_.getNrOfJoints());
  KDL::JntArray upper(chain_.getNrOfJoints());
  constexpr double kWideLimit = 1.0e9;
  for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
    const JointLimit& limit = joint_limits_[i];
    lower(i) = enforce_limits && limit.has_position_limits
                   ? limit.lower
                   : -kWideLimit;
    upper(i) = enforce_limits && limit.has_position_limits
                   ? limit.upper
                   : kWideLimit;
  }

  KDL::ChainIkSolverVel_wdls velocity_solver(chain_);
  const double epsilon =
      std::min(options.position_tolerance, options.orientation_tolerance);
  KDL::ChainIkSolverPos_NR_JL position_solver(
      chain_, lower, upper, *fk_solver_, velocity_solver,
      options.max_iterations, epsilon);
  const int code = position_solver.CartToJnt(seed, target, *solution);
  last_error_message_ =
      code == KDL::SolverI::E_NOERROR ? std::string()
                                      : position_solver.strError(code);
  return code;
}

}  // namespace tramcar::planning
