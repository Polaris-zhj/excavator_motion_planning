#ifndef EXCAVATOR_KINEMATICS__KDL_SOLVER_BUNDLE_HPP_
#define EXCAVATOR_KINEMATICS__KDL_SOLVER_BUNDLE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "excavator_kinematics/backend/kdl/kdl_model_converter.hpp"
#include "excavator_kinematics/kinematics_types.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainjnttojacsolver.hpp"

namespace tramcar::planning {

class KdlSolverBundle {
 public:
  explicit KdlSolverBundle(KdlChainDescription description);
  KinematicsResult Initialize();
  int ComputeFk(const KDL::JntArray& q, KDL::Frame* frame);
  int ComputeJacobian(const KDL::JntArray& q, KDL::Jacobian* jacobian);
  int SolveIk(const KDL::JntArray& seed, const KDL::Frame& target,
              const IkOptions& options, bool enforce_limits,
              KDL::JntArray* solution);
  const std::vector<std::size_t>& joint_indices() const {
    return joint_indices_;
  }
  const std::vector<JointLimit>& joint_limits() const {
    return joint_limits_;
  }
  const std::string& last_error_message() const { return last_error_message_; }

 private:
  KDL::Chain chain_;
  std::vector<std::size_t> joint_indices_;
  std::vector<JointLimit> joint_limits_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::string last_error_message_;
};

}  // namespace tramcar::planning

#endif  // EXCAVATOR_KINEMATICS__KDL_SOLVER_BUNDLE_HPP_
