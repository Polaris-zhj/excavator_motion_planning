#ifndef EXCAVATOR_KINEMATICS__KINEMATICS_SOLVER_FACTORY_HPP_
#define EXCAVATOR_KINEMATICS__KINEMATICS_SOLVER_FACTORY_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "excavator_kinematics/kinematics_solver.hpp"

namespace excavator {
namespace planning {

class KinematicsSolverFactory {
 public:
  using Creator = std::function<std::unique_ptr<KinematicsSolver>(
      const KinematicsSolverConfig&)>;

  static KinematicsSolverFactory& Instance();
  void Register(const std::string& backend, Creator creator);
  std::unique_ptr<KinematicsSolver> Create(
      const KinematicsSolverConfig& config = {}) const;
  std::vector<std::string> AvailableBackends() const;

 private:
  KinematicsSolverFactory();
  mutable std::mutex mutex_;
  std::unordered_map<std::string, Creator> creators_;
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_KINEMATICS__KINEMATICS_SOLVER_FACTORY_HPP_
