#include "excavator_kinematics/kinematics_solver_factory.hpp"

#include <algorithm>
#include <utility>

#include "excavator_kinematics/backend/kdl/kdl_kinematics_solver.hpp"

namespace excavator {
namespace planning {

KinematicsSolverFactory& KinematicsSolverFactory::Instance() {
  static KinematicsSolverFactory factory;
  return factory;
}

KinematicsSolverFactory::KinematicsSolverFactory() {
  Register("kdl", [](const KinematicsSolverConfig& config) {
    return std::make_unique<KdlKinematicsSolver>(config);
  });
}

void KinematicsSolverFactory::Register(const std::string& backend,
                                       Creator creator) {
  if (backend.empty() || !creator) return;
  std::lock_guard<std::mutex> lock(mutex_);
  creators_[backend] = std::move(creator);
}

std::unique_ptr<KinematicsSolver> KinematicsSolverFactory::Create(
    const KinematicsSolverConfig& config) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = creators_.find(config.backend);
  if (it == creators_.end()) return nullptr;
  return it->second(config);
}

std::vector<std::string> KinematicsSolverFactory::AvailableBackends() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> backends;
  backends.reserve(creators_.size());
  for (const auto& entry : creators_) {
    backends.push_back(entry.first);
  }
  std::sort(backends.begin(), backends.end());
  return backends;
}

}  // namespace planning
}  // namespace excavator
