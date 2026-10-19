#include "excavator_planning/planner_factory.hpp"

#include <algorithm>

#include "excavator_planning/baseline_planners.hpp"

namespace tramcar {
namespace planning {

void PlannerFactory::Register(const std::string& name, Creator creator) {
  if (name.empty() || !creator) {
    return;
  }
  creators_[name] = std::move(creator);
}

bool PlannerFactory::Contains(const std::string& name) const {
  return creators_.find(name) != creators_.end();
}

std::unique_ptr<PathPlanner> PlannerFactory::Create(
    const std::string& name, const PlannerSettings& settings) const {
  const auto found = creators_.find(name);
  if (found == creators_.end()) {
    return nullptr;
  }
  return found->second(settings);
}

std::vector<std::string> PlannerFactory::Names() const {
  std::vector<std::string> names;
  names.reserve(creators_.size());
  for (const auto& entry : creators_) {
    names.push_back(entry.first);
  }
  return names;
}

PlannerFactory PlannerFactory::CreateDefault() {
  PlannerFactory factory;
  factory.Register(
      "linear", [](const PlannerSettings& settings) {
        return std::make_unique<LinearPathPlanner>(
            settings.joint_names, settings.joint_limits,
            settings.max_joint_step, settings.max_detour_attempts);
      });
  return factory;
}

}  // namespace planning
}  // namespace tramcar
