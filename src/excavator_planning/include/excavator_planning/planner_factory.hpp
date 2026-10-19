#ifndef EXCAVATOR_PLANNING__PLANNER_FACTORY_HPP_
#define EXCAVATOR_PLANNING__PLANNER_FACTORY_HPP_

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "excavator_common/planner_interfaces.hpp"
#include "excavator_planning/planner_plugin.hpp"

namespace tramcar {
namespace planning {

// Lightweight in-process registry used by unit tests. Runtime planner
// selection uses the pluginlib loader in planning_server so algorithms can
// live in independent packages.
class PlannerFactory {
 public:
  using Creator =
      std::function<std::unique_ptr<PathPlanner>(const PlannerSettings&)>;

  void Register(const std::string& name, Creator creator);
  bool Contains(const std::string& name) const;
  std::unique_ptr<PathPlanner> Create(
      const std::string& name, const PlannerSettings& settings) const;
  std::vector<std::string> Names() const;

  static PlannerFactory CreateDefault();

 private:
  std::map<std::string, Creator> creators_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_PLANNING__PLANNER_FACTORY_HPP_
