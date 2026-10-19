#ifndef EXCAVATOR_PLANNING__PLANNER_PLUGIN_HPP_
#define EXCAVATOR_PLANNING__PLANNER_PLUGIN_HPP_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "excavator_common/planner_interfaces.hpp"

namespace tramcar {
namespace planning {

// Configuration passed to every planner plugin. Sampling planners use the
// RRT* fields; the built-in linear planner ignores them.
struct PlannerSettings {
  std::vector<std::string> joint_names;
  std::vector<std::pair<double, double>> joint_limits;
  double max_joint_step{0.05};
  std::size_t max_detour_attempts{12};

  // Sampling-based planner settings.
  std::size_t max_iterations{2000};
  double step_size{0.25};
  double goal_bias{0.10};
  double rewire_radius{0.60};
  double goal_tolerance{0.15};
  unsigned int random_seed{42};
  // Joint-space resolution used when validating edges for collision.
  double collision_check_resolution{0.03};
};

// Base class exported through pluginlib. A new algorithm only needs to:
//   1. derive from PlannerPlugin in its own package,
//   2. export a plugin description with category "excavator_planning",
//   3. set the planner_class launch parameter to its class name.
// The planning pipeline, controller and visualization do not need changes.
class PlannerPlugin : public PathPlanner {
 public:
  ~PlannerPlugin() override = default;

  virtual void Configure(const PlannerSettings& settings) = 0;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_PLANNING__PLANNER_PLUGIN_HPP_
