#include <memory>

#include "excavator_planning/baseline_planners.hpp"
#include "excavator_planning/planner_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace tramcar {
namespace planning {

class LinearPlannerPlugin final : public PlannerPlugin {
 public:
  void Configure(const PlannerSettings& settings) override {
    planner_ = std::make_unique<LinearPathPlanner>(
        settings.joint_names, settings.joint_limits, settings.max_joint_step,
        settings.max_detour_attempts);
  }

  PlanningResponse Plan(const PlanningRequest& request) override {
    if (!planner_) {
      PlanningResponse response;
      response.message = "LinearPlannerPlugin is not configured";
      return response;
    }
    return planner_->Plan(request);
  }

 private:
  std::unique_ptr<LinearPathPlanner> planner_;
};

}  // namespace planning
}  // namespace tramcar

PLUGINLIB_EXPORT_CLASS(tramcar::planning::LinearPlannerPlugin,
                       tramcar::planning::PlannerPlugin)
