#ifndef EXCAVATOR_PLANNER_RRTSTAR__RRT_STAR_PLANNER_HPP_
#define EXCAVATOR_PLANNER_RRTSTAR__RRT_STAR_PLANNER_HPP_

#include <cstddef>

#include "excavator_planning/planner_plugin.hpp"

namespace tramcar {
namespace planning {

// Sampling-based asymptotically optimal planner. It is intentionally kept in
// a separate package so it can be selected, replaced or removed without
// touching the planning pipeline, controller or visualization.
class RrtStarPlanner final : public PlannerPlugin {
 public:
  void Configure(const PlannerSettings& settings) override;
  PlanningResponse Plan(const PlanningRequest& request) override;

 private:
  struct Node {
    JointPositions positions;
    std::size_t parent{0};
    double cost{0.0};
  };

  PlannerSettings settings_;
  bool configured_{false};
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_PLANNER_RRTSTAR__RRT_STAR_PLANNER_HPP_
