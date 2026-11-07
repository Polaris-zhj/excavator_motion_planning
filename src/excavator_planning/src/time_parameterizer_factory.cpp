#include "excavator_planning/time_parameterizer_factory.hpp"

#include "excavator_planning/baseline_planners.hpp"
#include "excavator_planning/toppra_time_parameterizer.hpp"

namespace tramcar {
namespace planning {

std::unique_ptr<TimeParameterizer> CreateTimeParameterizer(
    const std::string& name,
    const TimeParameterizerOptions& options) {
  if (name == "simple" || name == "SimpleTimeParameterizer" ||
      name == "excavator_planning/SimpleTimeParameterizer") {
    return std::make_unique<SimpleTimeParameterizer>();
  }
  if (name == "toppra" || name == "ToppraTimeParameterizer" ||
      name == "excavator_planning/ToppraTimeParameterizer") {
    return std::make_unique<ToppraTimeParameterizer>(
        options.toppra_grid_points);
  }
  return nullptr;
}

std::vector<std::string> TimeParameterizerNames() {
  return {"simple", "toppra"};
}

}  // namespace planning
}  // namespace tramcar
