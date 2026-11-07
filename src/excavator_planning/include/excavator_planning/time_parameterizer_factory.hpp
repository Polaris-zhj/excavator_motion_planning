#ifndef EXCAVATOR_PLANNING__TIME_PARAMETERIZER_FACTORY_HPP_
#define EXCAVATOR_PLANNING__TIME_PARAMETERIZER_FACTORY_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "excavator_common/planner_interfaces.hpp"

namespace tramcar {
namespace planning {

struct TimeParameterizerOptions {
  std::size_t toppra_grid_points{200};
};

// Creates a time parameterizer by name. Supported names are "simple" (the
// conservative baseline) and "toppra" (time-optimal path parameterization).
// The fully qualified spellings "excavator_planning/SimpleTimeParameterizer"
// and "excavator_planning/ToppraTimeParameterizer" are accepted as well.
// Returns nullptr for an unknown name.
std::unique_ptr<TimeParameterizer> CreateTimeParameterizer(
    const std::string& name,
    const TimeParameterizerOptions& options = TimeParameterizerOptions{});

std::vector<std::string> TimeParameterizerNames();

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_PLANNING__TIME_PARAMETERIZER_FACTORY_HPP_
