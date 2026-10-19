#include "excavator_planning/path_post_processor.hpp"

#include <algorithm>
#include <utility>

namespace tramcar {
namespace planning {

PathPostProcessor::PathPostProcessor(std::size_t max_iterations)
    : max_iterations_(std::max<std::size_t>(1, max_iterations)) {}

bool PathPostProcessor::SegmentIsValid(
    const JointPositions& start, const JointPositions& goal,
    const SegmentValidityCheck& segment_valid, std::string* message) const {
  if (!segment_valid) {
    return true;
  }
  return segment_valid(start, goal, message);
}

ValidationResult PathPostProcessor::Simplify(
    const JointPath& path, const SegmentValidityCheck& segment_valid,
    JointPath* simplified) const {
  if (simplified == nullptr) {
    return {false, "simplified path output pointer is null"};
  }
  const ValidationResult validation = ValidateJointPath(path);
  if (!validation.ok) {
    return validation;
  }
  if (path.points.size() <= 2) {
    *simplified = path;
    return {true, ""};
  }

  JointPath result;
  result.joint_names = path.joint_names;
  result.points.push_back(path.points.front());
  std::size_t current = 0;
  std::size_t iterations = 0;
  while (current + 1 < path.points.size()) {
    std::size_t chosen = current + 1;
    for (std::size_t candidate = path.points.size() - 1;
         candidate > current; --candidate) {
      if (++iterations > max_iterations_) {
        return {false, "path simplification iteration budget exhausted"};
      }
      std::string message;
      if (SegmentIsValid(path.points[current], path.points[candidate],
                         segment_valid, &message)) {
        chosen = candidate;
        break;
      }
    }
    result.points.push_back(path.points[chosen]);
    current = chosen;
  }
  *simplified = std::move(result);
  return {true, ""};
}

ValidationResult PathPostProcessor::Smooth(
    const JointPath& path, double alpha, int iterations,
    const SegmentValidityCheck& segment_valid, JointPath* smoothed) const {
  if (smoothed == nullptr) {
    return {false, "smoothed path output pointer is null"};
  }
  const ValidationResult validation = ValidateJointPath(path);
  if (!validation.ok) {
    return validation;
  }
  if (iterations <= 0 || path.points.size() < 3) {
    *smoothed = path;
    return {true, ""};
  }
  const double weight = std::clamp(alpha, 0.0, 1.0);
  JointPath current = path;
  std::string stop_message;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    JointPath next = current;
    for (std::size_t index = 1; index + 1 < current.points.size(); ++index) {
      for (std::size_t joint = 0; joint < current.joint_names.size(); ++joint) {
        const double neighbor_average =
            0.5 * (current.points[index - 1][joint] +
                   current.points[index + 1][joint]);
        next.points[index][joint] =
            (1.0 - weight) * current.points[index][joint] +
            weight * neighbor_average;
      }
    }
    bool valid = true;
    for (std::size_t segment = 0; segment + 1 < next.points.size();
         ++segment) {
      std::string message;
      if (!SegmentIsValid(next.points[segment], next.points[segment + 1],
                          segment_valid, &message)) {
        valid = false;
        stop_message =
            "smoothing stopped because a segment failed validation; the last "
            "valid path was kept";
        break;
      }
    }
    if (!valid) {
      break;
    }
    current = std::move(next);
  }
  *smoothed = std::move(current);
  return {true, stop_message};
}

}  // namespace planning
}  // namespace tramcar
