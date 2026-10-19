#include "kinematics/workspace_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tramcar {
namespace planning {
namespace {

bool IsFinite(const Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool IsOrdered(const WorkspaceBox& box) {
  return box.minimum.x <= box.maximum.x &&
         box.minimum.y <= box.maximum.y &&
         box.minimum.z <= box.maximum.z;
}

bool IsInside(const WorkspaceBox& box, const Vector3& point, double margin) {
  return point.x >= box.minimum.x - margin &&
         point.x <= box.maximum.x + margin &&
         point.y >= box.minimum.y - margin &&
         point.y <= box.maximum.y + margin &&
         point.z >= box.minimum.z - margin &&
         point.z <= box.maximum.z + margin;
}

double DistanceToBoundary(const WorkspaceBox& box, const Vector3& point,
                          double margin) {
  const double dx = std::min(point.x - (box.minimum.x + margin),
                             (box.maximum.x - margin) - point.x);
  const double dy = std::min(point.y - (box.minimum.y + margin),
                             (box.maximum.y - margin) - point.y);
  const double dz = std::min(point.z - (box.minimum.z + margin),
                             (box.maximum.z - margin) - point.z);
  return std::min({dx, dy, dz});
}

}  // namespace

WorkspaceChecker::WorkspaceChecker(WorkspaceConfig config)
    : config_(std::move(config)) {}

ValidationResult WorkspaceChecker::Validate() const {
  if (!std::isfinite(config_.margin) || config_.margin < 0.0) {
    return {false, "workspace margin must be finite and non-negative"};
  }
  const auto validate_boxes = [](const std::vector<WorkspaceBox>& boxes,
                                 const char* label) -> ValidationResult {
    for (const WorkspaceBox& box : boxes) {
      if (!IsFinite(box.minimum) || !IsFinite(box.maximum) ||
          !IsOrdered(box)) {
        return {false, std::string(label) +
                           " workspace box must be finite and ordered"};
      }
    }
    return {true, ""};
  };
  const ValidationResult allowed =
      validate_boxes(config_.allowed_regions, "allowed");
  if (!allowed.ok) {
    return allowed;
  }
  return validate_boxes(config_.forbidden_regions, "forbidden");
}

WorkspaceCheckResult WorkspaceChecker::Check(const Pose3& pose) const {
  WorkspaceCheckResult result;
  const ValidationResult validation = Validate();
  if (!validation.ok) {
    result.message = validation.message;
    return result;
  }
  if (!IsFinite(pose.position)) {
    result.message = "pose position contains non-finite values";
    return result;
  }

  for (const WorkspaceBox& box : config_.forbidden_regions) {
    if (IsInside(box, pose.position, config_.margin)) {
      result.valid = true;
      result.inside = false;
      result.distance_to_boundary = -1.0;
      result.message = "pose is inside a forbidden workspace region";
      return result;
    }
  }

  if (config_.allowed_regions.empty()) {
    result.valid = true;
    result.inside = true;
    result.distance_to_boundary = std::numeric_limits<double>::infinity();
    result.message = "workspace is unbounded";
    return result;
  }

  double best_distance = -std::numeric_limits<double>::infinity();
  for (const WorkspaceBox& box : config_.allowed_regions) {
    const double distance =
        DistanceToBoundary(box, pose.position, config_.margin);
    if (distance > best_distance) {
      best_distance = distance;
    }
  }
  result.valid = true;
  result.distance_to_boundary = best_distance;
  result.inside = best_distance >= 0.0;
  result.message =
      result.inside ? "pose is inside the allowed workspace"
                    : "pose is outside the allowed workspace";
  return result;
}

const WorkspaceConfig& WorkspaceChecker::config() const { return config_; }

}  // namespace planning
}  // namespace tramcar
