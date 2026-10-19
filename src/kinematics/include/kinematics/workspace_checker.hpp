#ifndef KINEMATICS__WORKSPACE_CHECKER_HPP_
#define KINEMATICS__WORKSPACE_CHECKER_HPP_

#include <string>
#include <vector>

#include "kinematics/kinematics_interface.hpp"

namespace tramcar {
namespace planning {

struct WorkspaceBox {
  Vector3 minimum;
  Vector3 maximum;
};

struct WorkspaceConfig {
  // An empty allowed_regions list means an unbounded workspace.
  std::vector<WorkspaceBox> allowed_regions;
  std::vector<WorkspaceBox> forbidden_regions;
  // Applied as an inward margin for allowed regions and an outward margin for
  // forbidden regions, in metres.
  double margin{0.0};
};

struct WorkspaceCheckResult {
  bool valid{false};
  bool inside{false};
  // Positive inside the allowed region, negative outside; +inf for an
  // unbounded workspace with no allowed regions.
  double distance_to_boundary{0.0};
  std::string message;
};

// Configurable Cartesian workspace check for the tool frame. Workspace
// membership is not a collision check and is not an IK reachability proof.
class WorkspaceChecker {
 public:
  explicit WorkspaceChecker(WorkspaceConfig config);

  ValidationResult Validate() const;
  WorkspaceCheckResult Check(const Pose3& pose) const;
  const WorkspaceConfig& config() const;

 private:
  WorkspaceConfig config_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // KINEMATICS__WORKSPACE_CHECKER_HPP_
