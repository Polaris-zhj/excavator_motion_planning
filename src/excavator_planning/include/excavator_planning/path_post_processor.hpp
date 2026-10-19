#ifndef EXCAVATOR_PLANNING__PATH_POST_PROCESSOR_HPP_
#define EXCAVATOR_PLANNING__PATH_POST_PROCESSOR_HPP_

#include <cstddef>
#include <functional>
#include <string>

#include "excavator_common/joint_trajectory.hpp"

namespace tramcar {
namespace planning {

// Path simplification/smoothing are kept separate from time parameterization
// so either stage can be replaced or disabled. All post-processing must be
// followed by a fresh collision and limit check in the planning pipeline.
class PathPostProcessor {
 public:
  using SegmentValidityCheck =
      std::function<bool(const JointPositions&, const JointPositions&,
                         std::string*)>;

  explicit PathPostProcessor(std::size_t max_iterations = 10000);

  // Greedy shortcut simplification. Endpoints are always preserved.
  ValidationResult Simplify(const JointPath& path,
                            const SegmentValidityCheck& segment_valid,
                            JointPath* simplified) const;

  // Weighted moving-average smoothing. If any smoothed segment fails the
  // validity check, the original path is returned unchanged.
  ValidationResult Smooth(const JointPath& path, double alpha, int iterations,
                          const SegmentValidityCheck& segment_valid,
                          JointPath* smoothed) const;

 private:
  bool SegmentIsValid(const JointPositions& start,
                      const JointPositions& goal,
                      const SegmentValidityCheck& segment_valid,
                      std::string* message) const;

  std::size_t max_iterations_;
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_PLANNING__PATH_POST_PROCESSOR_HPP_
