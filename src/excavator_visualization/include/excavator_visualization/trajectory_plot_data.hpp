#ifndef EXCAVATOR_VISUALIZATION__TRAJECTORY_PLOT_DATA_HPP_
#define EXCAVATOR_VISUALIZATION__TRAJECTORY_PLOT_DATA_HPP_

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_common/robot_state.hpp"

namespace tramcar {
namespace planning {
namespace visualization {

enum class PlotQuantity {
  kPosition = 0,
  kVelocity = 1,
  kAcceleration = 2
};

struct PlotSeries {
  std::vector<double> times;
  std::vector<double> values;
};

// Thread-safe cache used by ExcavatorJointPlotPanel. It stores the reference
// trajectory and measured samples, provides time-aligned series and never
// drives or modifies the controller/simulator.
class TrajectoryPlotData {
 public:
  void SetReference(const JointTrajectory& trajectory);
  void ClearReference();
  bool HasReference() const;
  JointTrajectory GetReference() const;

  void AddActual(double t, const RobotState& state);
  void ClearActual();
  std::size_t ActualSampleCount() const;

  std::size_t JointCount() const;
  std::vector<std::string> JointNames() const;
  double Duration() const;
  double LatestActualTime() const;

  // Reference curve sampled uniformly over the trajectory duration.
  PlotSeries ReferenceSeries(std::size_t joint, PlotQuantity quantity,
                             std::size_t max_samples = 400) const;

  // Reference curve sampled at the timestamps of the measured series, which
  // makes plotting reference and actual data on one time axis unambiguous.
  PlotSeries AlignedReferenceSeries(const PlotSeries& actual,
                                    std::size_t joint,
                                    PlotQuantity quantity) const;

  PlotSeries ActualSeries(std::size_t joint, PlotQuantity quantity) const;

 private:
  mutable std::mutex mutex_;
  JointTrajectory reference_;
  bool has_reference_{false};
  std::vector<RobotState> actual_;
  double latest_actual_time_{0.0};
};

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_VISUALIZATION__TRAJECTORY_PLOT_DATA_HPP_
