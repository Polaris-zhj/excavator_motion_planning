#ifndef EXCAVATOR_VISUALIZATION__MEASURED_CURVE_ALIGNER_HPP_
#define EXCAVATOR_VISUALIZATION__MEASURED_CURVE_ALIGNER_HPP_

#include "excavator_common/robot_state.hpp"

namespace excavator {
namespace planning {
namespace visualization {

struct MeasuredCurveAlignerConfig {
  // Maximum joint error at which a measured sample is considered to be on the
  // trajectory start. Keep this consistent with the controller's start
  // continuity tolerance.
  double start_tolerance{0.05};
  // A state change larger than this cannot be produced by the velocity-limited
  // controller when it happens in one control cycle. It means the arm was
  // re-initialized (or a new run started) and the old measured curve must not
  // be connected to the new samples.
  double discontinuity_tolerance{0.25};
  // Implied joint speed above which a position change is treated as a jump.
  double max_plausible_speed{8.0};
};

struct MeasuredSampleAlignment {
  // The sample belongs to the current run and may be appended to the curve.
  bool accept{false};
  // The previous measured curve is obsolete (new execution, clock restart or
  // discontinuity). The caller must clear it before appending this sample.
  bool restarted{false};
  // Time on the reference trajectory clock (first accepted sample is t=0).
  double time{0.0};
};

// Aligns measured /joint_states samples with the planned reference trajectory.
//
// A new run may start while the arm is still at the previous endpoint. Samples
// collected in that state must not be plotted against t=0 of the new
// reference, otherwise the dashed curve starts at the wrong position and shows
// a jump when the arm finally reaches the trajectory start. This class waits
// until the measured joints are at the reference start, anchors that instant
// to t=0 and reports discontinuities that require a fresh curve.
class MeasuredCurveAligner {
 public:
  explicit MeasuredCurveAligner(
      MeasuredCurveAlignerConfig config = MeasuredCurveAlignerConfig());

  // Forget the current run. The next matching sample becomes the new t=0.
  void Reset();

  bool waiting_for_start() const { return waiting_for_start_; }

  MeasuredSampleAlignment Update(double run_time,
                                 const JointPositions& reference_start,
                                 const JointPositions& measured_positions);

 private:
  MeasuredCurveAlignerConfig config_;
  bool waiting_for_start_{true};
  bool has_last_{false};
  double last_run_time_{0.0};
  JointPositions last_positions_;
  double time_offset_{0.0};
};

}  // namespace visualization
}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_VISUALIZATION__MEASURED_CURVE_ALIGNER_HPP_
