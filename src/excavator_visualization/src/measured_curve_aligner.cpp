#include "excavator_visualization/measured_curve_aligner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace excavator {
namespace planning {
namespace visualization {
namespace {

// A genuine run-clock restart is large (the controller resets elapsed_time to
// zero). Ignore small backwards jitter caused by timestamp extrapolation.
constexpr double kClockRestartTolerance = 0.05;
constexpr double kMinimumDeltaTime = 1.0e-3;

double MaxDifference(const JointPositions& first,
                     const JointPositions& second) {
  double maximum = 0.0;
  for (std::size_t index = 0; index < first.size(); ++index) {
    maximum = std::max(maximum, std::abs(first[index] - second[index]));
  }
  return maximum;
}

}  // namespace

MeasuredCurveAligner::MeasuredCurveAligner(MeasuredCurveAlignerConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.start_tolerance) ||
      config_.start_tolerance < 0.0) {
    config_.start_tolerance = 0.05;
  }
  if (!std::isfinite(config_.discontinuity_tolerance) ||
      config_.discontinuity_tolerance < 0.0) {
    config_.discontinuity_tolerance = 0.25;
  }
  if (!std::isfinite(config_.max_plausible_speed) ||
      config_.max_plausible_speed <= 0.0) {
    config_.max_plausible_speed = 8.0;
  }
}

void MeasuredCurveAligner::Reset() {
  waiting_for_start_ = true;
  has_last_ = false;
  last_run_time_ = 0.0;
  last_positions_.clear();
  time_offset_ = 0.0;
}

MeasuredSampleAlignment MeasuredCurveAligner::Update(
    double run_time, const JointPositions& reference_start,
    const JointPositions& measured_positions) {
  MeasuredSampleAlignment alignment;
  if (!std::isfinite(run_time) || reference_start.empty() ||
      measured_positions.size() != reference_start.size()) {
    return alignment;
  }

  // A new execution restarted the controller clock. The whole previously
  // collected curve belongs to the old run.
  if (has_last_ && run_time + kClockRestartTolerance < last_run_time_) {
    Reset();
    alignment.restarted = true;
  }

  // Even if the execution-status message has not arrived yet (or the clock was
  // not reset), the arm can jump to the new start when the simulator is
  // re-initialized. Do not append that jump to the old measured curve.
  if (!waiting_for_start_ && has_last_) {
    const double delta_time =
        std::max(kMinimumDeltaTime, run_time - last_run_time_);
    const double jump = MaxDifference(measured_positions, last_positions_);
    if (jump > config_.discontinuity_tolerance &&
        jump / delta_time > config_.max_plausible_speed) {
      Reset();
      alignment.restarted = true;
    }
  }

  if (waiting_for_start_) {
    // The arm may still be at the previous endpoint. Only start collecting
    // once it actually reaches the trajectory start.
    const double start_error =
        MaxDifference(measured_positions, reference_start);
    if (start_error > config_.start_tolerance) {
      last_run_time_ = run_time;
      last_positions_ = measured_positions;
      has_last_ = true;
      return alignment;
    }
    waiting_for_start_ = false;
    time_offset_ = run_time;
    alignment.accept = true;
    alignment.time = 0.0;
  } else {
    alignment.accept = true;
    alignment.time = run_time - time_offset_;
  }

  last_run_time_ = run_time;
  last_positions_ = measured_positions;
  has_last_ = true;
  if (alignment.time < 0.0) {
    alignment.accept = false;
  }
  return alignment;
}

}  // namespace visualization
}  // namespace planning
}  // namespace excavator
