#include "excavator_visualization/trajectory_plot_data.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

constexpr std::size_t kMaxActualSamples = 20000;

double ValueAt(const JointTrajectoryPoint& point, PlotQuantity quantity,
               std::size_t joint) {
  const JointPositions* values = nullptr;
  switch (quantity) {
    case PlotQuantity::kPosition:
      values = &point.positions;
      break;
    case PlotQuantity::kVelocity:
      values = &point.velocities;
      break;
    case PlotQuantity::kAcceleration:
      values = &point.accelerations;
      break;
  }
  if (values == nullptr || joint >= values->size()) {
    return 0.0;
  }
  return (*values)[joint];
}

double MeasuredValue(const RobotState& state, PlotQuantity quantity,
                     std::size_t joint) {
  const JointPositions* values = nullptr;
  switch (quantity) {
    case PlotQuantity::kPosition:
      values = &state.positions;
      break;
    case PlotQuantity::kVelocity:
      values = &state.velocities;
      break;
    case PlotQuantity::kAcceleration:
      values = &state.accelerations;
      break;
  }
  if (values == nullptr || joint >= values->size()) {
    return 0.0;
  }
  return (*values)[joint];
}

bool SampleReferenceAt(const JointTrajectory& trajectory, double t,
                       std::size_t joint, PlotQuantity quantity,
                       double* value) {
  if (trajectory.points.empty() || value == nullptr) {
    return false;
  }
  if (t <= trajectory.points.front().t) {
    *value = ValueAt(trajectory.points.front(), quantity, joint);
    return true;
  }
  if (t >= trajectory.points.back().t) {
    *value = ValueAt(trajectory.points.back(), quantity, joint);
    return true;
  }
  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    const JointTrajectoryPoint& first = trajectory.points[index - 1];
    const JointTrajectoryPoint& second = trajectory.points[index];
    if (t > second.t) {
      continue;
    }
    const double duration = second.t - first.t;
    const double ratio =
        duration > 0.0 ? (t - first.t) / duration : 0.0;
    *value = ValueAt(first, quantity, joint) +
             ratio * (ValueAt(second, quantity, joint) -
                      ValueAt(first, quantity, joint));
    return true;
  }
  return false;
}

}  // namespace

void TrajectoryPlotData::SetReference(const JointTrajectory& trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);
  reference_ = trajectory;
  has_reference_ = true;
  actual_.clear();
  latest_actual_time_ = 0.0;
}

void TrajectoryPlotData::ClearReference() {
  std::lock_guard<std::mutex> lock(mutex_);
  reference_ = JointTrajectory{};
  has_reference_ = false;
  actual_.clear();
  latest_actual_time_ = 0.0;
}

bool TrajectoryPlotData::HasReference() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_reference_;
}

JointTrajectory TrajectoryPlotData::GetReference() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reference_;
}

void TrajectoryPlotData::AddActual(double t, const RobotState& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(t)) {
    return;
  }
  // A new trajectory clears the history explicitly. A slightly out-of-order
  // time-stamped sample (status extrapolation jitter) must not silently erase
  // the measured curve, so it is ignored instead.
  if (!actual_.empty() && t < latest_actual_time_) {
    return;
  }
  RobotState sample = state;
  sample.t = t;
  latest_actual_time_ = std::max(latest_actual_time_, t);
  actual_.push_back(std::move(sample));
  if (actual_.size() > kMaxActualSamples) {
    actual_.erase(actual_.begin(),
                  actual_.begin() +
                      static_cast<std::ptrdiff_t>(actual_.size() -
                                                  kMaxActualSamples));
  }
}

void TrajectoryPlotData::ClearActual() {
  std::lock_guard<std::mutex> lock(mutex_);
  actual_.clear();
  latest_actual_time_ = 0.0;
}

std::size_t TrajectoryPlotData::ActualSampleCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_.size();
}

std::size_t TrajectoryPlotData::JointCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reference_.joint_names.size();
}

std::vector<std::string> TrajectoryPlotData::JointNames() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reference_.joint_names;
}

double TrajectoryPlotData::Duration() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reference_.points.empty() ? 0.0 : reference_.points.back().t;
}

double TrajectoryPlotData::LatestActualTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_actual_time_;
}

PlotSeries TrajectoryPlotData::ReferenceSeries(
    std::size_t joint, PlotQuantity quantity, std::size_t max_samples) const {
  std::lock_guard<std::mutex> lock(mutex_);
  PlotSeries series;
  if (!has_reference_ || reference_.points.size() < 2 ||
      joint >= reference_.joint_names.size()) {
    return series;
  }
  const double duration = reference_.points.back().t;
  const std::size_t sample_count =
      std::max<std::size_t>(2, std::min<std::size_t>(
                                   max_samples, std::max<std::size_t>(
                                                    2, reference_.points.size() *
                                                           4)));
  series.times.reserve(sample_count);
  series.values.reserve(sample_count);
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double t = duration * static_cast<double>(index) /
                     static_cast<double>(sample_count - 1);
    double value = 0.0;
    if (SampleReferenceAt(reference_, t, joint, quantity, &value)) {
      series.times.push_back(t);
      series.values.push_back(value);
    }
  }
  return series;
}

PlotSeries TrajectoryPlotData::AlignedReferenceSeries(
    const PlotSeries& actual, std::size_t joint, PlotQuantity quantity) const {
  std::lock_guard<std::mutex> lock(mutex_);
  PlotSeries series;
  if (!has_reference_) {
    return series;
  }
  series.times.reserve(actual.times.size());
  series.values.reserve(actual.times.size());
  for (double t : actual.times) {
    double value = 0.0;
    if (SampleReferenceAt(reference_, t, joint, quantity, &value)) {
      series.times.push_back(t);
      series.values.push_back(value);
    }
  }
  return series;
}

PlotSeries TrajectoryPlotData::ActualSeries(std::size_t joint,
                                            PlotQuantity quantity) const {
  std::lock_guard<std::mutex> lock(mutex_);
  PlotSeries series;
  if (reference_.joint_names.size() <= joint) {
    return series;
  }
  series.times.reserve(actual_.size());
  series.values.reserve(actual_.size());
  if (quantity != PlotQuantity::kAcceleration) {
    for (const RobotState& state : actual_) {
      series.times.push_back(state.t);
      series.values.push_back(MeasuredValue(state, quantity, joint));
    }
    return series;
  }

  // JointState does not provide acceleration. Derive it with central
  // differences from measured velocity, then apply a short zero-phase moving
  // average. Differentiating a 100 Hz velocity signal amplifies the small
  // velocity tracking error of the controller; central differences plus a
  // 5-sample window keep the real acceleration profile without the jagged
  // sample-to-sample noise. Endpoints use one-sided differences.
  const std::size_t count = actual_.size();
  if (count == 0) {
    return series;
  }
  std::vector<double> times(count);
  std::vector<double> velocities(count);
  for (std::size_t index = 0; index < count; ++index) {
    times[index] = actual_[index].t;
    velocities[index] = MeasuredValue(actual_[index],
                                      PlotQuantity::kVelocity, joint);
  }
  std::vector<double> raw(count, 0.0);
  for (std::size_t index = 0; index < count; ++index) {
    if (count < 2) {
      break;
    }
    if (index == 0) {
      const double dt = times[1] - times[0];
      raw[index] = dt > 1.0e-9
                       ? (velocities[1] - velocities[0]) / dt
                       : 0.0;
    } else if (index + 1 == count) {
      const double dt = times[count - 1] - times[count - 2];
      raw[index] = dt > 1.0e-9
                       ? (velocities[count - 1] - velocities[count - 2]) / dt
                       : 0.0;
    } else {
      const double dt = times[index + 1] - times[index - 1];
      raw[index] = dt > 1.0e-9
                       ? (velocities[index + 1] - velocities[index - 1]) / dt
                       : 0.0;
    }
  }
  constexpr std::size_t kHalfWindow = 2;
  series.times.reserve(count);
  series.values.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const std::size_t first = index > kHalfWindow ? index - kHalfWindow : 0;
    const std::size_t last = std::min(count - 1, index + kHalfWindow);
    double sum = 0.0;
    for (std::size_t sample = first; sample <= last; ++sample) {
      sum += raw[sample];
    }
    series.times.push_back(times[index]);
    series.values.push_back(
        sum / static_cast<double>(last - first + 1));
  }
  return series;
}

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
