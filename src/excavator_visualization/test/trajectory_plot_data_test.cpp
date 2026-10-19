#include "excavator_visualization/trajectory_plot_data.hpp"

#include <gtest/gtest.h>

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

JointTrajectory MakeTrajectory() {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a", "joint_b"};
  trajectory.points = {
      {0.0, {0.0, 1.0}, {0.0, 0.0}, {0.0, 0.0}},
      {1.0, {0.5, 2.0}, {0.5, 1.0}, {0.0, 0.0}},
      {2.0, {1.0, 3.0}, {0.0, 0.0}, {0.0, 0.0}}};
  return trajectory;
}

TEST(TrajectoryPlotDataTest, SamplesReferenceSeries) {
  TrajectoryPlotData data;
  data.SetReference(MakeTrajectory());
  EXPECT_TRUE(data.HasReference());
  EXPECT_EQ(data.JointCount(), 2U);
  EXPECT_DOUBLE_EQ(data.Duration(), 2.0);

  const PlotSeries position =
      data.ReferenceSeries(0, PlotQuantity::kPosition, 20);
  ASSERT_GE(position.times.size(), 2U);
  EXPECT_DOUBLE_EQ(position.times.front(), 0.0);
  EXPECT_DOUBLE_EQ(position.times.back(), 2.0);
  EXPECT_DOUBLE_EQ(position.values.front(), 0.0);
  EXPECT_DOUBLE_EQ(position.values.back(), 1.0);

  const PlotSeries velocity =
      data.ReferenceSeries(1, PlotQuantity::kVelocity, 20);
  ASSERT_FALSE(velocity.values.empty());
  EXPECT_NEAR(velocity.values[velocity.values.size() / 2], 1.0, 0.15);
}

TEST(TrajectoryPlotDataTest, AlignsMeasuredSamples) {
  TrajectoryPlotData data;
  data.SetReference(MakeTrajectory());
  RobotState state;
  state.positions = {0.25, 1.5};
  state.velocities = {0.5, 0.5};
  data.AddActual(0.5, state);
  EXPECT_EQ(data.ActualSampleCount(), 1U);

  const PlotSeries actual =
      data.ActualSeries(0, PlotQuantity::kPosition);
  ASSERT_EQ(actual.times.size(), 1U);
  EXPECT_DOUBLE_EQ(actual.values[0], 0.25);

  const PlotSeries reference =
      data.AlignedReferenceSeries(actual, 0, PlotQuantity::kPosition);
  ASSERT_EQ(reference.times.size(), 1U);
  EXPECT_NEAR(reference.values[0], 0.25, 1.0e-9);
}

TEST(TrajectoryPlotDataTest, IgnoresOutOfOrderSamples) {
  TrajectoryPlotData data;
  data.SetReference(MakeTrajectory());
  RobotState state;
  state.positions = {0.0, 0.0};
  state.velocities = {0.0, 0.0};
  data.AddActual(1.0, state);
  data.AddActual(0.1, state);
  EXPECT_EQ(data.ActualSampleCount(), 1U);
  EXPECT_DOUBLE_EQ(data.LatestActualTime(), 1.0);
}

TEST(TrajectoryPlotDataTest, DerivesMeasuredAcceleration) {
  TrajectoryPlotData data;
  data.SetReference(MakeTrajectory());
  // Constant acceleration of 5 rad/s^2: velocity rises by 0.5 every 0.1 s.
  // Central differences and the zero-phase smoothing window must preserve it.
  for (int index = 0; index < 5; ++index) {
    RobotState state;
    state.positions = {0.0, 1.0};
    state.velocities = {0.5 * static_cast<double>(index), 0.0};
    data.AddActual(0.1 * static_cast<double>(index), state);
  }

  const PlotSeries acceleration =
      data.ActualSeries(0, PlotQuantity::kAcceleration);
  ASSERT_EQ(acceleration.values.size(), 5U);
  for (double value : acceleration.values) {
    EXPECT_NEAR(value, 5.0, 1.0e-9);
  }
}

TEST(TrajectoryPlotDataTest, NewReferenceResetsMeasuredSamples) {
  TrajectoryPlotData data;
  data.SetReference(MakeTrajectory());
  RobotState state;
  state.positions = {0.0, 0.0};
  data.AddActual(0.5, state);
  data.SetReference(MakeTrajectory());
  EXPECT_EQ(data.ActualSampleCount(), 0U);
}

}  // namespace
}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
