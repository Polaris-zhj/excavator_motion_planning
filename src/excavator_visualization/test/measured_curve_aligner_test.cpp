#include "excavator_visualization/measured_curve_aligner.hpp"

#include <gtest/gtest.h>

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

const JointPositions kStart = {0.0, 0.0};
const JointPositions kEndpoint = {1.0, -1.0};

TEST(MeasuredCurveAlignerTest, DiscardsSamplesUntilTheArmReachesStart) {
  MeasuredCurveAligner aligner;

  // The previous run ended at the endpoint. These samples must not be plotted
  // against the new reference: otherwise the dashed curve starts at the wrong
  // position and jumps when the arm reaches the start.
  MeasuredSampleAlignment alignment =
      aligner.Update(0.0, kStart, kEndpoint);
  EXPECT_FALSE(alignment.accept);
  EXPECT_FALSE(alignment.restarted);
  EXPECT_TRUE(aligner.waiting_for_start());

  alignment = aligner.Update(0.5, kStart, kEndpoint);
  EXPECT_FALSE(alignment.accept);

  // Reaching the start anchors this instant to t=0.
  alignment = aligner.Update(0.8, kStart, kStart);
  EXPECT_TRUE(alignment.accept);
  EXPECT_DOUBLE_EQ(alignment.time, 0.0);
  EXPECT_FALSE(aligner.waiting_for_start());

  alignment = aligner.Update(1.0, kStart, JointPositions{0.2, -0.2});
  EXPECT_TRUE(alignment.accept);
  EXPECT_NEAR(alignment.time, 0.2, 1.0e-12);
}

TEST(MeasuredCurveAlignerTest, RestartsWhenTheExecutionClockResets) {
  MeasuredCurveAligner aligner;
  ASSERT_TRUE(aligner.Update(2.0, kStart, kStart).accept);
  // A full endpoint stroke over one second is a plausible controlled motion
  // and must not be mistaken for a teleport.
  ASSERT_TRUE(aligner.Update(3.0, kStart, kEndpoint).accept);

  // A second execution starts at elapsed_time = 0 while the arm is already
  // back at the trajectory start.
  const MeasuredSampleAlignment alignment =
      aligner.Update(0.0, kStart, kStart);
  EXPECT_TRUE(alignment.restarted);
  EXPECT_TRUE(alignment.accept);
  EXPECT_DOUBLE_EQ(alignment.time, 0.0);
}

TEST(MeasuredCurveAlignerTest, RestartsOnTeleportBeforeTheStatusArrives) {
  MeasuredCurveAligner aligner;
  ASSERT_TRUE(aligner.Update(0.0, kStart, kStart).accept);
  ASSERT_TRUE(aligner.Update(1.0, kStart, kEndpoint).accept);

  // The simulator was re-initialized, but the panel has not received the new
  // execution status yet. The position jump itself must break the curve.
  const MeasuredSampleAlignment alignment =
      aligner.Update(1.01, kStart, kStart);
  EXPECT_TRUE(alignment.restarted);
  EXPECT_TRUE(alignment.accept);
  EXPECT_DOUBLE_EQ(alignment.time, 0.0);
}

TEST(MeasuredCurveAlignerTest, KeepsTheCurveForTimestampJitter) {
  MeasuredCurveAligner aligner;
  ASSERT_TRUE(aligner.Update(0.0, kStart, kStart).accept);
  const MeasuredSampleAlignment alignment =
      aligner.Update(-0.01, kStart, JointPositions{0.01, 0.0});
  // Small jitter is not a new run. The negative aligned time is rejected by
  // the caller (TrajectoryPlotData also ignores it), but the curve is kept.
  EXPECT_FALSE(alignment.restarted);
  EXPECT_FALSE(alignment.accept);
  EXPECT_FALSE(aligner.waiting_for_start());
}

TEST(MeasuredCurveAlignerTest, RejectsMismatchedDimensions) {
  MeasuredCurveAligner aligner;
  const MeasuredSampleAlignment alignment =
      aligner.Update(0.0, kStart, JointPositions{0.0});
  EXPECT_FALSE(alignment.accept);
  EXPECT_TRUE(aligner.waiting_for_start());
}

TEST(MeasuredCurveAlignerTest, ResetWaitsForTheStartAgain) {
  MeasuredCurveAligner aligner;
  ASSERT_TRUE(aligner.Update(0.0, kStart, kStart).accept);
  aligner.Reset();
  EXPECT_TRUE(aligner.waiting_for_start());

  const MeasuredSampleAlignment alignment =
      aligner.Update(0.0, kStart, kEndpoint);
  EXPECT_FALSE(alignment.accept);
}

}  // namespace
}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
