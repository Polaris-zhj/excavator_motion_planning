#include "excavator_common/linear_trajectory_interpolator.hpp"

#include <gtest/gtest.h>

namespace excavator {
namespace planning {
namespace {

TEST(LinearTrajectoryInterpolatorTest, SamplesBetweenWaypoints) {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory.points = {{0.0, {0.0}, {0.0}, {0.0}},
                       {2.0, {1.0}, {0.0}, {0.0}}};
  JointTrajectoryPoint sample;
  const ValidationResult result =
      LinearTrajectoryInterpolator().Sample(trajectory, 1.0, &sample);

  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_DOUBLE_EQ(sample.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(sample.t, 1.0);
}

TEST(LinearTrajectoryInterpolatorTest, RejectsTimeOutsideTrajectory) {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory.points = {{0.0, {0.0}, {0.0}, {0.0}},
                       {2.0, {1.0}, {0.0}, {0.0}}};
  JointTrajectoryPoint sample;
  EXPECT_FALSE(
      LinearTrajectoryInterpolator().Sample(trajectory, -0.1, &sample).ok);
  EXPECT_FALSE(
      LinearTrajectoryInterpolator().Sample(trajectory, 2.1, &sample).ok);
}

}  // namespace
}  // namespace planning
}  // namespace excavator
