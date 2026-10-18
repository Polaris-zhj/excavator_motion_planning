#include "excavator_sim_viz/kinematic_simulator.hpp"

#include <gtest/gtest.h>

namespace excavator_sim_viz {
namespace {

tramcar::planning::JointTrajectory MakeTrajectory() {
  tramcar::planning::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory.points = {{0.0, {0.0}, {}, {}}, {1.0, {1.0}, {}, {}}};
  return trajectory;
}

TEST(KinematicSimulatorTest, StepsAlongSubmittedTrajectory) {
  KinematicSimulator simulator({"joint_a"}, {0.0});
  ASSERT_TRUE(simulator.SubmitTrajectory(MakeTrajectory()).ok);

  SimulationState state;
  ASSERT_TRUE(simulator.Step(0.5, &state).ok);
  ASSERT_EQ(state.positions.size(), 1U);
  EXPECT_NEAR(state.positions[0], 0.5, 1.0e-8);
  EXPECT_TRUE(state.trajectory_active);

  ASSERT_TRUE(simulator.Step(0.5, &state).ok);
  EXPECT_DOUBLE_EQ(state.positions[0], 1.0);
  EXPECT_TRUE(state.trajectory_finished);
  EXPECT_FALSE(state.trajectory_active);
}

TEST(KinematicSimulatorTest, RejectsDiscontinuousReplacement) {
  KinematicSimulator simulator({"joint_a"}, {0.0});
  ASSERT_TRUE(simulator.SubmitTrajectory(MakeTrajectory()).ok);
  SimulationState state;
  ASSERT_TRUE(simulator.Step(0.5, &state).ok);

  auto replacement = MakeTrajectory();
  replacement.points.front().positions[0] = -0.5;
  EXPECT_FALSE(simulator.SubmitTrajectory(replacement).ok);
}

TEST(KinematicSimulatorTest, RejectsDifferentJointOrdering) {
  KinematicSimulator simulator({"joint_a", "joint_b"}, {0.0, 0.0});
  EXPECT_FALSE(simulator.SubmitTrajectory(MakeTrajectory()).ok);
}

}  // namespace
}  // namespace excavator_sim_viz
