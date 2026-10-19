#include "excavator_sim_viz/kinematic_simulator.hpp"

#include <gtest/gtest.h>

namespace tramcar {
namespace planning {
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

TEST(KinematicSimulatorTest, AppliesCommandsAndLimitsAcceleration) {
  SimulatorLimits limits;
  limits.max_velocity = {0.5};
  limits.max_acceleration = {1.0};
  limits.lower_position_limits = {-1.0};
  limits.upper_position_limits = {1.0};
  KinematicSimulator simulator({"joint_a"}, {0.0}, limits);

  ControlCommand command;
  command.velocities = {2.0};
  SimulationState state;
  const ValidationResult result = simulator.ApplyCommand(command, 0.1, &state);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_NEAR(state.velocities[0], 0.1, 1.0e-9);
  EXPECT_NEAR(state.positions[0], 0.01, 1.0e-9);
}

TEST(KinematicSimulatorTest, PositionLimitEngagesSafetyFault) {
  SimulatorLimits limits;
  limits.max_velocity = {1.0};
  limits.max_acceleration = {10.0};
  limits.lower_position_limits = {-0.05};
  limits.upper_position_limits = {0.05};
  KinematicSimulator simulator({"joint_a"}, {0.0}, limits);

  ControlCommand command;
  command.velocities = {1.0};
  SimulationState state;
  EXPECT_FALSE(simulator.ApplyCommand(command, 0.1, &state).ok);
  EXPECT_TRUE(state.safety_fault);
  EXPECT_TRUE(simulator.HasSafetyFault());
  EXPECT_FALSE(simulator.ApplyCommand(command, 0.1, &state).ok);

  RobotState reset;
  reset.positions = {0.0};
  reset.velocities = {0.0};
  ASSERT_TRUE(simulator.SetState(reset).ok);
  EXPECT_FALSE(simulator.HasSafetyFault());
  EXPECT_TRUE(simulator.ApplyCommand(command, 0.01, &state).ok);
}

TEST(KinematicSimulatorTest, RejectsCommandModeDuringTrajectoryPlayback) {
  KinematicSimulator simulator({"joint_a"}, {0.0});
  ASSERT_TRUE(simulator.SubmitTrajectory(MakeTrajectory()).ok);
  ControlCommand command;
  command.velocities = {0.1};
  SimulationState state;
  EXPECT_FALSE(simulator.ApplyCommand(command, 0.01, &state).ok);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
