#include "excavator_sim_viz/trajectory_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "excavator_common/robot_state.hpp"
#include "excavator_sim_viz/controller_interface.hpp"
#include "excavator_sim_viz/kinematic_simulator.hpp"
#include "excavator_sim_viz/velocity_tracking_controller.hpp"

namespace tramcar {
namespace planning {
namespace {

JointTrajectory MakeTrajectory(double end_position, double duration = 2.0) {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory.points = {
      {0.0, {0.0}, {0.0}, {0.0}},
      {duration * 0.5, {end_position * 0.5}, {0.0}, {0.0}},
      {duration, {end_position}, {0.0}, {0.0}}};
  return trajectory;
}

TrajectoryControllerConfig MakeConfig() {
  TrajectoryControllerConfig config;
  config.joint_names = {"joint_a"};
  config.max_velocity = {1.0};
  config.max_acceleration = {2.0};
  config.lower_position_limits = {-2.0};
  config.upper_position_limits = {2.0};
  config.max_tracking_error = 0.6;
  config.completion_position_tolerance = 0.02;
  config.execution_timeout_margin = 3.0;
  config.start_continuity_tolerance = 0.05;
  config.update_position_continuity_tolerance = 0.05;
  config.update_velocity_continuity_tolerance = 1.0;
  return config;
}

VelocityControllerConfig MakeVelocityConfig() {
  VelocityControllerConfig config;
  config.max_velocity = {1.0};
  config.max_acceleration = {2.0};
  config.lower_position_limits = {-2.0};
  config.upper_position_limits = {2.0};
  config.position_gain = 4.0;
  return config;
}

std::unique_ptr<TrajectoryController> MakeController(
    TrajectoryControllerConfig config = MakeConfig()) {
  return std::make_unique<TrajectoryController>(
      std::move(config),
      std::make_unique<VelocityTrackingController>(MakeVelocityConfig()));
}

TEST(VelocityTrackingControllerTest, LimitsVelocityAndAcceleration) {
  VelocityTrackingController controller(MakeVelocityConfig());
  RobotState current;
  current.positions = {0.0};
  current.velocities = {0.0};
  JointTrajectoryPoint reference;
  reference.positions = {1.0};
  reference.velocities = {5.0};

  ControlCommand command;
  const ControllerOutput first =
      controller.ComputeCommand(current, reference, 0.01, &command);
  ASSERT_TRUE(first.ok) << first.message;
  EXPECT_TRUE(first.limited);
  EXPECT_LE(std::abs(command.velocities[0]), 0.0200001);

  current.velocities = {0.5};
  const ControllerOutput second =
      controller.ComputeCommand(current, reference, 0.1, &command);
  ASSERT_TRUE(second.ok) << second.message;
  EXPECT_LE(std::abs(command.velocities[0] - current.velocities[0]), 0.2 + 1e-9);
}

TEST(VelocityTrackingControllerTest, RejectsDimensionMismatch) {
  VelocityTrackingController controller(MakeVelocityConfig());
  RobotState current;
  current.positions = {0.0, 1.0};
  current.velocities = {0.0, 0.0};
  JointTrajectoryPoint reference;
  reference.positions = {1.0};
  ControlCommand command;
  EXPECT_FALSE(
      controller.ComputeCommand(current, reference, 0.01, &command).ok);
}

TEST(TrajectoryControllerTest, SubmitDoesNotStartExecution) {
  auto controller = MakeController();
  RobotState current;
  current.positions = {0.0};
  current.velocities = {0.0};

  ASSERT_TRUE(controller->SubmitTrajectory(MakeTrajectory(1.0), current).ok);
  EXPECT_EQ(controller->GetStatus().state, ExecutionState::kReady);

  const TrajectoryStepResult step = controller->Update(current, 0.01);
  EXPECT_TRUE(step.ok);
  EXPECT_FALSE(step.has_command);
  EXPECT_EQ(step.status.state, ExecutionState::kReady);
}

TEST(TrajectoryControllerTest, RejectsDiscontinuousStart) {
  auto controller = MakeController();
  RobotState current;
  current.positions = {0.5};
  current.velocities = {0.0};

  const ValidationResult result =
      controller->ExecuteTrajectory(MakeTrajectory(1.0), current);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(controller->GetStatus().state, ExecutionState::kIdle);
}

TEST(TrajectoryControllerTest, RunsClosedLoopToCompletion) {
  auto controller = MakeController();
  KinematicSimulator simulator({"joint_a"}, {0.0});
  RobotState current;
  current.positions = simulator.positions();
  current.velocities = simulator.velocities();

  ASSERT_TRUE(
      controller->ExecuteTrajectory(MakeTrajectory(1.0), current).ok);
  constexpr double kDt = 0.01;
  bool completed = false;
  for (int cycle = 0; cycle < 1000 && !completed; ++cycle) {
    current.positions = simulator.positions();
    current.velocities = simulator.velocities();
    const TrajectoryStepResult step = controller->Update(current, kDt);
    ASSERT_TRUE(step.ok) << step.message;
    ASSERT_NE(step.status.state, ExecutionState::kSafetyStop);
    ASSERT_NE(step.status.state, ExecutionState::kError);
    if (step.has_command) {
      SimulationState state;
      ASSERT_TRUE(simulator.ApplyCommand(step.command, kDt, &state).ok);
    }
    completed = step.status.state == ExecutionState::kCompleted;
  }
  ASSERT_TRUE(completed);
  EXPECT_NEAR(simulator.positions()[0], 1.0, 0.025);
  EXPECT_LE(controller->GetStatus().progress, 1.0);
}

TEST(TrajectoryControllerTest, InvalidUpdateKeepsActiveTrajectory) {
  auto controller = MakeController();
  RobotState current;
  current.positions = {0.0};
  current.velocities = {0.0};
  ASSERT_TRUE(
      controller->ExecuteTrajectory(MakeTrajectory(1.0), current).ok);

  JointTrajectory invalid_update = MakeTrajectory(1.5);
  invalid_update.points.front().positions[0] = 1.0;
  EXPECT_FALSE(controller->UpdateTrajectory(invalid_update, current).ok);
  EXPECT_EQ(controller->GetStatus().state, ExecutionState::kRunning);

  const TrajectoryStepResult step = controller->Update(current, 0.01);
  EXPECT_TRUE(step.ok);
  EXPECT_TRUE(step.has_command);
}

TEST(TrajectoryControllerTest, PauseHoldsProgressAndZeroCommand) {
  auto controller = MakeController();
  RobotState current;
  current.positions = {0.0};
  current.velocities = {0.0};
  ASSERT_TRUE(
      controller->ExecuteTrajectory(MakeTrajectory(1.0), current).ok);

  ASSERT_TRUE(controller->Update(current, 0.1).has_command);
  ASSERT_TRUE(controller->Pause().ok);
  const double elapsed_before = controller->GetStatus().elapsed_time;
  const TrajectoryStepResult paused = controller->Update(current, 0.1);
  EXPECT_TRUE(paused.ok);
  EXPECT_TRUE(paused.has_command);
  EXPECT_DOUBLE_EQ(paused.command.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(controller->GetStatus().elapsed_time, elapsed_before);
  EXPECT_EQ(controller->GetStatus().state, ExecutionState::kPaused);

  ASSERT_TRUE(controller->Resume().ok);
  EXPECT_EQ(controller->GetStatus().state, ExecutionState::kRunning);
}

TEST(TrajectoryControllerTest, ValidUpdateIsAppliedAndCompletesAtNewGoal) {
  auto controller = MakeController();
  KinematicSimulator simulator({"joint_a"}, {0.0});
  RobotState current;
  current.positions = simulator.positions();
  current.velocities = simulator.velocities();
  ASSERT_TRUE(
      controller->ExecuteTrajectory(MakeTrajectory(1.0), current).ok);

  constexpr double kDt = 0.01;
  for (int cycle = 0; cycle < 30; ++cycle) {
    current.positions = simulator.positions();
    current.velocities = simulator.velocities();
    const TrajectoryStepResult step = controller->Update(current, kDt);
    ASSERT_TRUE(step.has_command);
    SimulationState state;
    ASSERT_TRUE(simulator.ApplyCommand(step.command, kDt, &state).ok);
  }

  current.positions = simulator.positions();
  current.velocities = simulator.velocities();
  JointTrajectory replacement = MakeTrajectory(1.5);
  replacement.points.front().positions = current.positions;
  ASSERT_TRUE(controller->UpdateTrajectory(replacement, current).ok);
  EXPECT_EQ(controller->GetStatus().state, ExecutionState::kRunning);

  bool completed = false;
  for (int cycle = 0; cycle < 1000 && !completed; ++cycle) {
    current.positions = simulator.positions();
    current.velocities = simulator.velocities();
    const TrajectoryStepResult step = controller->Update(current, kDt);
    ASSERT_TRUE(step.ok) << step.message;
    if (step.has_command) {
      SimulationState state;
      ASSERT_TRUE(simulator.ApplyCommand(step.command, kDt, &state).ok);
    }
    completed = step.status.state == ExecutionState::kCompleted;
  }
  ASSERT_TRUE(completed);
  EXPECT_NEAR(simulator.positions()[0], 1.5, 0.03);
}

TEST(TrajectoryControllerTest, LargeTrackingErrorTriggersSafetyStop) {
  auto controller = MakeController();
  RobotState current;
  current.positions = {0.0};
  current.velocities = {0.0};
  ASSERT_TRUE(
      controller->ExecuteTrajectory(MakeTrajectory(1.0), current).ok);
  ASSERT_TRUE(controller->Update(current, 0.01).has_command);

  current.positions = {1.0};
  current.velocities = {0.0};
  const TrajectoryStepResult step = controller->Update(current, 0.01);
  EXPECT_EQ(step.status.state, ExecutionState::kSafetyStop);
  EXPECT_TRUE(step.has_command);
  EXPECT_DOUBLE_EQ(step.command.velocities[0], 0.0);
}

TEST(TrajectoryControllerTest, InvalidDtEntersErrorState) {
  auto controller = MakeController();
  RobotState current;
  current.positions = {0.0};
  current.velocities = {0.0};
  ASSERT_TRUE(
      controller->ExecuteTrajectory(MakeTrajectory(1.0), current).ok);
  const TrajectoryStepResult step = controller->Update(current, 0.0);
  EXPECT_FALSE(step.ok);
  EXPECT_EQ(step.status.state, ExecutionState::kError);
}

TEST(TrajectoryControllerTest, ContinuousUpdateReplacesActiveTrajectory) {
  auto controller = MakeController();
  RobotState current;
  current.positions = {0.0};
  current.velocities = {0.0};
  ASSERT_TRUE(
      controller->ExecuteTrajectory(MakeTrajectory(1.0), current).ok);

  for (int cycle = 0; cycle < 20; ++cycle) {
    ASSERT_TRUE(controller->Update(current, 0.01).ok);
  }
  JointTrajectory replacement = MakeTrajectory(1.2);
  replacement.points.front().positions[0] = current.positions[0];
  const ValidationResult update =
      controller->UpdateTrajectory(replacement, current);
  ASSERT_TRUE(update.ok) << update.message;
  const TrajectoryStepResult step = controller->Update(current, 0.01);
  EXPECT_EQ(step.status.state, ExecutionState::kRunning);
  EXPECT_DOUBLE_EQ(controller->GetStatus().duration, 2.0);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
