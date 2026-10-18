#include "excavator_planning/baseline_planners.hpp"

#include <gtest/gtest.h>

#include <string>

namespace tramcar {
namespace planning {
namespace {

TEST(LinearPathPlannerTest, GeneratesEndpointsAndIntermediateSamples) {
  LinearPathPlanner planner({"joint_a", "joint_b"},
                            {{-1.0, 1.0}, {-2.0, 2.0}}, 0.25);
  PlanningRequest request;
  request.start = {0.0, 0.0};
  request.goal = {0.5, -0.5};
  const PlanningResponse response = planner.Plan(request);

  ASSERT_TRUE(response.success) << response.message;
  ASSERT_GE(response.path.points.size(), 3U);
  EXPECT_DOUBLE_EQ(response.path.points.front()[0], 0.0);
  EXPECT_DOUBLE_EQ(response.path.points.back()[0], 0.5);
  EXPECT_DOUBLE_EQ(response.path.points.back()[1], -0.5);
}

TEST(LinearPathPlannerTest, RejectsDuplicateConfiguredJointNames) {
  LinearPathPlanner planner({"joint_a", "joint_a"},
                            {{-1.0, 1.0}, {-1.0, 1.0}});
  PlanningRequest request;
  request.start = {0.0, 0.0};
  request.goal = {0.5, -0.5};

  const PlanningResponse response = planner.Plan(request);
  EXPECT_FALSE(response.success);
  EXPECT_NE(response.message.find("duplicates"), std::string::npos);
}

TEST(LinearPathPlannerTest, RejectsGoalOutsideJointLimit) {
  LinearPathPlanner planner({"joint_a"}, {{-1.0, 1.0}});
  PlanningRequest request;
  request.start = {0.0};
  request.goal = {2.0};

  EXPECT_FALSE(planner.Plan(request).success);
}

TEST(SimpleTimeParameterizerTest, ProducesStrictlyIncreasingTime) {
  JointPath path;
  path.joint_names = {"joint_a"};
  path.points = {{0.0}, {0.5}, {1.0}};
  TimeParameterizationRequest request;
  request.path = path;
  request.max_velocity = {1.0};
  request.max_acceleration = {2.0};

  const TimeParameterizationResponse response =
      SimpleTimeParameterizer().Parameterize(request);
  ASSERT_TRUE(response.success) << response.message;
  ASSERT_EQ(response.trajectory.points.size(), 3U);
  EXPECT_DOUBLE_EQ(response.trajectory.points.front().t, 0.0);
  EXPECT_LT(response.trajectory.points[0].t, response.trajectory.points[1].t);
  EXPECT_LT(response.trajectory.points[1].t, response.trajectory.points[2].t);
}

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

}  // namespace
}  // namespace planning
}  // namespace
