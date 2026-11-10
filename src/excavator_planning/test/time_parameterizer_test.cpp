#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "excavator_common/planner_interfaces.hpp"
#include "excavator_planning/baseline_planners.hpp"
#include "excavator_planning/time_parameterizer_factory.hpp"
#include "excavator_planning/toppra_time_parameterizer.hpp"

namespace excavator {
namespace planning {
namespace {

const std::vector<std::string> kJointNames{
    "swing_joint", "boom_joint", "stick_joint", "bucket_joint"};
const JointPositions kMaxVelocity{0.5, 0.3, 0.4, 0.5};
const JointPositions kMaxAcceleration{1.0, 0.6, 0.8, 1.0};

JointPath MakePath(const std::vector<JointPositions>& points) {
  JointPath path;
  path.joint_names = kJointNames;
  path.points = points;
  return path;
}

TimeParameterizationRequest MakeRequest(
    const std::vector<JointPositions>& points) {
  TimeParameterizationRequest request;
  request.path = MakePath(points);
  request.max_velocity = kMaxVelocity;
  request.max_acceleration = kMaxAcceleration;
  return request;
}

void ExpectStrictlyIncreasingTime(const JointTrajectory& trajectory) {
  ASSERT_FALSE(trajectory.points.empty());
  EXPECT_EQ(trajectory.points.front().t, 0.0);
  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    EXPECT_GT(trajectory.points[index].t, trajectory.points[index - 1].t);
  }
}

}  // namespace

TEST(TimeParameterizerFactoryTest, CreatesSimpleAndToppra) {
  const std::unique_ptr<TimeParameterizer> simple =
      CreateTimeParameterizer("simple");
  ASSERT_NE(simple, nullptr);
  EXPECT_NE(dynamic_cast<SimpleTimeParameterizer*>(simple.get()), nullptr);

  TimeParameterizerOptions options;
  options.toppra_grid_points = 50;
  const std::unique_ptr<TimeParameterizer> toppra =
      CreateTimeParameterizer("toppra", options);
  ASSERT_NE(toppra, nullptr);
  EXPECT_NE(dynamic_cast<ToppraTimeParameterizer*>(toppra.get()), nullptr);

  EXPECT_NE(CreateTimeParameterizer(
                "excavator_planning/ToppraTimeParameterizer"),
            nullptr);
  EXPECT_EQ(CreateTimeParameterizer("not_a_parameterizer"), nullptr);

  const std::vector<std::string> names = TimeParameterizerNames();
  EXPECT_NE(std::find(names.begin(), names.end(), "simple"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "toppra"), names.end());
}

TEST(ToppraTimeParameterizerTest, RespectsLimitsAndEndpoints) {
  const std::vector<JointPositions> waypoints{
      {0.0, -0.35, 0.45, -0.4},
      {0.1, -0.33, 0.43, -0.36},
      {0.2, -0.3, 0.4, -0.3}};
  ToppraTimeParameterizer parameterizer(100);
  const TimeParameterizationResponse response =
      parameterizer.Parameterize(MakeRequest(waypoints));

  ASSERT_TRUE(response.success) << response.message;
  const JointTrajectory& trajectory = response.trajectory;
  ASSERT_GE(trajectory.points.size(), 3U);
  ExpectStrictlyIncreasingTime(trajectory);

  // Clamped boundary conditions + zero path velocity give rest-to-rest.
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    EXPECT_NEAR(trajectory.points.front().positions[joint],
                waypoints.front()[joint], 1e-9);
    EXPECT_NEAR(trajectory.points.back().positions[joint],
                waypoints.back()[joint], 1e-9);
    EXPECT_LT(std::abs(trajectory.points.front().velocities[joint]), 1e-9);
    EXPECT_LT(std::abs(trajectory.points.back().velocities[joint]), 1e-9);
  }

  // Analytic samples produced by TOPP-RA must respect the requested limits.
  for (const JointTrajectoryPoint& point : trajectory.points) {
    for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
      EXPECT_LE(std::abs(point.velocities[joint]),
                kMaxVelocity[joint] * 1.001 + 1e-9);
      EXPECT_LE(std::abs(point.accelerations[joint]),
                kMaxAcceleration[joint] * 1.001 + 1e-9);
    }
  }

  // Use the same strict tolerance as the trajectory controller so a plan that
  // passes here is accepted for execution.
  const ValidationResult limit_validation =
      ValidateJointTrajectoryLimits(trajectory, kMaxVelocity,
                                    kMaxAcceleration);
  EXPECT_TRUE(limit_validation.ok) << limit_validation.message;
}

TEST(ToppraTimeParameterizerTest, HandlesTwoPointPath) {
  const std::vector<JointPositions> waypoints{
      {0.0, -0.35, 0.45, -0.4}, {0.2, -0.3, 0.4, -0.3}};
  ToppraTimeParameterizer parameterizer(200);
  const TimeParameterizationResponse response =
      parameterizer.Parameterize(MakeRequest(waypoints));

  ASSERT_TRUE(response.success) << response.message;
  ExpectStrictlyIncreasingTime(response.trajectory);
  EXPECT_GT(response.trajectory.points.back().t, 0.0);
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    EXPECT_NEAR(response.trajectory.points.back().positions[joint],
                waypoints.back()[joint], 1e-9);
  }
}

TEST(ToppraTimeParameterizerTest, MergesDuplicateWaypoints) {
  const std::vector<JointPositions> waypoints{
      {0.0, -0.35, 0.45, -0.4},
      {0.0, -0.35, 0.45, -0.4},
      {0.2, -0.3, 0.4, -0.3}};
  ToppraTimeParameterizer parameterizer(100);
  const TimeParameterizationResponse response =
      parameterizer.Parameterize(MakeRequest(waypoints));

  ASSERT_TRUE(response.success) << response.message;
  ExpectStrictlyIncreasingTime(response.trajectory);
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    EXPECT_NEAR(response.trajectory.points.back().positions[joint],
                waypoints.back()[joint], 1e-9);
  }
}

TEST(ToppraTimeParameterizerTest, RejectsDegenerateInput) {
  ToppraTimeParameterizer parameterizer(50);

  const std::vector<JointPositions> stationary{
      {0.1, -0.3, 0.4, -0.3}, {0.1, -0.3, 0.4, -0.3}};
  TimeParameterizationResponse response =
      parameterizer.Parameterize(MakeRequest(stationary));
  EXPECT_FALSE(response.success);

  TimeParameterizationRequest bad_limits =
      MakeRequest({{0.0, -0.35, 0.45, -0.4}, {0.2, -0.3, 0.4, -0.3}});
  bad_limits.max_acceleration[1] = 0.0;
  response = parameterizer.Parameterize(bad_limits);
  EXPECT_FALSE(response.success);

  TimeParameterizationRequest mismatched =
      MakeRequest({{0.0, -0.35, 0.45, -0.4}, {0.2, -0.3, 0.4, -0.3}});
  mismatched.max_velocity.pop_back();
  response = parameterizer.Parameterize(mismatched);
  EXPECT_FALSE(response.success);
}

}  // namespace planning
}  // namespace excavator
