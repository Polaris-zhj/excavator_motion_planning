#include "excavator_planning/baseline_planners.hpp"
#include "excavator_planning/path_post_processor.hpp"
#include "excavator_planning/planner_factory.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

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

TEST(SimpleTimeParameterizerTest, RespectsDiscreteVelocityAndAccelerationLimits) {
  JointPath path;
  path.joint_names = {"joint_a"};
  path.points = {{0.0}, {1.0}, {0.2}, {1.2}};

  TimeParameterizationRequest request;
  request.path = path;
  request.max_velocity = {0.5};
  request.max_acceleration = {0.2};

  const TimeParameterizationResponse response =
      SimpleTimeParameterizer().Parameterize(request);
  ASSERT_TRUE(response.success) << response.message;
  const JointTrajectory& trajectory = response.trajectory;

  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    const double dt = trajectory.points[index].t -
                      trajectory.points[index - 1].t;
    const double velocity =
        (trajectory.points[index].positions[0] -
         trajectory.points[index - 1].positions[0]) / dt;
    EXPECT_LE(std::abs(velocity), request.max_velocity[0] + 1.0e-9);
  }

  for (std::size_t index = 1; index + 1 < trajectory.points.size(); ++index) {
    const double previous_dt = trajectory.points[index].t -
                               trajectory.points[index - 1].t;
    const double next_dt = trajectory.points[index + 1].t -
                           trajectory.points[index].t;
    const double previous_velocity =
        (trajectory.points[index].positions[0] -
         trajectory.points[index - 1].positions[0]) / previous_dt;
    const double next_velocity =
        (trajectory.points[index + 1].positions[0] -
         trajectory.points[index].positions[0]) / next_dt;
    const double acceleration =
        2.0 * (next_velocity - previous_velocity) /
        (previous_dt + next_dt);
    EXPECT_LE(std::abs(acceleration),
              request.max_acceleration[0] + 1.0e-9);
  }
}

TEST(SimpleTimeParameterizerTest, ProducesRestToRestProfileForTwoPointPath) {
  JointPath path;
  path.joint_names = {"joint_a"};
  path.points = {{0.0}, {0.4}};

  TimeParameterizationRequest request;
  request.path = path;
  request.max_velocity = {1.0};
  request.max_acceleration = {2.0};

  const TimeParameterizationResponse response =
      SimpleTimeParameterizer().Parameterize(request);
  ASSERT_TRUE(response.success) << response.message;
  const JointTrajectory& trajectory = response.trajectory;
  ASSERT_GE(trajectory.points.size(), 5U);
  EXPECT_NEAR(trajectory.points.front().positions[0], 0.0, 1.0e-9);
  EXPECT_NEAR(trajectory.points.back().positions[0], 0.4, 1.0e-9);

  // A simplified two-point path is expanded into a quintic rest-to-rest
  // profile, so the reference velocity is nonzero while the joints move and
  // both endpoints start/finish at rest.
  EXPECT_NEAR(trajectory.points.front().velocities[0], 0.0, 1.0e-9);
  EXPECT_NEAR(trajectory.points.back().velocities[0], 0.0, 1.0e-9);
  double peak_velocity = 0.0;
  for (const JointTrajectoryPoint& point : trajectory.points) {
    peak_velocity = std::max(peak_velocity, std::abs(point.velocities[0]));
    EXPECT_LE(std::abs(point.velocities[0]),
              request.max_velocity[0] + 1.0e-9);
    EXPECT_LE(std::abs(point.accelerations[0]),
              request.max_acceleration[0] + 1.0e-9);
  }
  EXPECT_GT(peak_velocity, 0.0);
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

class MockPlanner final : public PathPlanner {
 public:
  PlanningResponse Plan(const PlanningRequest&) override {
    PlanningResponse response;
    response.success = true;
    response.message = "mock";
    return response;
  }
};

TEST(LinearPathPlannerTest, FindsFiniteDetourAroundBlockedRegion) {
  LinearPathPlanner planner({"joint_a", "joint_b"},
                            {{-2.0, 2.0}, {-2.0, 2.0}}, 0.05);
  PlanningRequest request;
  request.start = {0.0, 0.0};
  request.goal = {1.0, 0.0};
  request.state_validity_check =
      [](const JointPositions& positions, std::string*) {
        const bool blocked =
            std::abs(positions[0] - 0.5) < 0.15 &&
            std::abs(positions[1]) < 0.1;
        return !blocked;
      };

  const PlanningResponse response = planner.Plan(request);
  ASSERT_TRUE(response.success) << response.message;
  ASSERT_GT(response.path.points.size(), 2U);
  bool has_detour = false;
  for (const JointPositions& point : response.path.points) {
    if (std::abs(point[1]) > 0.3) {
      has_detour = true;
    }
    EXPECT_FALSE(std::abs(point[0] - 0.5) < 0.15 &&
                 std::abs(point[1]) < 0.1);
  }
  EXPECT_TRUE(has_detour);
}

TEST(LinearPathPlannerTest, ReportsFailureWhenNoDetourIsValid) {
  LinearPathPlanner planner({"joint_a", "joint_b"},
                            {{-2.0, 2.0}, {-2.0, 2.0}}, 0.05);
  PlanningRequest request;
  request.start = {0.0, 0.0};
  request.goal = {1.0, 0.0};
  request.state_validity_check =
      [](const JointPositions&, std::string*) { return false; };

  const PlanningResponse response = planner.Plan(request);
  EXPECT_FALSE(response.success);
  EXPECT_NE(response.message.find("detour"), std::string::npos);
}

TEST(LinearPathPlannerTest, HonorsCancellationHook) {
  LinearPathPlanner planner({"joint_a", "joint_b"},
                            {{-2.0, 2.0}, {-2.0, 2.0}}, 0.05);
  PlanningRequest request;
  request.start = {0.0, 0.0};
  request.goal = {1.0, 0.0};
  request.state_validity_check =
      [](const JointPositions&, std::string*) { return true; };
  request.should_continue = []() { return false; };

  const PlanningResponse response = planner.Plan(request);
  EXPECT_FALSE(response.success);
  EXPECT_NE(response.message.find("canceled"), std::string::npos);
}

TEST(PlannerFactoryTest, CreatesDefaultAndRegisteredPlanners) {
  PlannerSettings settings;
  settings.joint_names = {"joint_a"};
  settings.joint_limits = {{-1.0, 1.0}};
  PlannerFactory factory = PlannerFactory::CreateDefault();
  EXPECT_TRUE(factory.Contains("linear"));
  const std::vector<std::string> names = factory.Names();
  EXPECT_NE(std::find(names.begin(), names.end(), "linear"), names.end());
  EXPECT_NE(factory.Create("linear", settings), nullptr);
  EXPECT_EQ(factory.Create("missing", settings), nullptr);

  factory.Register("mock", [](const PlannerSettings&) {
    return std::make_unique<MockPlanner>();
  });
  auto mock = factory.Create("mock", settings);
  ASSERT_NE(mock, nullptr);
  EXPECT_EQ(mock->Plan(PlanningRequest{}).message, "mock");
}

TEST(PathPostProcessorTest, SimplifiesUsingSegmentValidity) {
  JointPath path;
  path.joint_names = {"joint_a"};
  path.points = {{0.0}, {0.25}, {0.5}, {0.75}, {1.0}};
  PathPostProcessor processor;
  const auto validator = [](const JointPositions& start,
                            const JointPositions& goal, std::string*) {
    return !(start[0] < 0.3 && goal[0] > 0.7);
  };
  JointPath simplified;
  ASSERT_TRUE(processor.Simplify(path, validator, &simplified).ok);
  ASSERT_EQ(simplified.points.size(), 3U);
  EXPECT_DOUBLE_EQ(simplified.points.front()[0], 0.0);
  EXPECT_DOUBLE_EQ(simplified.points[1][0], 0.5);
  EXPECT_DOUBLE_EQ(simplified.points.back()[0], 1.0);

  JointPath fully_simplified;
  ASSERT_TRUE(processor.Simplify(
                  path, PathPostProcessor::SegmentValidityCheck{},
                  &fully_simplified)
                  .ok);
  EXPECT_EQ(fully_simplified.points.size(), 2U);
}

TEST(PathPostProcessorTest, SmoothsInteriorWaypoints) {
  JointPath path;
  path.joint_names = {"joint_a", "joint_b"};
  path.points = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}};
  PathPostProcessor processor;
  JointPath smoothed;
  const auto result = processor.Smooth(
      path, 0.5, 1, PathPostProcessor::SegmentValidityCheck{}, &smoothed);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(smoothed.points.size(), 3U);
  EXPECT_DOUBLE_EQ(smoothed.points.front()[1], 0.0);
  EXPECT_DOUBLE_EQ(smoothed.points.back()[1], 0.0);
  EXPECT_LT(smoothed.points[1][1], 1.0);
  EXPECT_GT(smoothed.points[1][1], 0.0);
}

TEST(PathPostProcessorTest, KeepsLastValidPathWhenSmoothingIsInvalid) {
  JointPath path;
  path.joint_names = {"joint_a", "joint_b"};
  path.points = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}};
  PathPostProcessor processor;
  const auto validator = [](const JointPositions& start,
                            const JointPositions& goal,
                            std::string*) {
    return start[1] == 0.0 && goal[1] == 0.0;
  };
  JointPath smoothed;
  const auto result = processor.Smooth(path, 0.5, 2, validator, &smoothed);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_FALSE(result.message.empty());
  EXPECT_EQ(smoothed.points, path.points);
}

}  // namespace
}  // namespace planning
}  // namespace
