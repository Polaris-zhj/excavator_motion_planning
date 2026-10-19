#include "excavator_planner_rrtstar/rrt_star_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace tramcar {
namespace planning {
namespace {

PlannerSettings MakeSettings() {
  PlannerSettings settings;
  settings.joint_names = {"joint_a", "joint_b"};
  settings.joint_limits = {{-1.0, 1.0}, {-1.0, 1.0}};
  settings.max_joint_step = 0.02;
  settings.collision_check_resolution = 0.005;
  settings.max_iterations = 6000;
  settings.step_size = 0.2;
  settings.goal_bias = 0.15;
  settings.rewire_radius = 0.5;
  settings.goal_tolerance = 0.12;
  settings.random_seed = 7;
  return settings;
}

PlanningRequest MakeRequest(const JointPositions& start,
                            const JointPositions& goal) {
  PlanningRequest request;
  request.start = start;
  request.goal = goal;
  request.timeout_seconds = 10.0;
  return request;
}

void ExpectPathEdgesValid(
    const JointPath& path,
    const std::function<bool(const JointPositions&)>& valid) {
  for (std::size_t segment = 1; segment < path.points.size(); ++segment) {
    const JointPositions& start = path.points[segment - 1];
    const JointPositions& goal = path.points[segment];
    double distance = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint) {
      distance = std::max(distance, std::abs(goal[joint] - start[joint]));
    }
    const std::size_t steps =
        std::max<std::size_t>(1, static_cast<std::size_t>(
                                     std::ceil(distance / 0.005)));
    for (std::size_t step = 0; step <= steps; ++step) {
      const double ratio =
          static_cast<double>(step) / static_cast<double>(steps);
      JointPositions sample(start.size(), 0.0);
      for (std::size_t joint = 0; joint < start.size(); ++joint) {
        sample[joint] =
            start[joint] + ratio * (goal[joint] - start[joint]);
      }
      EXPECT_TRUE(valid(sample));
    }
  }
}

TEST(RrtStarPlannerTest, FindsPathInOpenJointSpace) {
  RrtStarPlanner planner;
  planner.Configure(MakeSettings());
  const PlanningRequest request =
      MakeRequest({-0.8, -0.8}, {0.8, 0.8});
  const PlanningResponse response = planner.Plan(request);

  ASSERT_TRUE(response.success) << response.message;
  ASSERT_GE(response.path.points.size(), 2U);
  EXPECT_EQ(response.path.points.front(), request.start);
  EXPECT_EQ(response.path.points.back(), request.goal);
  for (const JointPositions& point : response.path.points) {
    ASSERT_EQ(point.size(), 2U);
    EXPECT_GE(point[0], -1.0);
    EXPECT_LE(point[0], 1.0);
    EXPECT_GE(point[1], -1.0);
    EXPECT_LE(point[1], 1.0);
  }
}

TEST(RrtStarPlannerTest, AvoidsBlockedRegion) {
  RrtStarPlanner planner;
  planner.Configure(MakeSettings());
  PlanningRequest request = MakeRequest({-0.8, 0.0}, {0.8, 0.0});
  const auto valid = [](const JointPositions& positions, std::string*) {
    return !(std::abs(positions[0]) < 0.12 &&
             std::abs(positions[1]) < 0.4);
  };
  request.state_validity_check = valid;
  const PlanningResponse response = planner.Plan(request);

  ASSERT_TRUE(response.success) << response.message;
  ExpectPathEdgesValid(response.path,
                       [&valid](const JointPositions& positions) {
                         std::string message;
                         return valid(positions, &message);
                       });
  bool has_detour = false;
  for (const JointPositions& point : response.path.points) {
    if (std::abs(point[1]) > 0.12) {
      has_detour = true;
    }
  }
  EXPECT_TRUE(has_detour);
}

TEST(RrtStarPlannerTest, IsDeterministicForAFixedSeed) {
  RrtStarPlanner planner;
  planner.Configure(MakeSettings());
  const PlanningRequest request =
      MakeRequest({-0.8, -0.8}, {0.8, 0.8});
  const PlanningResponse first = planner.Plan(request);
  const PlanningResponse second = planner.Plan(request);

  ASSERT_TRUE(first.success) << first.message;
  ASSERT_TRUE(second.success) << second.message;
  ASSERT_EQ(first.path.points.size(), second.path.points.size());
  for (std::size_t index = 0; index < first.path.points.size(); ++index) {
    EXPECT_EQ(first.path.points[index], second.path.points[index]);
  }
}

TEST(RrtStarPlannerTest, ReportsInvalidStartAndGoal) {
  RrtStarPlanner planner;
  planner.Configure(MakeSettings());
  PlanningRequest request = MakeRequest({0.0, 0.0}, {0.5, 0.5});
  request.state_validity_check =
      [](const JointPositions&, std::string*) { return false; };
  const PlanningResponse start_response = planner.Plan(request);
  EXPECT_FALSE(start_response.success);
  EXPECT_NE(start_response.message.find("start state is invalid"),
            std::string::npos);

  PlanningRequest goal_request = MakeRequest({0.0, 0.0}, {0.5, 0.5});
  goal_request.state_validity_check =
      [](const JointPositions& positions, std::string*) {
        return positions[0] < 0.25;
      };
  const PlanningResponse goal_response = planner.Plan(goal_request);
  EXPECT_FALSE(goal_response.success);
  EXPECT_NE(goal_response.message.find("goal state is invalid"),
            std::string::npos);
}

TEST(RrtStarPlannerTest, HonoursCancellationHook) {
  RrtStarPlanner planner;
  planner.Configure(MakeSettings());
  PlanningRequest request = MakeRequest({-0.8, -0.8}, {0.8, 0.8});
  request.should_continue = []() { return false; };
  const PlanningResponse response = planner.Plan(request);
  EXPECT_FALSE(response.success);
  EXPECT_NE(response.message.find("canceled or timed out"), std::string::npos);
}

TEST(RrtStarPlannerTest, RejectsInvalidConfiguration) {
  RrtStarPlanner planner;
  PlannerSettings settings = MakeSettings();
  settings.step_size = 0.0;
  planner.Configure(settings);
  const PlanningResponse response =
      planner.Plan(MakeRequest({0.0, 0.0}, {0.5, 0.5}));
  EXPECT_FALSE(response.success);
  EXPECT_NE(response.message.find("not configured"), std::string::npos);
}

TEST(RrtStarPlannerTest, HandlesStartEqualsGoal) {
  RrtStarPlanner planner;
  planner.Configure(MakeSettings());
  const PlanningRequest request = MakeRequest({0.2, 0.3}, {0.2, 0.3});
  const PlanningResponse response = planner.Plan(request);
  ASSERT_TRUE(response.success) << response.message;
  EXPECT_GE(response.path.points.size(), 2U);
  EXPECT_EQ(response.path.points.front(), request.start);
  EXPECT_EQ(response.path.points.back(), request.goal);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
