#include "excavator_visualization/replay_manager.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

JointTrajectory MakeTrajectory() {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a", "joint_b"};
  trajectory.points = {
      {0.0, {0.0, 1.0}, {0.0, 0.0}, {0.0, 0.0}},
      {1.5, {0.5, 1.5}, {0.25, 0.5}, {0.1, 0.2}},
      {3.0, {1.0, 2.0}, {0.0, 0.0}, {0.0, 0.0}}};
  return trajectory;
}

TEST(ReplayManagerTest, TrajectoryRoundTrip) {
  const std::string path = testing::TempDir() + "excavator_trajectory.csv";
  ReplayManager manager;
  std::string error;
  ASSERT_TRUE(manager.SaveTrajectory(path, MakeTrajectory(), &error))
      << error;

  JointTrajectory loaded;
  ASSERT_TRUE(manager.LoadTrajectory(path, &loaded, &error)) << error;
  ASSERT_EQ(loaded.joint_names.size(), 2U);
  ASSERT_EQ(loaded.points.size(), 3U);
  EXPECT_EQ(loaded.joint_names[1], "joint_b");
  EXPECT_DOUBLE_EQ(loaded.points[1].t, 1.5);
  EXPECT_DOUBLE_EQ(loaded.points[1].positions[0], 0.5);
  EXPECT_DOUBLE_EQ(loaded.points[1].velocities[1], 0.5);
  EXPECT_DOUBLE_EQ(loaded.points[1].accelerations[0], 0.1);
}

TEST(ReplayManagerTest, StateRoundTrip) {
  const std::string path = testing::TempDir() + "excavator_states.csv";
  ReplayManager manager;
  std::vector<RobotState> states;
  RobotState first;
  first.t = 0.0;
  first.positions = {0.0, 1.0};
  first.velocities = {0.0, 0.0};
  RobotState second;
  second.t = 0.2;
  second.positions = {0.1, 1.1};
  second.velocities = {0.5, 0.5};
  states.push_back(first);
  states.push_back(second);
  std::string error;
  ASSERT_TRUE(manager.SaveStates(path, {"joint_a", "joint_b"}, states, &error))
      << error;

  std::vector<std::string> names;
  std::vector<RobotState> loaded;
  ASSERT_TRUE(manager.LoadStates(path, &names, &loaded, &error)) << error;
  ASSERT_EQ(names.size(), 2U);
  ASSERT_EQ(loaded.size(), 2U);
  EXPECT_DOUBLE_EQ(loaded[1].t, 0.2);
  EXPECT_DOUBLE_EQ(loaded[1].positions[1], 1.1);
  EXPECT_DOUBLE_EQ(loaded[1].velocities[0], 0.5);
}

TEST(ReplayManagerTest, RejectsMalformedFiles) {
  const std::string path = testing::TempDir() + "excavator_bad.csv";
  {
    std::ofstream file(path);
    file << "wrong_header,joint_a\n0.0,0.0\n";
  }
  ReplayManager manager;
  JointTrajectory trajectory;
  std::string error;
  EXPECT_FALSE(manager.LoadTrajectory(path, &trajectory, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
