#include "excavator_common/joint_trajectory.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace tramcar {
namespace planning {
namespace {

TEST(JointPathValidationTest, AcceptsConsistentPath) {
  JointPath path;
  path.joint_names = {"joint_a", "joint_b"};
  path.points = {{0.0, 1.0}, {0.5, 1.5}};

  EXPECT_TRUE(ValidateJointPath(path).ok);
}

TEST(JointPathValidationTest, RejectsDuplicateJointNames) {
  JointPath path;
  path.joint_names = {"joint_a", "joint_a"};
  path.points = {{0.0, 1.0}};

  EXPECT_FALSE(ValidateJointPath(path).ok);
}

TEST(JointPathValidationTest, RejectsWrongJointCount) {
  JointPath path;
  path.joint_names = {"joint_a", "joint_b"};
  path.points = {{0.0}};

  EXPECT_FALSE(ValidateJointPath(path).ok);
}

TEST(JointTrajectoryValidationTest, AcceptsIncreasingTimes) {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory.points = {{0.0, {0.0}, {}, {}}, {1.0, {1.0}, {}, {}}};

  EXPECT_TRUE(ValidateJointTrajectory(trajectory).ok);
}

TEST(JointTrajectoryValidationTest, RejectsDuplicateJointNames) {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a", "joint_a"};
  trajectory.points = {{0.0, {0.0, 1.0}, {}, {}}};

  EXPECT_FALSE(ValidateJointTrajectory(trajectory).ok);
}

TEST(JointTrajectoryValidationTest, RejectsNonIncreasingTimes) {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory.points = {{0.0, {0.0}, {}, {}}, {0.0, {1.0}, {}, {}}};

  EXPECT_FALSE(ValidateJointTrajectory(trajectory).ok);
}

TEST(JointTrajectoryValidationTest, RejectsNonFinitePosition) {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory.points = {
      {0.0, {std::numeric_limits<double>::quiet_NaN()}, {}, {}}};

  EXPECT_FALSE(ValidateJointTrajectory(trajectory).ok);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
