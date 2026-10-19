#include "kinematics/workspace_checker.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace tramcar {
namespace planning {
namespace {

Vector3 MakeVector(double x, double y, double z) {
  Vector3 value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

WorkspaceBox MakeBox(double min, double max) {
  return WorkspaceBox{MakeVector(min, min, min), MakeVector(max, max, max)};
}

Pose3 MakePose(double x, double y, double z) {
  Pose3 pose;
  pose.position = MakeVector(x, y, z);
  return pose;
}

TEST(WorkspaceCheckerTest, ChecksAllowedBoxAndDistance) {
  WorkspaceConfig config;
  config.allowed_regions.push_back(MakeBox(-1.0, 1.0));
  const WorkspaceChecker checker(config);
  ASSERT_TRUE(checker.Validate().ok);

  const WorkspaceCheckResult center = checker.Check(MakePose(0.0, 0.0, 0.0));
  ASSERT_TRUE(center.valid);
  EXPECT_TRUE(center.inside);
  EXPECT_DOUBLE_EQ(center.distance_to_boundary, 1.0);

  const WorkspaceCheckResult outside =
      checker.Check(MakePose(1.2, 0.0, 0.0));
  ASSERT_TRUE(outside.valid);
  EXPECT_FALSE(outside.inside);
  EXPECT_LT(outside.distance_to_boundary, 0.0);
}

TEST(WorkspaceCheckerTest, AppliesMargin) {
  WorkspaceConfig config;
  config.allowed_regions.push_back(MakeBox(-1.0, 1.0));
  config.margin = 0.1;
  const WorkspaceChecker checker(config);
  const WorkspaceCheckResult near_edge =
      checker.Check(MakePose(0.95, 0.0, 0.0));
  EXPECT_FALSE(near_edge.inside);
}

TEST(WorkspaceCheckerTest, RejectsForbiddenRegions) {
  WorkspaceConfig config;
  config.allowed_regions.push_back(MakeBox(-2.0, 2.0));
  config.forbidden_regions.push_back(MakeBox(-0.2, 0.2));
  config.margin = 0.05;
  const WorkspaceChecker checker(config);
  const WorkspaceCheckResult result =
      checker.Check(MakePose(0.0, 0.0, 0.0));
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.inside);
  EXPECT_NE(result.message.find("forbidden"), std::string::npos);
}

TEST(WorkspaceCheckerTest, EmptyAllowedListIsUnbounded) {
  WorkspaceConfig config;
  const WorkspaceChecker checker(config);
  const WorkspaceCheckResult result =
      checker.Check(MakePose(100.0, 100.0, 100.0));
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.inside);
  EXPECT_EQ(result.distance_to_boundary,
            std::numeric_limits<double>::infinity());
}

TEST(WorkspaceCheckerTest, RejectsInvalidConfiguration) {
  WorkspaceConfig config;
  WorkspaceBox invalid;
  invalid.minimum = MakeVector(1.0, 0.0, 0.0);
  invalid.maximum = MakeVector(-1.0, 1.0, 1.0);
  config.allowed_regions.push_back(invalid);
  const WorkspaceChecker checker(config);
  EXPECT_FALSE(checker.Validate().ok);
  EXPECT_FALSE(checker.Check(MakePose(0.0, 0.0, 0.0)).valid);

  WorkspaceConfig negative_margin;
  negative_margin.margin = -0.1;
  EXPECT_FALSE(WorkspaceChecker(negative_margin).Validate().ok);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
