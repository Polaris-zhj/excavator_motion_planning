#include "kinematics/urdf_kinematics_model.hpp"

#include <cmath>
#include <string>

#include <gtest/gtest.h>

namespace tramcar {
namespace planning {
namespace {

class UrdfKinematicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = std::make_unique<UrdfKinematicsModel>(
        std::vector<std::string>{"joint_a"});
    const ValidationResult result = model_->InitializeFromFile(TEST_URDF_PATH);
    ASSERT_TRUE(result.ok) << result.message;
  }

  std::unique_ptr<UrdfKinematicsModel> model_;
};

TEST_F(UrdfKinematicsTest, ComputesForwardKinematics) {
  Pose3 pose;
  const ValidationResult result =
      model_->GetLinkPose({0.0}, "tool", &pose);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_NEAR(pose.position.x, 2.0, 1.0e-8);
  EXPECT_NEAR(pose.position.y, 0.0, 1.0e-8);
  EXPECT_NEAR(pose.position.z, 0.0, 1.0e-8);
}

TEST_F(UrdfKinematicsTest, ComputesGeometricJacobian) {
  Jacobian jacobian;
  const ValidationResult result =
      model_->GetJacobian({0.0}, "tool", &jacobian);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(jacobian.rows.size(), 6U);
  ASSERT_EQ(jacobian.rows[0].size(), 1U);
  EXPECT_NEAR(jacobian.rows[0][0], 0.0, 1.0e-8);
  EXPECT_NEAR(jacobian.rows[1][0], 1.0, 1.0e-8);
  EXPECT_NEAR(jacobian.rows[5][0], 1.0, 1.0e-8);
}

TEST_F(UrdfKinematicsTest, SolvesConsistentInverseKinematicsTarget) {
  constexpr double kTargetAngle = 0.5;
  Pose3 target;
  target.position = {1.0 + std::cos(kTargetAngle), std::sin(kTargetAngle), 0.0};
  target.orientation = {0.0, 0.0, std::sin(kTargetAngle / 2.0),
                        std::cos(kTargetAngle / 2.0)};

  InverseKinematicsOptions options;
  options.max_iterations = 100;
  options.position_tolerance = 1.0e-6;
  options.orientation_tolerance = 1.0e-6;
  options.damping = 1.0e-4;
  const auto result = model_->SolveIK(target, "tool", {0.2}, options);

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.joint_positions.size(), 1U);
  EXPECT_NEAR(result.joint_positions[0], kTargetAngle, 1.0e-4);
}

TEST_F(UrdfKinematicsTest, RejectsWrongJointDimension) {
  Pose3 pose;
  const ValidationResult result =
      model_->GetLinkPose({0.0, 0.1}, "tool", &pose);
  EXPECT_FALSE(result.ok);
}

TEST_F(UrdfKinematicsTest, PositionOnlyIkIgnoresOrientation) {
  Pose3 target;
  target.position.x = 1.5;
  target.position.y = 0.8660254;
  target.position.z = 0.0;
  target.orientation.w = 1.0;  // deliberately not the FK orientation
  InverseKinematicsOptions options;
  options.max_iterations = 200;
  options.orientation_weight = 0.0;
  options.orientation_tolerance = 1.0e9;
  const auto result = model_->SolveIK(target, "tool", {0.2}, options);
  ASSERT_TRUE(result.success) << result.message;
  Pose3 reached;
  ASSERT_TRUE(model_->GetLinkPose(result.joint_positions, "tool", &reached).ok);
  EXPECT_NEAR(reached.position.x, target.position.x, 1.0e-4);
  EXPECT_NEAR(reached.position.y, target.position.y, 1.0e-4);
  EXPECT_NEAR(reached.position.z, target.position.z, 1.0e-4);
}

TEST_F(UrdfKinematicsTest, ReportsIkTimeBudgetExceeded) {
  Pose3 target;
  target.position.x = 2.0;
  target.orientation.w = 1.0;
  InverseKinematicsOptions options;
  options.max_iterations = 100000;
  options.time_budget_seconds = 1.0e-12;
  const auto result = model_->SolveIK(target, "tool", {0.2}, options);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("time budget"), std::string::npos);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
