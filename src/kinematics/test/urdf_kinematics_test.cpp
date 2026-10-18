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

TEST_F(UrdfKinematicsTest, RejectsWrongJointDimension) {
  Pose3 pose;
  const ValidationResult result =
      model_->GetLinkPose({0.0, 0.1}, "tool", &pose);
  EXPECT_FALSE(result.ok);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
