#include "kinematics/dh_kinematics_model.hpp"
#include "kinematics/urdf_kinematics_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace excavator {
namespace planning {
namespace {

const std::vector<std::string> kJoints = {
    "swing_joint", "boom_joint", "stick_joint", "bucket_joint"};

const std::vector<std::string> kLinks = {
    "swing_link", "boom_link", "stick_link", "bucket_link", "bucket_tip"};

double QuaternionAngle(const Quaternion& first, const Quaternion& second) {
  Eigen::Quaterniond a(first.w, first.x, first.y, first.z);
  Eigen::Quaterniond b(second.w, second.x, second.y, second.z);
  a.normalize();
  b.normalize();
  Eigen::Quaterniond error = a.inverse() * b;
  error.normalize();
  return 2.0 * std::acos(std::clamp(std::abs(error.w()), 0.0, 1.0));
}

class DhKinematicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(urdf_.InitializeFromFile(TEST_URDF_PATH).ok);
    ASSERT_TRUE(dh_.InitializeFromFile(TEST_DH_PATH).ok);
    limits_ = urdf_.GetPositionLimits();
  }

  JointPositions RandomPositions(std::mt19937* generator) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    JointPositions positions;
    for (const auto& limit : limits_) {
      positions.push_back(limit.first +
                          unit(*generator) * (limit.second - limit.first));
    }
    return positions;
  }

  UrdfKinematicsModel urdf_{kJoints};
  DhKinematicsModel dh_{kJoints};
  std::vector<std::pair<double, double>> limits_;
};

TEST_F(DhKinematicsTest, FkMatchesUrdfForAllLinks) {
  std::mt19937 generator(42);
  for (int sample = 0; sample < 200; ++sample) {
    const JointPositions positions = RandomPositions(&generator);
    for (const std::string& link : kLinks) {
      Pose3 urdf_pose;
      Pose3 dh_pose;
      ASSERT_TRUE(urdf_.GetLinkPose(positions, link, &urdf_pose).ok)
          << link;
      ASSERT_TRUE(dh_.GetLinkPose(positions, link, &dh_pose).ok) << link;
      EXPECT_LT(std::abs(urdf_pose.position.x - dh_pose.position.x), 1.0e-8)
          << link;
      EXPECT_LT(std::abs(urdf_pose.position.y - dh_pose.position.y), 1.0e-8)
          << link;
      EXPECT_LT(std::abs(urdf_pose.position.z - dh_pose.position.z), 1.0e-8)
          << link;
      EXPECT_LT(QuaternionAngle(urdf_pose.orientation, dh_pose.orientation),
                1.0e-8) << link;
    }
  }
}

TEST_F(DhKinematicsTest, LimitsMatchUrdf) {
  EXPECT_EQ(dh_.GetPositionLimits(), urdf_.GetPositionLimits());
  EXPECT_EQ(dh_.GetVelocityLimits(), urdf_.GetVelocityLimits());
  EXPECT_EQ(dh_.joint_names(), urdf_.joint_names());
}

TEST_F(DhKinematicsTest, JacobianMatchesFiniteDifference) {
  std::mt19937 generator(7);
  const JointPositions positions = RandomPositions(&generator);
  Jacobian jacobian;
  ASSERT_TRUE(dh_.GetJacobian(positions, "bucket_tip", &jacobian).ok);
  constexpr double kStep = 1.0e-6;
  for (std::size_t joint = 0; joint < positions.size(); ++joint) {
    JointPositions plus = positions;
    JointPositions minus = positions;
    plus[joint] += kStep;
    minus[joint] -= kStep;
    Pose3 plus_pose;
    Pose3 minus_pose;
    ASSERT_TRUE(dh_.GetLinkPose(plus, "bucket_tip", &plus_pose).ok);
    ASSERT_TRUE(dh_.GetLinkPose(minus, "bucket_tip", &minus_pose).ok);
    for (int row = 0; row < 3; ++row) {
      const double numeric =
          ((row == 0 ? plus_pose.position.x - minus_pose.position.x
                     : row == 1 ? plus_pose.position.y - minus_pose.position.y
                                : plus_pose.position.z - minus_pose.position.z) /
           (2.0 * kStep));
      EXPECT_NEAR(jacobian.rows[static_cast<std::size_t>(row)][joint],
                  numeric, 1.0e-6);
    }
    Eigen::Quaterniond plus_q(plus_pose.orientation.w,
                              plus_pose.orientation.x,
                              plus_pose.orientation.y,
                              plus_pose.orientation.z);
    Eigen::Quaterniond minus_q(minus_pose.orientation.w,
                               minus_pose.orientation.x,
                               minus_pose.orientation.y,
                               minus_pose.orientation.z);
    const Eigen::Vector3d angular =
        (plus_q * minus_q.inverse()).vec() / kStep;
    for (int row = 0; row < 3; ++row) {
      EXPECT_NEAR(jacobian.rows[static_cast<std::size_t>(3 + row)][joint],
                  angular[row], 1.0e-5);
    }
  }
}

TEST_F(DhKinematicsTest, PositionIkRoundTrip) {
  std::mt19937 generator(11);
  for (int sample = 0; sample < 20; ++sample) {
    const JointPositions target_positions = RandomPositions(&generator);
    Pose3 target_pose;
    ASSERT_TRUE(
        urdf_.GetLinkPose(target_positions, "bucket_tip", &target_pose).ok);
    std::uniform_real_distribution<double> offset(-0.1, 0.1);
    JointPositions seed = target_positions;
    for (double& value : seed) {
      value += offset(generator);
    }
    InverseKinematicsOptions options;
    options.orientation_weight = 0.0;
    options.max_iterations = 300;
    options.position_tolerance = 1.0e-5;
    options.time_budget_seconds = 1.0;
    const auto result =
        dh_.SolveIK(target_pose, "bucket_tip", seed, options);
    ASSERT_TRUE(result.success) << result.message;
    Pose3 solved;
    ASSERT_TRUE(
        dh_.GetLinkPose(result.joint_positions, "bucket_tip", &solved).ok);
    EXPECT_LT(std::hypot(std::hypot(solved.position.x - target_pose.position.x,
                                   solved.position.y - target_pose.position.y),
                         solved.position.z - target_pose.position.z),
              1.0e-4);
    for (std::size_t joint = 0; joint < result.joint_positions.size();
         ++joint) {
      EXPECT_GE(result.joint_positions[joint], limits_[joint].first);
      EXPECT_LE(result.joint_positions[joint], limits_[joint].second);
    }
  }
}

TEST_F(DhKinematicsTest, RejectsUnknownLink) {
  const JointPositions positions(4, 0.0);
  Pose3 pose;
  EXPECT_FALSE(dh_.GetLinkPose(positions, "not_a_link", &pose).ok);
}

}  // namespace
}  // namespace planning
}  // namespace excavator
