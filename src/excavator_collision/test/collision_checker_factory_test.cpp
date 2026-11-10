#include "excavator_collision/collision_checker_factory.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include "excavator_collision/fcl_collision_checker.hpp"
#include "excavator_collision/collision_checker_interface.hpp"

namespace excavator {
namespace planning {
namespace {

class MockCollisionChecker final : public CollisionChecker {
 public:
  ValidationResult Initialize(const CollisionModelConfig&,
                               const KinematicsModel*) override {
    return {true, ""};
  }
  void SetAllowedCollision(const std::string&, const std::string&,
                           bool) override {}
  CollisionCheckResult CheckCollision(
      const JointPositions&, const ObstacleList&,
      const CollisionQueryOptions&) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionCheckResult CheckSelfCollision(
      const JointPositions&) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionCheckResult CheckEnvironmentCollision(
      const JointPositions&, const ObstacleList&,
      const CollisionQueryOptions&) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionCheckResult CheckPathCollision(
      const JointPath&, const ObstacleList&, double,
      const CollisionQueryOptions&) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionCheckResult CheckPathCollision(
      const JointPath&, const Obstacle&, double,
      const CollisionQueryOptions&) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionCheckResult CheckTrajectory(
      const JointTrajectory&, const ObstacleList&, double,
      const CollisionQueryOptions&) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionDistanceResult ComputeDistance(
      const JointPositions&, const Obstacle&,
      const CollisionQueryOptions&) const override {
    CollisionDistanceResult result;
    result.valid = true;
    result.distance = 0.0;
    return result;
  }
};

TEST(CollisionCheckerFactoryTest, RegistersFclBackendByDefault) {
  auto checker = CollisionCheckerFactory::Instance().Create("fcl");
  ASSERT_NE(checker, nullptr);
  const auto types = CollisionCheckerFactory::Instance().AvailableTypes();
  EXPECT_NE(std::find(types.begin(), types.end(), "fcl"), types.end());
}

TEST(CollisionCheckerFactoryTest, UnknownBackendReturnsNull) {
  EXPECT_EQ(CollisionCheckerFactory::Instance().Create("not_a_backend"),
            nullptr);
}

TEST(CollisionCheckerFactoryTest, RuntimeRegistrationWorks) {
  CollisionCheckerFactory::Instance().Register(
      "mock", []() -> std::unique_ptr<CollisionChecker> {
        return std::make_unique<MockCollisionChecker>();
      });
  auto checker = CollisionCheckerFactory::Instance().Create("mock");
  ASSERT_NE(checker, nullptr);
  EXPECT_TRUE(checker->Initialize(CollisionModelConfig{}).ok);
}

}  // namespace
}  // namespace planning
}  // namespace excavator
