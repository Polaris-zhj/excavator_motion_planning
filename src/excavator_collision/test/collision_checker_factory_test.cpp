#include "excavator_collision/collision_checker_factory.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include "excavator_collision/collision_checker.hpp"
#include "excavator_collision/collision_checker_interface.hpp"

namespace tramcar {
namespace planning {
namespace {

class MockCollisionChecker final : public CollisionChecker {
 public:
  ValidationResult Initialize(const CollisionModelConfig&,
                               const KinematicsModel*) override {
    return {true, ""};
  }
  void SetAllowedCollisionPairs(
      const std::vector<std::pair<std::string, std::string>>&) override {}
  CollisionCheckResult CheckState(
      const JointPositions&,
      const excavator_interfaces::msg::PlanningScene&) const override {
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
      const JointPositions&,
      const excavator_interfaces::msg::PlanningScene&) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionCheckResult CheckPath(
      const JointPath&,
      const excavator_interfaces::msg::PlanningScene&,
      double) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionCheckResult CheckPathCollisionWithObstacle(
      const JointPath&,
      const excavator_interfaces::msg::PlanningScene&,
      const std::string&, double) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionCheckResult CheckTrajectory(
      const JointTrajectory&,
      const excavator_interfaces::msg::PlanningScene&,
      double) const override {
    CollisionCheckResult result;
    result.valid = true;
    return result;
  }
  CollisionDistanceResult ComputeDistanceToObstacle(
      const JointPositions&,
      const excavator_interfaces::msg::PlanningScene&,
      const std::string&) const override {
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
}  // namespace tramcar
