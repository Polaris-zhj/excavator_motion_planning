#include "excavator_collision/fcl_collision_checker.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#include "excavator_interfaces/msg/planning_scene.hpp"
#include "kinematics/dh_kinematics_model.hpp"

namespace excavator {
namespace planning {
namespace {

std::string ReadFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return {};
  }
  return std::string((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
}

// Performance check for the full excavator DH + YAML collision model.
TEST(FclCollisionCheckerTest, BenchmarksFullExcavatorModel) {
  const std::string dh_yaml = ReadFile(TEST_DH_PATH);
  const std::string collision_yaml = ReadFile(TEST_COLLISION_YAML_PATH);
  ASSERT_FALSE(dh_yaml.empty()) << TEST_DH_PATH;
  ASSERT_FALSE(collision_yaml.empty()) << TEST_COLLISION_YAML_PATH;

  DhKinematicsModel kinematics(
      {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"});
  const auto kinematics_initialized =
      kinematics.InitializeFromString(dh_yaml);
  ASSERT_TRUE(kinematics_initialized.ok) << kinematics_initialized.message;

  FclCollisionChecker checker;
  CollisionModelConfig config;
  config.joint_names = kinematics.joint_names();
  config.model_source = collision_yaml;
  config.model_format = "yaml";
  config.kinematics_reference_frame = "base_link";
  const auto initialized = checker.Initialize(config, &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;
  const std::vector<std::pair<std::string, std::string>> allowed_pairs = {
      {"base_link", "boom_link"},
      {"base_link", "stick_link"},
      {"base_link", "bucket_link"},
      {"swing_link", "stick_link"},
      {"swing_link", "bucket_link"}};
  for (const auto& pair : allowed_pairs) {
    checker.SetAllowedCollision(pair.first, pair.second, true);
  }

  const ObstacleList obstacles;
  const JointPositions home = {0.0, -0.35, 0.45, -0.40};
  CollisionCheckResult result = checker.CheckCollision(home, obstacles);
  ASSERT_TRUE(result.valid) << result.message;
  ASSERT_FALSE(result.collision) << result.message;

  constexpr int kIterations = 200;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIterations; ++i) {
    result = checker.CheckCollision(home, obstacles);
  }
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  std::cout << "CheckCollision average: " << 1000.0 * seconds / kIterations
            << " ms" << std::endl;
  EXPECT_TRUE(result.valid);
}

}  // namespace
}  // namespace planning
}  // namespace excavator
