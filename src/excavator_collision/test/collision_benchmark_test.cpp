#include "excavator_collision/collision_checker.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace tramcar {
namespace planning {
namespace {

// Optional performance check for the full excavator model. Run manually:
//   EXCAVATOR_URDF=/tmp/excavator.urdf ./collision_benchmark_test
TEST(UrdfFclCollisionCheckerTest, BenchmarksFullExcavatorModel) {
  const char* path = std::getenv("EXCAVATOR_URDF");
  if (path == nullptr) {
    GTEST_SKIP() << "set EXCAVATOR_URDF to benchmark the full model";
  }
  std::ifstream file(path);
  ASSERT_TRUE(file.good()) << path;
  const std::string urdf((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

  UrdfFclCollisionChecker checker;
  const auto initialized = checker.Initialize(
      urdf, {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"});
  ASSERT_TRUE(initialized.ok) << initialized.message;
  checker.SetAllowedCollisionPairs(
      {{"base_link", "boom_link"},
       {"base_link", "stick_link"},
       {"base_link", "bucket_link"},
       {"swing_link", "stick_link"},
       {"swing_link", "bucket_link"}});

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "world";
  const JointPositions home = {0.0, -0.35, 0.45, -0.40};
  CollisionCheckResult result = checker.CheckState(home, scene);
  ASSERT_TRUE(result.valid) << result.message;
  ASSERT_FALSE(result.collision) << result.message;

  constexpr int kIterations = 200;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIterations; ++i) {
    result = checker.CheckState(home, scene);
  }
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  std::cout << "CheckState average: " << 1000.0 * seconds / kIterations
            << " ms" << std::endl;
  EXPECT_TRUE(result.valid);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
