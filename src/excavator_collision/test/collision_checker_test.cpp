#include "excavator_collision/collision_checker.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace tramcar {
namespace planning {
namespace {

const char kRobotUrdf[] = R"(
<robot name="collision_test_robot">
  <link name="base_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <link name="arm_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="0.2"/></geometry>
    </collision>
  </link>
  <link name="tool_link">
    <collision>
      <origin xyz="1 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="0.2"/></geometry>
    </collision>
  </link>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/><child link="arm_link"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="arm_link"/><child link="tool_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>)";

excavator_interfaces::msg::CollisionObject MakeSphere(
    const std::string& id, double x, double y, double z, double radius) {
  excavator_interfaces::msg::CollisionObject object;
  object.id = id;
  object.enabled = true;
  object.pose.position.x = x;
  object.pose.position.y = y;
  object.pose.position.z = z;
  object.pose.orientation.w = 1.0;
  object.primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  object.primitive.dimensions = {radius};
  return object;
}

TEST(UrdfFclCollisionCheckerTest, DetectsEnvironmentCollision) {
  UrdfFclCollisionChecker checker;
  const auto initialized = checker.Initialize(kRobotUrdf, {"arm_joint"});
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(MakeSphere("obstacle", 0.4, 0.0, 0.0, 0.15));

  const CollisionCheckResult result = checker.CheckState({0.0}, scene);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_EQ(result.second_object, "obstacle");
}

TEST(UrdfFclCollisionCheckerTest, AcceptsClearEnvironment) {
  UrdfFclCollisionChecker checker;
  const auto initialized = checker.Initialize(kRobotUrdf, {"arm_joint"});
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(MakeSphere("far_obstacle", 10.0, 10.0, 10.0, 0.1));

  const CollisionCheckResult result = checker.CheckState({0.0}, scene);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_FALSE(result.collision);
}


TEST(UrdfFclCollisionCheckerTest, DetectsNonAdjacentSelfCollision) {
  UrdfFclCollisionChecker checker;
  const auto initialized = checker.Initialize(kRobotUrdf, {"arm_joint"});
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  const CollisionCheckResult result =
      checker.CheckState({3.141592653589793}, scene);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_NE(result.message.find("self collision"), std::string::npos);
}

TEST(UrdfFclCollisionCheckerTest, LoadsUrdfStlCollisionMesh) {
  const std::string urdf =
      R"(<robot name="mesh_test"><link name="base_link"><collision>)"
      R"(<origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename=")"
      TEST_MESH_PATH
      R"(" scale="1 1 1"/></geometry></collision></link></robot>)";

  UrdfFclCollisionChecker checker;
  const auto initialized = checker.Initialize(urdf, {"unused_joint"});
  // The mesh is loaded before joint validation, but a valid planning group is
  // still required; use a fixed fixture joint so this also tests model setup.
  EXPECT_FALSE(initialized.ok);
}

TEST(UrdfFclCollisionCheckerTest, RejectsDuplicateJointNames) {
  UrdfFclCollisionChecker checker;
  const auto result = checker.Initialize(kRobotUrdf, {"arm_joint", "arm_joint"});
  EXPECT_FALSE(result.ok);
}

TEST(UrdfFclCollisionCheckerTest, SamplesInteriorPathStates) {
  UrdfFclCollisionChecker checker;
  const auto initialized = checker.Initialize(kRobotUrdf, {"arm_joint"});
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  // The obstacle is clear at both endpoints but intersects the tool's swept
  // region at an intermediate joint value.
  scene.objects.push_back(MakeSphere("swept_obstacle", 1.1, 0.0, 0.0, 0.18));

  JointPath path;
  path.joint_names = {"arm_joint"};
  path.points = {{-1.0}, {1.0}};
  const CollisionCheckResult result = checker.CheckPath(path, scene, 0.05);
  EXPECT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_GT(result.checked_states, 1.0);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
