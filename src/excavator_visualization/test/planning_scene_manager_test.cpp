#include "excavator_visualization/planning_scene_manager.hpp"

#include <gtest/gtest.h>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

geometry_msgs::msg::Pose MakePose(double x, double y, double z) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Vector3 MakeSize(double x, double y, double z) {
  geometry_msgs::msg::Vector3 size;
  size.x = x;
  size.y = y;
  size.z = z;
  return size;
}

TEST(PlanningSceneManagerTest, AddsAndRemovesBoxes) {
  PlanningSceneManager manager("world");
  ASSERT_TRUE(manager.AddBox("box_1", MakePose(1.0, 2.0, 0.5),
                             MakeSize(1.0, 2.0, 0.5))
                  .ok);
  EXPECT_EQ(manager.Size(), 1U);
  EXPECT_FALSE(manager.AddBox("box_1", MakePose(1.0, 2.0, 0.5),
                              MakeSize(1.0, 2.0, 0.5))
                   .ok);

  const auto snapshot = manager.Snapshot();
  ASSERT_EQ(snapshot.objects.size(), 1U);
  EXPECT_EQ(snapshot.header.frame_id, "world");
  EXPECT_FALSE(snapshot.is_diff);
  EXPECT_EQ(snapshot.objects[0].id, "box_1");
  EXPECT_TRUE(snapshot.objects[0].enabled);
  EXPECT_EQ(snapshot.objects[0].primitive.type,
            shape_msgs::msg::SolidPrimitive::BOX);
  ASSERT_EQ(snapshot.objects[0].primitive.dimensions.size(), 3U);
  EXPECT_DOUBLE_EQ(snapshot.objects[0].primitive.dimensions[2], 0.5);

  ASSERT_TRUE(manager.UpdateBox("box_1", MakePose(2.0, 2.0, 0.5),
                                MakeSize(1.0, 2.0, 1.0))
                  .ok);
  EXPECT_DOUBLE_EQ(manager.Snapshot().objects[0].pose.position.x, 2.0);
  ASSERT_TRUE(manager.RemoveBox("box_1").ok);
  EXPECT_EQ(manager.Size(), 0U);
  EXPECT_FALSE(manager.RemoveBox("box_1").ok);
}

TEST(PlanningSceneManagerTest, RejectsInvalidBoxes) {
  PlanningSceneManager manager;
  EXPECT_FALSE(
      manager.AddBox("", MakePose(0.0, 0.0, 0.0), MakeSize(1.0, 1.0, 1.0))
          .ok);
  EXPECT_FALSE(
      manager.AddBox("box", MakePose(0.0, 0.0, 0.0), MakeSize(0.0, 1.0, 1.0))
          .ok);
  geometry_msgs::msg::Pose invalid_pose = MakePose(0.0, 0.0, 0.0);
  invalid_pose.orientation.w = 0.0;
  EXPECT_FALSE(
      manager.AddBox("box", invalid_pose, MakeSize(1.0, 1.0, 1.0)).ok);
}

TEST(PlanningSceneManagerTest, ClearRemovesAllObjects) {
  PlanningSceneManager manager;
  ASSERT_TRUE(
      manager.AddBox("a", MakePose(0.0, 0.0, 0.0), MakeSize(1.0, 1.0, 1.0))
          .ok);
  ASSERT_TRUE(
      manager.AddBox("b", MakePose(1.0, 0.0, 0.0), MakeSize(1.0, 1.0, 1.0))
          .ok);
  manager.Clear();
  EXPECT_EQ(manager.Size(), 0U);
  EXPECT_TRUE(manager.Snapshot().objects.empty());
}

}  // namespace
}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
