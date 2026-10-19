#include "excavator_visualization/trajectory_occupancy_visualizer.hpp"
#include "excavator_visualization/trajectory_visualizer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "kinematics/urdf_kinematics_model.hpp"

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

constexpr double kPi = 3.14159265358979323846;

JointTrajectory MakeTrajectory() {
  JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory.points = {
      {0.0, {0.0}, {0.0}, {0.0}},
      {1.0, {kPi / 2.0}, {0.0}, {0.0}}};
  return trajectory;
}

class TrajectoryVisualizerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(kinematics_.InitializeFromFile(TEST_URDF_PATH).ok);
  }
  UrdfKinematicsModel kinematics_{{"joint_a"}};
};

TEST_F(TrajectoryVisualizerTest, BuildsMarkersAndEndEffectorPath) {
  TrajectoryVisualizer visualizer(kinematics_, "base", "tool");
  visualization_msgs::msg::MarkerArray markers;
  const auto result = visualizer.BuildMarkers(MakeTrajectory(), &markers);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_GE(markers.markers.size(), 5U);
  EXPECT_EQ(markers.markers.front().action,
            visualization_msgs::msg::Marker::DELETEALL);
  for (const auto& marker : markers.markers) {
    EXPECT_EQ(marker.header.frame_id, "base");
  }

  nav_msgs::msg::Path path;
  ASSERT_TRUE(visualizer.BuildEndEffectorPath(MakeTrajectory(), &path).ok);
  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_EQ(path.header.frame_id, "base");
  EXPECT_NEAR(path.poses[0].pose.position.x, 2.0, 1.0e-9);
  EXPECT_NEAR(path.poses[1].pose.position.x, 1.0, 1.0e-9);
  EXPECT_NEAR(path.poses[1].pose.position.y, 1.0, 1.0e-9);
}

TEST_F(TrajectoryVisualizerTest, RejectsInvalidTrajectory) {
  TrajectoryVisualizer visualizer(kinematics_, "base", "tool");
  JointTrajectory trajectory = MakeTrajectory();
  trajectory.joint_names = {"joint_a", "joint_b"};
  trajectory.points[0].positions = {0.0, 0.0};
  trajectory.points[1].positions = {0.0, 0.0};
  visualization_msgs::msg::MarkerArray markers;
  EXPECT_FALSE(visualizer.BuildMarkers(trajectory, &markers).ok);
}

TEST_F(TrajectoryVisualizerTest, BuildsMeshOccupancyPreview) {
  LinkVisual arm;
  arm.link_name = "arm";
  arm.mesh_resource = "package://test_robot/meshes/arm.stl";
  arm.alpha = 0.3F;
  LinkVisual tool;
  tool.link_name = "tool";
  tool.mesh_resource = "package://test_robot/meshes/tool.stl";
  tool.alpha = 0.3F;

  TrajectoryOccupancyVisualizer visualizer(
      kinematics_, {arm, tool}, "base", 4);
  visualization_msgs::msg::MarkerArray markers;
  const auto result = visualizer.Build(MakeTrajectory(), &markers);
  ASSERT_TRUE(result.ok) << result.message;
  // DELETEALL + 4 time samples * 2 link meshes, even though the trajectory
  // only contains two waypoints.
  ASSERT_EQ(markers.markers.size(), 9U);
  EXPECT_EQ(markers.markers.front().action,
            visualization_msgs::msg::Marker::DELETEALL);
  for (std::size_t index = 1; index < markers.markers.size(); ++index) {
    const auto& marker = markers.markers[index];
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::MESH_RESOURCE);
    EXPECT_EQ(marker.header.frame_id, "base");
    EXPECT_FALSE(marker.mesh_resource.empty());
    EXPECT_NEAR(marker.color.a, 0.3F, 1.0e-6);
  }
  EXPECT_EQ(markers.markers[1].ns, "excavator_occupancy_arm");
  EXPECT_EQ(markers.markers[1].id, 0);
  EXPECT_EQ(markers.markers[2].ns, "excavator_occupancy_tool");
  EXPECT_EQ(markers.markers[3].id, 1);
}

TEST_F(TrajectoryVisualizerTest, RejectsIncompleteOccupancyVisuals) {
  LinkVisual arm;
  arm.link_name = "arm";
  TrajectoryOccupancyVisualizer visualizer(kinematics_, {arm}, "base", 2);
  visualization_msgs::msg::MarkerArray markers;
  EXPECT_FALSE(visualizer.Build(MakeTrajectory(), &markers).ok);
}

}  // namespace
}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
