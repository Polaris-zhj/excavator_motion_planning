#include "excavator_visualization/trajectory_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

std_msgs::msg::ColorRGBA MakeColor(float red, float green, float blue,
                                   float alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

geometry_msgs::msg::Point ToPoint(const Vector3& position) {
  geometry_msgs::msg::Point point;
  point.x = position.x;
  point.y = position.y;
  point.z = position.z;
  return point;
}

geometry_msgs::msg::Pose ToPose(const Pose3& pose) {
  geometry_msgs::msg::Pose message;
  message.position.x = pose.position.x;
  message.position.y = pose.position.y;
  message.position.z = pose.position.z;
  message.orientation.x = pose.orientation.x;
  message.orientation.y = pose.orientation.y;
  message.orientation.z = pose.orientation.z;
  message.orientation.w = pose.orientation.w;
  return message;
}

visualization_msgs::msg::Marker MakeDeleteAll(const std::string& frame_id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace

TrajectoryVisualizer::TrajectoryVisualizer(
    const KinematicsModel& kinematics, std::string frame_id,
    std::string tip_link)
    : kinematics_(kinematics),
      frame_id_(std::move(frame_id)),
      tip_link_(std::move(tip_link)) {}

void TrajectoryVisualizer::SetFrameId(const std::string& frame_id) {
  frame_id_ = frame_id;
}

void TrajectoryVisualizer::SetTipLink(const std::string& tip_link) {
  tip_link_ = tip_link;
}

ValidationResult TrajectoryVisualizer::ComputeTipPoses(
    const JointTrajectory& trajectory, std::vector<Pose3>* poses) const {
  if (poses == nullptr) {
    return {false, "tip pose output pointer is null"};
  }
  const ValidationResult validation = ValidateJointTrajectory(trajectory);
  if (!validation.ok) {
    return validation;
  }
  poses->clear();
  poses->reserve(trajectory.points.size());
  for (const JointTrajectoryPoint& point : trajectory.points) {
    Pose3 pose;
    const ValidationResult fk =
        kinematics_.GetLinkPose(point.positions, tip_link_, &pose);
    if (!fk.ok) {
      return {false, "forward kinematics failed: " + fk.message};
    }
    poses->push_back(pose);
  }
  return {true, ""};
}

ValidationResult TrajectoryVisualizer::BuildMarkers(
    const JointTrajectory& trajectory,
    visualization_msgs::msg::MarkerArray* markers) const {
  if (markers == nullptr) {
    return {false, "marker array output pointer is null"};
  }
  std::vector<Pose3> poses;
  const ValidationResult poses_result = ComputeTipPoses(trajectory, &poses);
  if (!poses_result.ok) {
    return poses_result;
  }
  markers->markers.clear();
  markers->markers.push_back(MakeDeleteAll(frame_id_));

  visualization_msgs::msg::Marker path;
  path.header.frame_id = frame_id_;
  path.ns = "excavator_planned_path";
  path.id = 0;
  path.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path.action = visualization_msgs::msg::Marker::ADD;
  path.scale.x = 0.03;
  path.color = MakeColor(0.0F, 0.85F, 0.2F, 1.0F);
  path.points.reserve(poses.size());
  for (const Pose3& pose : poses) {
    path.points.push_back(ToPoint(pose.position));
  }
  markers->markers.push_back(path);

  visualization_msgs::msg::Marker waypoints;
  waypoints.header.frame_id = frame_id_;
  waypoints.ns = "excavator_planned_waypoints";
  waypoints.id = 1;
  waypoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  waypoints.action = visualization_msgs::msg::Marker::ADD;
  waypoints.scale.x = 0.04;
  waypoints.scale.y = 0.04;
  waypoints.scale.z = 0.04;
  waypoints.color = MakeColor(0.1F, 0.4F, 1.0F, 1.0F);
  waypoints.points = path.points;
  markers->markers.push_back(waypoints);

  visualization_msgs::msg::Marker start;
  start.header.frame_id = frame_id_;
  start.ns = "excavator_plan_start";
  start.id = 2;
  start.type = visualization_msgs::msg::Marker::SPHERE;
  start.action = visualization_msgs::msg::Marker::ADD;
  start.pose = ToPose(poses.front());
  start.scale.x = 0.12;
  start.scale.y = 0.12;
  start.scale.z = 0.12;
  start.color = MakeColor(0.0F, 0.6F, 0.1F, 0.9F);
  markers->markers.push_back(start);

  visualization_msgs::msg::Marker goal;
  goal.header.frame_id = frame_id_;
  goal.ns = "excavator_plan_goal";
  goal.id = 3;
  goal.type = visualization_msgs::msg::Marker::SPHERE;
  goal.action = visualization_msgs::msg::Marker::ADD;
  goal.pose = ToPose(poses.back());
  goal.scale.x = 0.12;
  goal.scale.y = 0.12;
  goal.scale.z = 0.12;
  goal.color = MakeColor(0.95F, 0.2F, 0.1F, 0.9F);
  markers->markers.push_back(goal);

  visualization_msgs::msg::Marker axis;
  axis.header.frame_id = frame_id_;
  axis.ns = "excavator_plan_goal_axis";
  axis.id = 4;
  axis.type = visualization_msgs::msg::Marker::ARROW;
  axis.action = visualization_msgs::msg::Marker::ADD;
  axis.pose = ToPose(poses.back());
  axis.pose.position.z += 0.25;
  axis.scale.x = 0.25;
  axis.scale.y = 0.04;
  axis.scale.z = 0.06;
  axis.color = MakeColor(0.95F, 0.2F, 0.1F, 0.9F);
  markers->markers.push_back(axis);
  return {true, ""};
}

ValidationResult TrajectoryVisualizer::BuildEndEffectorPath(
    const JointTrajectory& trajectory, nav_msgs::msg::Path* path) const {
  if (path == nullptr) {
    return {false, "path output pointer is null"};
  }
  std::vector<Pose3> poses;
  const ValidationResult poses_result = ComputeTipPoses(trajectory, &poses);
  if (!poses_result.ok) {
    return poses_result;
  }
  path->header.frame_id = frame_id_;
  path->poses.clear();
  path->poses.reserve(poses.size());
  for (const Pose3& pose : poses) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = frame_id_;
    stamped.pose = ToPose(pose);
    path->poses.push_back(std::move(stamped));
  }
  return {true, ""};
}

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
