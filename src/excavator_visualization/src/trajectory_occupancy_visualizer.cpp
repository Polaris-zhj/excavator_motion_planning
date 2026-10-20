#include "excavator_visualization/trajectory_occupancy_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "std_msgs/msg/color_rgba.hpp"

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

Quaternion Multiply(const Quaternion& first, const Quaternion& second) {
  Quaternion result;
  result.w = first.w * second.w - first.x * second.x -
             first.y * second.y - first.z * second.z;
  result.x = first.w * second.x + first.x * second.w +
             first.y * second.z - first.z * second.y;
  result.y = first.w * second.y - first.x * second.z +
             first.y * second.w + first.z * second.x;
  result.z = first.w * second.z + first.x * second.y -
             first.y * second.x + first.z * second.w;
  return result;
}

Vector3 Rotate(const Quaternion& quaternion, const Vector3& vector) {
  const double x2 = quaternion.x * quaternion.x;
  const double y2 = quaternion.y * quaternion.y;
  const double z2 = quaternion.z * quaternion.z;
  const double wx = quaternion.w * quaternion.x;
  const double wy = quaternion.w * quaternion.y;
  const double wz = quaternion.w * quaternion.z;
  const double xy = quaternion.x * quaternion.y;
  const double xz = quaternion.x * quaternion.z;
  const double yz = quaternion.y * quaternion.z;
  Vector3 result;
  result.x = (1.0 - 2.0 * (y2 + z2)) * vector.x +
             2.0 * (xy - wz) * vector.y + 2.0 * (xz + wy) * vector.z;
  result.y = 2.0 * (xy + wz) * vector.x +
             (1.0 - 2.0 * (x2 + z2)) * vector.y +
             2.0 * (yz - wx) * vector.z;
  result.z = 2.0 * (xz - wy) * vector.x + 2.0 * (yz + wx) * vector.y +
             (1.0 - 2.0 * (x2 + y2)) * vector.z;
  return result;
}

bool IsFinite(const Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool IsFinite(const Quaternion& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z) && std::isfinite(value.w);
}

JointPositions SamplePositions(const JointTrajectory& trajectory, double t) {
  if (trajectory.points.size() < 2) {
    return trajectory.points.front().positions;
  }
  if (t <= trajectory.points.front().t) {
    return trajectory.points.front().positions;
  }
  if (t >= trajectory.points.back().t) {
    return trajectory.points.back().positions;
  }
  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    const JointTrajectoryPoint& first = trajectory.points[index - 1];
    const JointTrajectoryPoint& second = trajectory.points[index];
    if (t > second.t) {
      continue;
    }
    const double duration = second.t - first.t;
    const double ratio = duration > 0.0 ? (t - first.t) / duration : 0.0;
    JointPositions positions(first.positions.size(), 0.0);
    for (std::size_t joint = 0; joint < first.positions.size(); ++joint) {
      positions[joint] = first.positions[joint] +
                         ratio * (second.positions[joint] -
                                  first.positions[joint]);
    }
    return positions;
  }
  return trajectory.points.back().positions;
}

}  // namespace

TrajectoryOccupancyVisualizer::TrajectoryOccupancyVisualizer(
    const KinematicsModel& kinematics, std::vector<LinkVisual> link_visuals,
    std::string frame_id, std::size_t max_samples)
    : kinematics_(kinematics),
      link_visuals_(std::move(link_visuals)),
      frame_id_(std::move(frame_id)),
      max_samples_(std::max<std::size_t>(1, max_samples)) {}

ValidationResult TrajectoryOccupancyVisualizer::Build(
    const JointTrajectory& trajectory,
    visualization_msgs::msg::MarkerArray* markers) const {
  if (markers == nullptr) {
    return {false, "marker array output pointer is null"};
  }
  const ValidationResult validation = ValidateJointTrajectory(trajectory);
  if (!validation.ok) {
    return validation;
  }
  if (link_visuals_.empty()) {
    return {false, "occupancy link visual list must not be empty"};
  }
  for (const LinkVisual& visual : link_visuals_) {
    if (visual.link_name.empty()) {
      return {false, "occupancy link visuals need a link name"};
    }
    if (visual.type == LinkVisualType::kMesh &&
        visual.mesh_resource.empty()) {
      return {false, "mesh occupancy visuals need a mesh resource"};
    }
    if (!IsFinite(visual.scale) || visual.scale.x <= 0.0 ||
        visual.scale.y <= 0.0 || visual.scale.z <= 0.0 ||
        !IsFinite(visual.origin.position) ||
        !IsFinite(visual.origin.orientation)) {
      return {false, "occupancy link visual transform or scale is invalid"};
    }
  }

  markers->markers.clear();
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id_;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers->markers.push_back(clear);

  // Sample by trajectory time, not just by waypoint index: time
  // parameterization may have simplified the path down to two waypoints.
  const std::size_t sample_count =
      trajectory.points.size() < 2 ? 1 : max_samples_;
  const double duration = trajectory.points.back().t;
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    const double ratio =
        sample_count > 1 ? static_cast<double>(sample) /
                               static_cast<double>(sample_count - 1)
                         : 0.0;
    const JointPositions positions =
        SamplePositions(trajectory, ratio * duration);
    for (const LinkVisual& visual : link_visuals_) {
      Pose3 link_pose;
      const ValidationResult fk = kinematics_.GetLinkPose(
          positions, visual.link_name, &link_pose);
      if (!fk.ok) {
        return {false, "occupancy forward kinematics failed for link " +
                           visual.link_name + ": " + fk.message};
      }
      const Vector3 rotated_origin =
          Rotate(link_pose.orientation, visual.origin.position);
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.ns = "excavator_occupancy_" + visual.link_name;
      marker.id = static_cast<int>(sample);
      const bool is_box = visual.type == LinkVisualType::kBox;
      marker.type = is_box ? visualization_msgs::msg::Marker::CUBE
                           : visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      if (!is_box) {
        marker.mesh_resource = visual.mesh_resource;
        marker.mesh_use_embedded_materials = false;
      }
      marker.pose.position.x = link_pose.position.x + rotated_origin.x;
      marker.pose.position.y = link_pose.position.y + rotated_origin.y;
      marker.pose.position.z = link_pose.position.z + rotated_origin.z;
      const Quaternion orientation =
          Multiply(link_pose.orientation, visual.origin.orientation);
      marker.pose.orientation.x = orientation.x;
      marker.pose.orientation.y = orientation.y;
      marker.pose.orientation.z = orientation.z;
      marker.pose.orientation.w = orientation.w;
      marker.scale.x = visual.scale.x;
      marker.scale.y = visual.scale.y;
      marker.scale.z = visual.scale.z;
      marker.color.r = visual.red;
      marker.color.g = visual.green;
      marker.color.b = visual.blue;
      marker.color.a = visual.alpha;
      markers->markers.push_back(std::move(marker));
    }
  }
  return {true, ""};
}

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
