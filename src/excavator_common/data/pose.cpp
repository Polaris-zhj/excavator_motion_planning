/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/
#include "pose.h"
namespace excavator {
namespace planning {
namespace data {
namespace {
constexpr double kPositionEps = 0.1;
}

Pose::Pose(const double x, const double y, const double z, const double _theta)
    : position(x, y, z),
      orientation(0.0, 0.0, std::sin(0.5 * _theta), std::cos(0.5 * _theta)) {}

Pose::Pose(const double x, const double y, const double z,
           const Orientation &_orientation)
    : position(x, y, z), orientation(_orientation) {}

Pose::Pose(const Point &_position, const double _theta)
    : position(_position),
      orientation(0.0, 0.0, std::sin(0.5 * _theta), std::cos(0.5 * _theta)) {}

Pose::Pose(const Point &_position, const Orientation &_orientation)
    : position(_position), orientation(_orientation) {}

bool Pose::IsEqual(const Pose &other, const double distance_eps,
                   const double angle_eps) const {
  if (DistanceSquare3d(other) >= distance_eps * distance_eps) {
    return false;
  }

  if (!IsValid() || !other.IsValid()) {
    return false;
  }

  return AngularDistance(other) < angle_eps;
}

Pose Pose::TransformTo2d(const Pose &target) const {
  Orientation q_inv = target.orientation.inverse();
  double x = position.x - target.position.x;
  double y = position.y - target.position.y;
  q_inv.RotateXY(x, y);
  return Pose(x, y, position.z, q_inv * orientation);
}

Pose Pose::TransformFrom2d(const Pose &source) const {
  double x = position.x;
  double y = position.y;
  source.orientation.RotateXY(x, y);
  return Pose(x + source.position.x, y + source.position.y, position.z,
              source.orientation * orientation);
}

Pose Pose::TransformTo3d(const Pose &target) const {
  Point relative = position - target.position;
  return Pose(target.orientation.inverse() * relative,
              target.orientation.inverse() * orientation);
}

Pose Pose::TransformFrom3d(const Pose &source) const {
  return Pose(source.orientation * position + source.position,
              source.orientation * orientation);
}

void Pose::TransformXYTo(double &x, double &y) const {
  x -= position.x;
  y -= position.y;
  orientation.inverse().RotateXY(x, y);
}

void Pose::TransformXYZTo(double &x, double &y, double &z) const {
  x -= position.x;
  y -= position.y;
  z -= position.z;
  orientation.inverse().RotateXYZ(x, y, z);
}

void Pose::TransformXYFrom(double &x, double &y) const {
  orientation.RotateXY(x, y);
  x += position.x;
  y += position.y;
}

void Pose::TransformXYZFrom(double &x, double &y, double &z) const {
  orientation.RotateXYZ(x, y, z);
  x += position.x;
  y += position.y;
  z += position.z;
}

std::string Pose::ToString(const int precision) const {
  std::stringstream ss;
  ss << "{" << position.XYToString(precision) << ", theta:" << yaw() << "}";
  return ss.str();
}

std::string Pose::XYToString(const int precision) const {
  return position.XYToString(precision);
}

} // namespace data
} // namespace planning
} // namespace excavator