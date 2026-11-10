/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "common/data/point_extensions.h"
#include "common/data/pose.h"
#include "common/data/trajectory/trajectory_common.h"

namespace excavator {
namespace planning {
namespace data {

struct CartesianTrajectoryPoint {
  CartesianTrajectoryPoint() = default;

  template <typename T>
  explicit CartesianTrajectoryPoint(const T &_pose) : pose(_pose) {}

  Pose pose;
  double kappa = 0.0;
  double s = 0.0;
  double t = 0.0;
  double speed = 0.0; // TODO(all): 改成v
  double a = 0.0;
  double j = 0.0;
  double slope = 0.0;
  PointExtensions extensions;

  std::string ToString(const int precision) const {
    return pose.ToString(precision);
  }
};

struct CartesianRoutePoint {
  CartesianRoutePoint() = default;

  template <typename T>
  explicit CartesianRoutePoint(const T &_pose) : pose(_pose) {}

  Pose pose;
  double s = 0.0;
  double kappa = 0.0;
  double speed = 0.0;
  double slope = 0.0;
  PointExtensions extensions;
};

struct CartesianTrajectoryInfo {
  std::vector<CartesianTrajectoryPoint> path;
  PointExtension extension = PointExtension::Unknown;
  int stitch_index = 0;
  uint64_t timer = 0;
  std::vector<StopType> stop_types;
  TargetPose target_pose;
};

struct RouteInfo {
  std::vector<CartesianRoutePoint> path;
  int scene = 0;
  int stitch_index = 0;
  uint64_t timer = 0;
  std::vector<StopType> stop_types;
  TargetPose target_pose;

  RouteInfo() = default;

  explicit RouteInfo(const std::vector<CartesianRoutePoint> &route_poses)
      : path(route_poses) {}
};

} // namespace data

} // namespace planning
} // namespace excavator
