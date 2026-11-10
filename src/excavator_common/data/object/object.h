/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "../position.h"

namespace excavator {
namespace planning {
namespace data {
enum ObstacleType : int {
  unkown = -1,
  person = 0,
  truck = 1,
  car = 2,
  digger = 3,
  pushcar = 4,
  stone = 5,
  lidar_stone = 6,
  heteromorphic_obstacles = 7,
  forbiden_area = 8,
  sprinkler = 9,
  tanker = 10,
  bucket = 11,
  slope = 12,
  boundary = 30,
  roi_boundary = 50,
  virtual_parking_space = 51
};

struct Object {
  int id;
  ObstacleType type;
  Point center;
  Point velocity;
  double length;
  double width;
  double height;
  std::vector<Point> cloud_points;
  std::vector<Point> polygon_points;

  Object()
      : id(-1), type(ObstacleType::unkown), center(Point()), velocity(Point()) {
    length = std::numeric_limits<double>::quiet_NaN();
    width = std::numeric_limits<double>::quiet_NaN();
    height = std::numeric_limits<double>::quiet_NaN();
    cloud_points.clear();
    polygon_points.clear();
  }
};

} // namespace data
} // namespace planning
} // namespace excavator