/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/

#include "orientation.h"

namespace excavator {
namespace planning {
namespace data {

Orientation::Orientation(const double _x, const double _y, const double _z,
                         const double _w)
    : x(_x), y(_y), z(_z), w(_w) {}

std::string Orientation::ToString() const {
  std::stringstream ss;
  ss << " (x:" << x << ", y:" << y << ", z:" << z << ", w:" << w << ")";
  return ss.str();
}

Orientation Orientation::CreateFromYaw(const double yaw) {
  return Orientation(0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw));
}

Orientation Orientation::CreateFromRollPitchYaw(const double roll,
                                                const double pitch,
                                                const double yaw) {
  Orientation result;
  result.SetFromRollPitchYaw(roll, pitch, yaw);
  return result;
}

} // namespace data
} // namespace planning
} // namespace excavator