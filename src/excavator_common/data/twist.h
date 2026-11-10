/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

namespace excavator {
namespace planning {
namespace data {

/// 二维速度（机身线速度 + 角速度）
struct Twist2D {
  double linear{0.0};   ///< 机身线速度 m/s
  double angular{0.0};  ///< 机身角速度 rad/s
};

}  // namespace data
}  // namespace planning
}  // namespace excavator
