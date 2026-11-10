/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <limits>
#include <vector>

#include "common/data/point_extensions.h"
#include "common/data/trajectory/trajectory_common.h"

namespace excavator {
namespace planning {
namespace data {

/// 关节名称（单关节轨迹设计：一条轨迹对应一个关节）
enum class JointName { Unknown = -1, Swing = 0, Boom = 1, Arm = 2, Bucket = 3 };

/// 关节点：单关节角度
struct JointPoint {
  double rad = 0.0;
  PointExtensions extensions;
};

/// 关节轨迹点：带时间戳与运动状态
struct JointTrajectoryPoint {
  JointPoint point;
  double t = 0.0;          ///< 相对时间戳 s
  double velocities = 0.0; ///< 关节速度 rad/s
  double acc = 0.0;        ///< 关节加速度 rad/s^2
};

struct JointTargetPoint {
  double rad = std::numeric_limits<double>::quiet_NaN();
  PointExtension extension = PointExtension::Unknown;
};

/// 单关节轨迹
struct JointTrajectory {
  JointName name = JointName::Unknown;
  PointExtension extension = PointExtension::Unknown;
  std::vector<JointTrajectoryPoint> points; ///< 带时间戳的轨迹点
  std::vector<StopType> stop_types;
  JointTargetPoint target_point;
};

} // namespace data

} // namespace planning
} // namespace excavator
