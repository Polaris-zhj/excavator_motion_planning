/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <vector>

#include "common/data/pose.h"

namespace excavator {
namespace planning {
namespace data {

enum class StopType {
  DEFAULT = 0,        // 默认停止点，包括速度过低、线路过短等情况
  STOP_PAUSE = 1,     // 中台暂停点
  STOP_WORK_POSE = 2, // 工作目标点
  STOP_OBSTALCE = 3,  // 障碍物停止点
  STOP_EMERGENCY = 4, // 紧急停车点
  STOP_OBU_MEET = 5,  // obu会车停车
  STOP_DEVIATE = 6,   // 偏离
  STOP_REASON_ROUTE = 7,
  STOP_REASON_ABNORMAL = 8, // 异常停车
  // 未知原因停止点，仅用于初始化，正常情况下不能出现该类型
  STOP_UNKNOW = 255
};

struct TargetPose {
  Pose pose;
  std::vector<StopType> stop_types;
};

} // namespace data
} // namespace planning
} // namespace excavator
