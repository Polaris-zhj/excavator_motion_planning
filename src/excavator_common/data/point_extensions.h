/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file point_extensions.h
 **/

#pragma once

#include "common/logging.hpp"
#include <stdint.h>

namespace excavator {
namespace planning {
namespace data {

enum class PointExtension : uint8_t { Unknown = 255, MAX = 64 };

struct PointExtensions {
public:
  PointExtensions() = default;

public:
  inline void Clear() { bits_ = 0; }

  inline void Add(const PointExtension &e) {
    if (e < PointExtension::MAX) {
      bits_ = (bits_ | (static_cast<uint64_t>(1) << static_cast<uint8_t>(e)));
    } else {
      LOG(WARNING) << "Extension type out of range:" << static_cast<int>(e);
    }
  }

  inline void Remove(const PointExtension &e) {
    if (e < PointExtension::MAX) {
      bits_ = (bits_ & ~(static_cast<uint64_t>(1) << static_cast<uint8_t>(e)));
    } else {
      LOG(WARNING) << "Extension type out of range:" << static_cast<int>(e);
    }
  }

  inline bool operator[](const PointExtension &e) const {
    if (e < PointExtension::MAX) {
      return (bits_ >> static_cast<uint8_t>(e)) & static_cast<uint64_t>(1);
    } else {
      LOG(WARNING) << "Extension type out of range:" << static_cast<int>(e);
      return false;
    }
  }

private:
  uint64_t bits_ = 0;
};

} // namespace data
} // namespace planning
} // namespace excavator
