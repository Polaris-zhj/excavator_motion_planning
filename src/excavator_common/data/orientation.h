/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "position.h"


namespace excavator {
namespace planning {
namespace data {
constexpr double kOrientationNormSquareLower = 0.99999;
constexpr double kOrientationNormSquareUpper = 1.00001;

struct Orientation {
public:
  /**
   * @brief Empty constructor.
   */
  Orientation() = default;

  /**
   * @brief Constructor which takes quaternion components‌.
   * @param x X component of quaternion.
   * @param y Y component of quaternion.
   * @param z Z component of quaternion.
   * @param w W component of quaternion.
   */
  Orientation(const double x, const double y, const double z, const double w);

  /**
   * @brief Get the square of the quaternion norm
   * @return Square of the quaternion norm
   */
  inline double norm_square() const { return x * x + y * y + z * z + w * w; }

  /**
   * @brief Get the conjugate of quaternion
   * @return Conjugate of quaternion
   */
  inline Orientation conjugate() const { return Orientation(-x, -y, -z, w); }

  /**
   * @brief Get the inverse of the quaternion
   * @return Inverse of the quaternion
   */
  inline Orientation inverse() const {
    const double square = norm_square();
    if (square <= 0.0) {
      return Orientation(0.0, 0.0, 0.0, 1.0);
    }
    const double div = 1.0 / square;
    return Orientation(-x * div, -y * div, -z * div, w * div);
  }

  /**
   * @brief Rotate the point by this quaternion
   * @param point The point to be rotated
   * @return Point rotated by this quaternion
   */
  inline Point operator*(const Point &point) const {
    double x = point.x;
    double y = point.y;
    double z = point.z;
    RotateXYZ(x, y, z);
    return Point(x, y, z);
  }

  /**
   * @brief Multiplies two quaternion
   * @param other Another quaternion
   * @return Rusult of quaternion multiplication
   */
  inline Orientation operator*(const Orientation &other) const {
    return Orientation(w * other.x + x * other.w + y * other.z - z * other.y,
                       w * other.y - x * other.z + y * other.w + z * other.x,
                       w * other.z + x * other.y - y * other.x + z * other.w,
                       w * other.w - x * other.x - y * other.y - z * other.z);
  }

  /**
   * @brief Rotate x-coordinate and y-coordinate
   * @param x X-coordinate to be rotated
   * @param x Y-coordinate to be rotated
   * @return X-coordinate and Y-coordinate rotated by quaternion
   */
  inline void RotateXY(double &px, double &py) const {
    const double uv_x = -2.0 * z * py;
    const double uv_y = 2.0 * z * px;
    const double uv_z = 2.0 * (x * py - y * px);
    px += (w * uv_x + (y * uv_z - z * uv_y));
    py += (w * uv_y + (z * uv_x - x * uv_z));
  }

  /**
   * @brief Rotate x, y, z coordinate
   * @param x x coordinate to be rotated
   * @param y y coordinate to be rotated
   * @param z z coordinate to be rotated
   * @return x,y,z coordinate rotated by quaternion
   */
  inline void RotateXYZ(double &px, double &py, double &pz) const {
    const double uv_x = 2.0 * (y * pz - z * py);
    const double uv_y = 2.0 * (z * px - x * pz);
    const double uv_z = 2.0 * (x * py - y * px);
    px += (w * uv_x + (y * uv_z - z * uv_y));
    py += (w * uv_y + (z * uv_x - x * uv_z));
    pz += (w * uv_z + (x * uv_y - y * uv_x));
  }

  /**
   * @brief Check the validity of this quaternion
   * @return Whether this pose is valid
   */
  inline bool IsValid() const {
    if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w)) {
      return false;
    }
    double squared_mag = norm_square();
    return kOrientationNormSquareLower < squared_mag &&
           squared_mag < kOrientationNormSquareUpper;
  }

  /**
   * @brief Get angular distance between two quaternions
   * @param other Another quaternion
   * @return Angular distance between two quaternions
   */
  inline double AngularDistance(const Orientation &other) const {
    Orientation d = (*this) * other.conjugate();
    return 2.0 * std::atan2(std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z),
                            std::abs(d.w));
  }

  /**
   * @brief Get yaw from quaternion
   * @return Yaw
   */
  inline double GetYaw() const {
    return std::atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z);
  }

  /**
   * @brief Get roll from quaternion (ZYX Euler convention, X-axis)
   * @return Roll
   */
  inline double GetRoll() const {
    return std::atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z);
  }

  /**
   * @brief Get pitch from quaternion (ZYX Euler convention, Y-axis)
   * @return Pitch
   */
  inline double GetPitch() const {
    const double sinp = 2.0 * (w * y - z * x);
    return std::asin(std::clamp(sinp, -1.0, 1.0));
  }

  /**
   * @brief Set quaternion components from yaw
   * @param yaw Yaw
   */
  inline void SetFromYaw(const double yaw) {
    x = 0.0;
    y = 0.0;
    z = std::sin(0.5 * yaw);
    w = std::cos(0.5 * yaw);
  }

  /**
   * @brief Set quaternion components from roll, pitch, yaw (ZYX Euler)
   * @param roll Roll (X-axis)
   * @param pitch Pitch (Y-axis)
   * @param yaw Yaw (Z-axis)
   */
  inline void SetFromRollPitchYaw(const double roll, const double pitch,
                                  const double yaw) {
    const double cr = std::cos(0.5 * roll);
    const double sr = std::sin(0.5 * roll);
    const double cp = std::cos(0.5 * pitch);
    const double sp = std::sin(0.5 * pitch);
    const double cy = std::cos(0.5 * yaw);
    const double sy = std::sin(0.5 * yaw);
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
    w = cr * cp * cy + sr * sp * sy;
  }

  /**
   * @brief Get a debug string of orientation
   * @return Debug string of orientation
   */
  std::string ToString() const;

  /**
   * @brief Create quaternion from yaw.
   * @param yaw Yaw
   * @return  Orientation created from yaw
   */
  static Orientation CreateFromYaw(const double yaw);

  /**
   * @brief Create quaternion from roll, pitch, yaw (ZYX Euler).
   * @param roll Roll (X-axis)
   * @param pitch Pitch (Y-axis)
   * @param yaw Yaw (Z-axis)
   * @return Orientation created from roll, pitch, yaw
   */
  static Orientation CreateFromRollPitchYaw(const double roll, const double pitch,
                                            const double yaw);

public:
  double x = std::numeric_limits<double>::quiet_NaN();
  double y = std::numeric_limits<double>::quiet_NaN();
  double z = std::numeric_limits<double>::quiet_NaN();
  double w = std::numeric_limits<double>::quiet_NaN();
};
} // namespace data
} // namespace planning
} // namespace excavator