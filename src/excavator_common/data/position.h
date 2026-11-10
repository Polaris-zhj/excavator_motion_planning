/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace excavator {
namespace planning {
namespace data {
struct Point {
public:
  /**
   * @brief Empty constructor.
   */
  Point() = default;

  /**
   * @brief Constructor which takes XYZ-coordinate.
   * @param x X-coordinate
   * @param y Y-coordinate
   * @param z Z-coordinate
   */
  Point(const double x, const double y, const double z = 0.0);

  /**
   * @brief Constructor which takes other types of position.
   * @param point The other types of position
   */
  template <typename T>
  explicit Point(const T &point) : x(point.x), y(point.y), z(point.z) {}

  /**
   * @brief Check the validity of this position
   * @return Whether this position is valid
   */
  inline bool IsValid() const {
    return !(std::isnan(x) || std::isnan(y) || std::isnan(z));
  }

  /**
   * @brief Sums two position
   */
  inline Point operator+(const Point &other) const {
    return Point(x + other.x, y + other.y, z + other.z);
  }

  /**
   * @brief Subtracts two position
   */
  inline Point operator-(const Point &other) const {
    return Point(x - other.x, y - other.y, z - other.z);
  }

  /**
   * @brief Multiplies position by a scalar
   */
  inline Point operator*(double scalar) const {
    return Point(x * scalar, y * scalar, z * scalar);
  }

  /**
   * @brief Get the Euclidean distance in 2D to another position
   * @param other Another position
   * @return The Euclidean distance in 2D to another position
   */
  inline double Distance2d(const Point &other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  /**
   * @brief Get the squared Euclidean distance in 2D to another position
   * @param other Another position
   * @return The squared Euclidean distance in 2D to another position
   */
  inline double DistanceSquare2d(const Point &other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    return dx * dx + dy * dy;
  }

  /**
   * @brief Get the Euclidean distance in 3D to another position
   * @param other Another position
   * @return The Euclidean distance in 3D to another position
   */
  inline double Distance3d(const Point &other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    const double dz = z - other.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  /**
   * @brief Get the squared Euclidean distance in 3D to another position
   * @param other Another position
   * @return The squared Euclidean distance in 3D to another position
   */
  inline double DistanceSquare3d(const Point &other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    const double dz = z - other.z;
    return dx * dx + dy * dy + dz * dz;
  }

  /**
   * @brief Check whether the two position are similar
   * @param other Another position
   * @param eps Maximum allowable distance error
   * @return Whether the two positions are similar
   */
  inline bool IsEqual(const Point &other, const double eps = 1e-6) const {
    return DistanceSquare3d(other) < eps * eps;
  }

  /**
   * @brief Get a debug string of position
   * @param precision Precision of output number
   * @return Debug string of pose
   */
  std::string ToString(const int precision = 0) const;

  /**
   * @brief Get a debug string of x-coordinate and y-coordinate
   * @param precision Precision of output number
   * @return Debug string of x-coordinate and y-coordinate
   */
  std::string XYToString(const int precision = 0) const;

public:
  double x = std::numeric_limits<double>::quiet_NaN();
  double y = std::numeric_limits<double>::quiet_NaN();
  double z = std::numeric_limits<double>::quiet_NaN();
};
} // namespace data
} // namespace planning
} // namespace excavator