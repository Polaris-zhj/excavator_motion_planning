/******************************************************************************
 * Copyright 2026 The CIDI Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "orientation.h"
#include "position.h"
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace excavator {
namespace planning {
namespace data {
constexpr double kPoseValidValue = 0.0;

class Pose {
public:
  /**
   * @brief Empty constructor.
   */
  Pose() = default;

  /**
   * @brief Constructor which takes XYZ-coordinate and yaw(rad).
   * @param x X-coordinate
   * @param y Y-coordinate
   * @param z Z-coordinate
   * @param theta Yaw(rad)
   */
  Pose(const double x, const double y, const double z, const double yaw);

  /**
   * @brief Constructor which takes XYZ-coordinate and quaternion.
   * @param x X-coordinate
   * @param y Y-coordinate
   * @param z Z-coordinate
   * @param orientation Orientation
   */
  Pose(const double x, const double y, const double z,
       const Orientation &orientation);

  /**
   * @brief Constructor which takes translation and yaw(rad).
   * @param position Translation
   * @param yaw Yaw(rad)
   */
  Pose(const Point &position, const double yaw);

  /**
   * @brief Constructor which takes translation and quaternion.
   * @param position Translation
   * @param orientation Orientation
   */
  Pose(const Point &position, const Orientation &orientation);

  /**
   * @brief Constructor which takes other types of pose.
   * @param pose The other types of pose
   */
  template <typename T>
  explicit Pose(const T &pose)
      : position(pose.position), orientation(pose.orientation) {}

  /**
   * @brief Get x-coordinate of this pose
   * @return X-coordinate of this pose
   */
  inline double x() const { return position.x; }

  /**
   * @brief Get y-coordinate of this pose
   * @return Y-coordinate of this pose
   */
  inline double y() const { return position.y; }

  /**
   * @brief Get z-coordinate of this pose
   * @return Z-coordinate of this pose
   */
  inline double z() const { return position.z; }

  /**
   * @brief Get yaw(rad) of this pose
   * @return Yaw of this pose
   */
  inline double yaw() const { return orientation.GetYaw(); }

  /**
   * @brief Get pitch(rad) of this pose
   * @return Pitch of this pose
   */
  inline double pitch() const { return orientation.GetPitch(); }

  /**
   * @brief Get roll(rad) of this pose
   * @return Roll of this pose
   */
  inline double roll() const { return orientation.GetRoll(); }

  /**
   * @brief Set x-coordinate of this pose
   * @param x The x-coordinate set to this pose
   */
  inline void set_x(const double x) { position.x = x; }

  /**
   * @brief Set y-coordinate of this pose
   * @param y The y-coordinate set to this pose
   */
  inline void set_y(const double y) { position.y = y; }

  /**
   * @brief Set z-coordinate of this pose
   * @param z The z-coordinate set to this pose
   */
  inline void set_z(const double z) { position.z = z; }

  /**
   * @brief Set yaw(rad) of this pose
   * @param yaw The yaw(rad) set to this pose
   */
  inline void set_yaw(const double yaw) { orientation.SetFromYaw(yaw); }

  /**
   * @brief Check whether this pose is valid (no NaN and orientation is valid)
   * @return Whether this pose is valid
   */
  inline bool IsValid() const {
    return position.IsValid() && orientation.IsValid();
  }

  /**
   * @brief Get the Euclidean distance in two dimensions to another pose
   * @param other Another pose
   * @return The Euclidean distance in two dimensions to another pose
   */
  inline double Distance2d(const Pose &other) const {
    return position.Distance2d(other.position);
  }

  /**
   * @brief Get the squared Euclidean distance in two dimensions to another pose
   * @param other Another pose
   * @return The squared Euclidean distance in two dimensions to another pose
   */
  inline double DistanceSquare2d(const Pose &other) const {
    return position.DistanceSquare2d(other.position);
  }

  /**
   * @brief Get the Euclidean distance in three dimensions to another pose
   * @param other Another pose
   * @return The Euclidean distance in three dimensions to another pose
   */
  inline double Distance3d(const Pose &other) const {
    return position.Distance3d(other.position);
  }

  /**
   * @brief Get the squared Euclidean distance in 3 dimensions to another pose
   * @param other Another pose
   * @return The squared Euclidean distance in 3 dimensions to another pose
   */
  inline double DistanceSquare3d(const Pose &other) const {
    return position.DistanceSquare3d(other.position);
  }

  /**
   * @brief Get the angular distance between two pose quaternions
   * @param other Another pose
   * @return Angular distance between two pose quaternions
   */
  inline double AngularDistance(const Pose &pose) const {
    return orientation.AngularDistance(pose.orientation);
  }

  /**
   * @brief Get the angle(rad) relative to target pose
   * @param other Target pose
   * @return Angle(rad) relative to target pose
   */
  inline double RelativeAngle(const Pose &other) const {
    return (other.orientation.inverse() * orientation).GetYaw();
  }

  /**
   * @brief Check whether the two poses are similar
   * @param other Another pose
   * @param distance_error Maximum allowable distance error
   * @param angle_error Maximum allowable angle error(rad)
   * @return Whether the two poses are similar
   */
  bool IsEqual(const Pose &other, const double distance_error = 0.5,
               const double angle_error = 0.04) const;

  /**
   * @brief Transform this pose to the coordinate system of target pose
   * @param target target pose
   * @return Pose transformed to the coordinate system of target pose
   */
  Pose TransformTo2d(const Pose &target) const;

  /**
   * @brief Transform this pose to the coordinate system of target pose (3D)
   * @param target target pose
   * @return Pose transformed to the coordinate system of target pose
   */
  Pose TransformTo3d(const Pose &target) const;

  /**
   * @brief Transform this pose from the coordinate system of source pose
   * @param source source pose
   * @return Pose transformed from the coordinate system of source pose
   */
  Pose TransformFrom2d(const Pose &source) const;

  /**
   * @brief Transform this pose from the coordinate system of source pose (3D)
   * @param source source pose
   * @return Pose transformed from the coordinate system of source pose
   */
  Pose TransformFrom3d(const Pose &source) const;

  /**
   * @brief Transform x, y to the coordinate system of this pose
   * @param x X-coordinate to be transformed
   * @param x Y-coordinate to be transformed
   * @return X, Y transformed to the coordinate system of this pose
   */
  void TransformXYTo(double &x, double &y) const;

  /**
   * @brief Transform x, y, z to the coordinate system of this pose (3D)
   * @param x X-coordinate to be transformed
   * @param y Y-coordinate to be transformed
   * @param z Z-coordinate to be transformed
   */
  void TransformXYZTo(double &x, double &y, double &z) const;

  /**
   * @brief Transform x, y from the coordinate system of this pose
   * @param x X-coordinate to be transformed
   * @param x Y-coordinate to be transformed
   * @return X, Y transformed from the coordinate system of this pose
   */
  void TransformXYFrom(double &x, double &y) const;

  /**
   * @brief Transform x, y, z from the coordinate system of this pose (3D)
   * @param x X-coordinate to be transformed
   * @param y Y-coordinate to be transformed
   * @param z Z-coordinate to be transformed
   */
  void TransformXYZFrom(double &x, double &y, double &z) const;

  /**
   * @brief Get a debug string of pose
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
  Point position;
  Orientation orientation;
};
} // namespace data
} // namespace planning
} // namespace excavator