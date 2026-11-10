#include "excavator_collision/obstacle_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace excavator {
namespace planning {
namespace {

constexpr int kMaxObstacleId = std::numeric_limits<int>::max();

bool IsPositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

ValidationResult PrimitiveHalfExtents(
    const shape_msgs::msg::SolidPrimitive& primitive, const std::string& id,
    Eigen::Vector3d* half_extents, bool* has_geometry) {
  *has_geometry = false;
  if (primitive.dimensions.empty()) {
    return {true, ""};
  }

  switch (primitive.type) {
    case shape_msgs::msg::SolidPrimitive::BOX:
      if (primitive.dimensions.size() != 3 ||
          !IsPositiveFinite(primitive.dimensions[0]) ||
          !IsPositiveFinite(primitive.dimensions[1]) ||
          !IsPositiveFinite(primitive.dimensions[2])) {
        return {false, "environment box requires three positive dimensions: " +
                           id};
      }
      *half_extents = 0.5 * Eigen::Vector3d(primitive.dimensions[0],
                                            primitive.dimensions[1],
                                            primitive.dimensions[2]);
      *has_geometry = true;
      return {true, ""};
    case shape_msgs::msg::SolidPrimitive::SPHERE:
      if (primitive.dimensions.size() != 1 ||
          !IsPositiveFinite(primitive.dimensions[0])) {
        return {false, "environment sphere requires one positive radius: " + id};
      }
      *half_extents = Eigen::Vector3d::Constant(primitive.dimensions[0]);
      *has_geometry = true;
      return {true, ""};
    case shape_msgs::msg::SolidPrimitive::CYLINDER:
      if (primitive.dimensions.size() != 2 ||
          !IsPositiveFinite(primitive.dimensions
                                [shape_msgs::msg::SolidPrimitive::
                                     CYLINDER_HEIGHT]) ||
          !IsPositiveFinite(primitive.dimensions
                                [shape_msgs::msg::SolidPrimitive::
                                     CYLINDER_RADIUS])) {
        return {false,
                "environment cylinder requires positive height and radius: " +
                    id};
      }
      *half_extents = Eigen::Vector3d(
          primitive.dimensions[shape_msgs::msg::SolidPrimitive::
                                   CYLINDER_RADIUS],
          primitive.dimensions[shape_msgs::msg::SolidPrimitive::
                                   CYLINDER_RADIUS],
          0.5 * primitive.dimensions[shape_msgs::msg::SolidPrimitive::
                                         CYLINDER_HEIGHT]);
      *has_geometry = true;
      return {true, ""};
    default:
      // Unsupported primitive types are reported later only when no point
      // cloud representation is available.
      return {true, ""};
  }
}

ValidationResult ReadPointCloud(const sensor_msgs::msg::PointCloud2& cloud,
                                const std::string& id,
                                std::vector<data::Point>* points) {
  if (cloud.point_step == 0 || cloud.width == 0 || cloud.height == 0) {
    return {true, ""};
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      data::Point point;
      point.x = *x;
      point.y = *y;
      point.z = *z;
      points->push_back(point);
    }
  } catch (const std::exception& exception) {
    return {false, "invalid point cloud fields for " + id + ": " +
                       exception.what()};
  }
  return {true, ""};
}

}  // namespace

int ObstacleAdapter::ObjectId(const std::string& id) {
  if (!id.empty()) {
    std::size_t consumed = 0;
    try {
      const long long value = std::stoll(id, &consumed, 10);
      if (consumed == id.size() && value >= 0 && value <= kMaxObstacleId) {
        return static_cast<int>(value);
      }
    } catch (const std::exception&) {
      // Fall through to the hash based id.
    }
  }

  std::uint32_t hash = 2166136261u;
  for (const unsigned char character : id) {
    hash ^= character;
    hash *= 16777619u;
  }
  return static_cast<int>(hash & 0x7FFFFFFFu);
}

ValidationResult ObstacleAdapter::Convert(
    const excavator_interfaces::msg::CollisionObject& source,
    data::Object* object) {
  if (object == nullptr) {
    return {false, "obstacle output is null"};
  }
  if (source.id.empty()) {
    return {false, "obstacle id must not be empty"};
  }
  if (!source.enabled) {
    return {false, "obstacle is disabled: " + source.id};
  }

  const auto& position = source.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z)) {
    return {false, "invalid obstacle pose: " + source.id};
  }

  data::Object converted;
  converted.id = ObjectId(source.id);
  converted.type = data::ObstacleType::unkown;
  converted.center.x = position.x;
  converted.center.y = position.y;
  converted.center.z = position.z;

  Eigen::Vector3d half_extents;
  bool has_geometry = false;
  const ValidationResult primitive =
      PrimitiveHalfExtents(source.primitive, source.id, &half_extents,
                           &has_geometry);
  if (!primitive.ok) {
    return primitive;
  }

  if (has_geometry) {
    Eigen::Vector3d extents = half_extents;
    if (source.primitive.type != shape_msgs::msg::SolidPrimitive::SPHERE) {
      const Eigen::Quaterniond quaternion(
          source.pose.orientation.w, source.pose.orientation.x,
          source.pose.orientation.y, source.pose.orientation.z);
      if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-12) {
        return {false, "invalid obstacle orientation: " + source.id};
      }
      // Enclose the oriented primitive with an axis-aligned box.
      extents = quaternion.normalized().toRotationMatrix().cwiseAbs() *
                half_extents;
    }
    converted.length = 2.0 * extents.x();
    converted.width = 2.0 * extents.y();
    converted.height = 2.0 * extents.z();
  }

  const ValidationResult cloud =
      ReadPointCloud(source.point_cloud, source.id, &converted.cloud_points);
  if (!cloud.ok) {
    return cloud;
  }

  *object = std::move(converted);
  return {true, ""};
}

ValidationResult ObstacleAdapter::Convert(
    const std::vector<excavator_interfaces::msg::CollisionObject>& sources,
    std::vector<data::Object>* objects) {
  if (objects == nullptr) {
    return {false, "obstacle list output is null"};
  }
  objects->clear();
  objects->reserve(sources.size());
  for (const auto& source : sources) {
    if (!source.enabled) {
      continue;
    }
    data::Object object;
    const ValidationResult result = Convert(source, &object);
    if (!result.ok) {
      return result;
    }
    objects->push_back(std::move(object));
  }
  return {true, ""};
}

ValidationResult ObstacleAdapter::Convert(
    const excavator_interfaces::msg::PlanningScene& scene,
    std::vector<data::Object>* objects) {
  return Convert(scene.objects, objects);
}

}  // namespace planning
}  // namespace excavator
