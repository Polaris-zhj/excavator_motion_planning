#include "excavator_collision/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fcl/fcl.h>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <urdf/model.h>

namespace tramcar {
namespace planning {
namespace {

using Transform = Eigen::Isometry3d;
using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using GeometryPtr = std::shared_ptr<fcl::CollisionGeometryd>;

Transform ToTransform(const urdf::Pose& pose) {
  Transform transform = Transform::Identity();
  transform.translation() =
      Vector3(pose.position.x, pose.position.y, pose.position.z);
  transform.linear() =
      Quaternion(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                 pose.rotation.z).normalized().toRotationMatrix();
  return transform;
}

Transform ToTransform(const geometry_msgs::msg::Pose& pose) {
  Transform transform = Transform::Identity();
  transform.translation() =
      Vector3(pose.position.x, pose.position.y, pose.position.z);
  Quaternion quaternion(pose.orientation.w, pose.orientation.x,
                        pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-12) {
    return Transform::Identity();
  }
  transform.linear() = quaternion.normalized().toRotationMatrix();
  return transform;
}

bool IsFinite(const Transform& transform) {
  return transform.matrix().allFinite();
}

bool IsFinitePose(const geometry_msgs::msg::Pose& pose) {
  const Quaternion quaternion(pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z);
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && quaternion.coeffs().allFinite() &&
         quaternion.norm() > 1.0e-12;
}

std::string ResolveMeshPath(const std::string& uri) {
  const std::string prefix = "package://";
  if (uri.compare(0, prefix.size(), prefix) != 0) {
    return uri;
  }
  const std::string resource = uri.substr(prefix.size());
  const std::size_t separator = resource.find('/');
  if (separator == std::string::npos) {
    return {};
  }
  const std::string package = resource.substr(0, separator);
  const std::string relative_path = resource.substr(separator + 1);
  try {
    return ament_index_cpp::get_package_share_directory(package) + "/" +
           relative_path;
  } catch (const std::exception&) {
    return {};
  }
}

bool LoadStl(const std::string& path, const Vector3& scale,
             std::vector<fcl::Vector3d>* vertices,
             std::vector<fcl::Triangle>* triangles, std::string* error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    *error = "cannot open collision mesh: " + path;
    return false;
  }
  input.seekg(0, std::ios::end);
  const std::streamoff file_size = input.tellg();
  input.seekg(0, std::ios::beg);
  if (file_size < 0) {
    *error = "cannot determine collision mesh size: " + path;
    return false;
  }

  bool binary_stl = false;
  std::uint32_t triangle_count = 0;
  if (file_size >= 84) {
    input.seekg(80, std::ios::beg);
    input.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
    binary_stl = input.good() &&
        static_cast<std::uint64_t>(file_size) ==
            84ULL + static_cast<std::uint64_t>(triangle_count) * 50ULL;
  }
  input.clear();
  input.seekg(0, std::ios::beg);

  if (binary_stl) {
    input.seekg(84, std::ios::beg);
    for (std::uint32_t i = 0; i < triangle_count; ++i) {
      input.seekg(12, std::ios::cur);  // facet normal
      const std::size_t first = vertices->size();
      for (int vertex = 0; vertex < 3; ++vertex) {
        float xyz[3]{};
        input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
        if (!input) {
          *error = "truncated binary STL collision mesh: " + path;
          return false;
        }
        vertices->emplace_back(xyz[0] * scale.x(), xyz[1] * scale.y(),
                               xyz[2] * scale.z());
      }
      triangles->emplace_back(static_cast<int>(first),
                              static_cast<int>(first + 1),
                              static_cast<int>(first + 2));
      input.seekg(2, std::ios::cur);
    }
  } else {
    std::string token;
    std::vector<Vector3> facet;
    while (input >> token) {
      if (token != "vertex") {
        continue;
      }
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!(input >> x >> y >> z)) {
        *error = "malformed ASCII STL collision mesh: " + path;
        return false;
      }
      facet.emplace_back(x * scale.x(), y * scale.y(), z * scale.z());
      if (facet.size() == 3) {
        const std::size_t first = vertices->size();
        for (const Vector3& point : facet) {
          vertices->emplace_back(point.x(), point.y(), point.z());
        }
        triangles->emplace_back(static_cast<int>(first),
                                static_cast<int>(first + 1),
                                static_cast<int>(first + 2));
        facet.clear();
      }
    }
    if (!facet.empty()) {
      *error = "incomplete facet in ASCII STL collision mesh: " + path;
      return false;
    }
  }
  if (vertices->empty() || triangles->empty()) {
    *error = "collision mesh contains no triangles: " + path;
    return false;
  }
  return true;
}

GeometryPtr LoadGeometry(const urdf::GeometrySharedPtr& geometry,
                         std::string* error) {
  if (!geometry) {
    *error = "URDF collision element has no geometry";
    return {};
  }
  switch (geometry->type) {
    case urdf::Geometry::BOX: {
      const auto box = std::dynamic_pointer_cast<urdf::Box>(geometry);
      if (!box || box->dim.x <= 0.0 || box->dim.y <= 0.0 ||
          box->dim.z <= 0.0) {
        *error = "invalid URDF box collision dimensions";
        return {};
      }
      return std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
    }
    case urdf::Geometry::SPHERE: {
      const auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
      if (!sphere || sphere->radius <= 0.0) {
        *error = "invalid URDF sphere collision radius";
        return {};
      }
      return std::make_shared<fcl::Sphered>(sphere->radius);
    }
    case urdf::Geometry::CYLINDER: {
      const auto cylinder =
          std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
      if (!cylinder || cylinder->radius <= 0.0 || cylinder->length <= 0.0) {
        *error = "invalid URDF cylinder collision dimensions";
        return {};
      }
      return std::make_shared<fcl::Cylinderd>(cylinder->radius,
                                               cylinder->length);
    }
    case urdf::Geometry::MESH: {
      const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
      if (!mesh) {
        *error = "invalid URDF mesh collision geometry";
        return {};
      }
      const std::string path = ResolveMeshPath(mesh->filename);
      if (path.empty()) {
        *error = "cannot resolve collision mesh URI: " + mesh->filename;
        return {};
      }
      std::vector<fcl::Vector3d> vertices;
      std::vector<fcl::Triangle> triangles;
      if (!LoadStl(path, Vector3(mesh->scale.x, mesh->scale.y, mesh->scale.z),
                   &vertices, &triangles, error)) {
        return {};
      }
      auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
      if (model->beginModel() != fcl::BVH_OK ||
          model->addSubModel(vertices, triangles) != fcl::BVH_OK ||
          model->endModel() != fcl::BVH_OK) {
        *error = "FCL failed to build BVH for collision mesh: " + path;
        return {};
      }
      return model;
    }
    default:
      *error = "unsupported URDF collision geometry type";
      return {};
  }
}

bool IsMovable(const urdf::JointConstSharedPtr& joint) {
  return joint && joint->type != urdf::Joint::FIXED;
}

Transform JointMotion(const urdf::JointConstSharedPtr& joint, double value) {
  Transform motion = Transform::Identity();
  const Vector3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
  if (joint->type == urdf::Joint::REVOLUTE ||
      joint->type == urdf::Joint::CONTINUOUS) {
    if (axis.norm() > 1.0e-12) {
      motion.linear() =
          Eigen::AngleAxisd(value, axis.normalized()).toRotationMatrix();
    }
  } else if (joint->type == urdf::Joint::PRISMATIC) {
    if (axis.norm() > 1.0e-12) {
      motion.translation() = axis.normalized() * value;
    }
  }
  return motion;
}

}  // namespace

struct UrdfFclCollisionChecker::Impl {
  struct LinkGeometry {
    std::string name;
    std::string parent_link;
    urdf::LinkConstSharedPtr link;
    std::vector<std::pair<Transform, GeometryPtr>> geometries;
  };

  urdf::Model model;
  std::vector<std::string> joint_names;
  std::unordered_map<std::string, std::size_t> joint_indices;
  std::vector<LinkGeometry> links;
  bool initialized{false};

  bool ComputeLinkTransforms(
      const JointPositions& positions,
      std::unordered_map<std::string, Transform>* transforms,
      std::string* error) const {
    if (positions.size() != joint_names.size()) {
      *error = "joint position count does not match configured joint group";
      return false;
    }
    std::unordered_map<std::string, double> joint_values;
    for (std::size_t i = 0; i < positions.size(); ++i) {
      if (!std::isfinite(positions[i])) {
        *error = "joint positions must be finite";
        return false;
      }
      joint_values.emplace(joint_names[i], positions[i]);
    }

    const auto root = model.getRoot();
    if (!root) {
      *error = "URDF has no root link";
      return false;
    }
    transforms->clear();
    transforms->emplace(root->name, Transform::Identity());
    std::vector<urdf::LinkConstSharedPtr> stack{root};
    while (!stack.empty()) {
      const auto parent = stack.back();
      stack.pop_back();
      const Transform parent_transform = transforms->at(parent->name);
      for (const auto& child : parent->child_links) {
        const auto joint = child->parent_joint;
        if (!joint) {
          *error = "URDF child link has no parent joint: " + child->name;
          return false;
        }
        double value = 0.0;
        if (IsMovable(joint)) {
          const auto found = joint_values.find(joint->name);
          if (found != joint_values.end()) {
            value = found->second;
          } else if (!joint->mimic) {
            // Non-planning joints are held at zero. This is suitable for the
            // current fixed/visual-only auxiliary links; callers should add
            // them to the planning group if their motion affects collision.
            value = 0.0;
          }
        }
        const Transform world_child =
            parent_transform * ToTransform(joint->parent_to_joint_origin_transform) *
            JointMotion(joint, value);
        if (!IsFinite(world_child)) {
          *error = "non-finite transform for link: " + child->name;
          return false;
        }
        transforms->insert_or_assign(child->name, world_child);
        stack.push_back(child);
      }
    }
    return true;
  }

  bool BuildEnvironmentGeometry(
      const excavator_interfaces::msg::PlanningScene& scene,
      std::vector<std::pair<std::string, std::pair<Transform, GeometryPtr>>>*
          objects,
      std::string* error) const {
    objects->clear();
    const auto root = model.getRoot();
    if (!scene.header.frame_id.empty() &&
        (!root || scene.header.frame_id != root->name)) {
      *error = "planning scene frame_id must be empty or the URDF root frame (" +
               (root ? root->name : std::string("unknown")) + ")";
      return false;
    }
    std::unordered_set<std::string> ids;
    for (const auto& object : scene.objects) {
      if (object.id.empty() || !ids.insert(object.id).second) {
        *error = "planning scene object IDs must be non-empty and unique";
        return false;
      }
      if (!object.enabled) {
        continue;
      }
      GeometryPtr geometry;
      const auto& primitive = object.primitive;
      switch (primitive.type) {
        case shape_msgs::msg::SolidPrimitive::BOX:
          if (primitive.dimensions.size() != 3 ||
              primitive.dimensions[0] <= 0.0 ||
              primitive.dimensions[1] <= 0.0 ||
              primitive.dimensions[2] <= 0.0) {
            *error = "environment box requires three positive dimensions: " +
                     object.id;
            return false;
          }
          geometry = std::make_shared<fcl::Boxd>(
              primitive.dimensions[0], primitive.dimensions[1],
              primitive.dimensions[2]);
          break;
        case shape_msgs::msg::SolidPrimitive::SPHERE:
          if (primitive.dimensions.size() != 1 ||
              primitive.dimensions[0] <= 0.0) {
            *error = "environment sphere requires one positive radius: " +
                     object.id;
            return false;
          }
          geometry =
              std::make_shared<fcl::Sphered>(primitive.dimensions[0]);
          break;
        case shape_msgs::msg::SolidPrimitive::CYLINDER:
          if (primitive.dimensions.size() != 2 ||
              primitive.dimensions[0] <= 0.0 ||
              primitive.dimensions[1] <= 0.0) {
            *error = "environment cylinder requires positive height and radius: " +
                     object.id;
            return false;
          }
          geometry = std::make_shared<fcl::Cylinderd>(
              primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
              primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT]);
          break;
        default:
          *error = "unsupported environment primitive type for object: " +
                   object.id;
          return false;
      }
      if (!IsFinitePose(object.pose)) {
        *error = "invalid environment object pose: " + object.id;
        return false;
      }
      const Transform pose = ToTransform(object.pose);
      objects->push_back({object.id, {pose, std::move(geometry)}});
    }
    return true;
  }

  CollisionCheckResult Check(
      const JointPositions& positions,
      const excavator_interfaces::msg::PlanningScene& scene) const {
    CollisionCheckResult result;
    if (!initialized) {
      result.message = "collision checker has not been initialized";
      return result;
    }
    std::unordered_map<std::string, Transform> transforms;
    if (!ComputeLinkTransforms(positions, &transforms, &result.message)) {
      return result;
    }
    std::vector<std::pair<std::string, std::pair<Transform, GeometryPtr>>>
        environment;
    if (!BuildEnvironmentGeometry(scene, &environment, &result.message)) {
      return result;
    }

    struct WorldGeometry {
      std::string link;
      std::string parent;
      std::shared_ptr<fcl::CollisionObjectd> object;
    };
    std::vector<WorldGeometry> robot_geometry;
    for (const auto& link : links) {
      const auto transform = transforms.find(link.name);
      if (transform == transforms.end()) {
        result.message = "no computed transform for collision link: " + link.name;
        return result;
      }
      for (const auto& local_geometry : link.geometries) {
        auto object = std::make_shared<fcl::CollisionObjectd>(
            local_geometry.second,
            transform->second * local_geometry.first);
        object->computeAABB();
        robot_geometry.push_back({link.name, link.parent_link, object});
      }
    }

    for (const auto& robot : robot_geometry) {
      for (const auto& obstacle : environment) {
        fcl::CollisionObjectd obstacle_object(
            obstacle.second.second, obstacle.second.first);
        obstacle_object.computeAABB();
        fcl::CollisionRequestd request;
        request.num_max_contacts = 1;
        fcl::CollisionResultd collision_result;
        fcl::collide(robot.object.get(), &obstacle_object, request,
                     collision_result);
        if (collision_result.isCollision()) {
          result.valid = true;
          result.collision = true;
          result.first_link = robot.link;
          result.second_object = obstacle.first;
          result.message = "environment collision: " + robot.link + " with " +
                           obstacle.first;
          return result;
        }
      }
    }

    for (std::size_t i = 0; i < robot_geometry.size(); ++i) {
      for (std::size_t j = i + 1; j < robot_geometry.size(); ++j) {
        const auto& first = robot_geometry[i];
        const auto& second = robot_geometry[j];
        if (first.link == second.link || first.parent == second.link ||
            second.parent == first.link) {
          continue;
        }
        fcl::CollisionRequestd request;
        request.num_max_contacts = 1;
        fcl::CollisionResultd collision_result;
        fcl::collide(first.object.get(), second.object.get(), request,
                     collision_result);
        if (collision_result.isCollision()) {
          result.valid = true;
          result.collision = true;
          result.first_link = first.link;
          result.second_object = second.link;
          result.message = "self collision: " + first.link + " with " +
                           second.link;
          return result;
        }
      }
    }
    result.valid = true;
    result.collision = false;
    result.message = "no collision detected";
    result.checked_states = 1.0;
    return result;
  }
};

UrdfFclCollisionChecker::UrdfFclCollisionChecker()
    : impl_(std::make_unique<Impl>()) {}
UrdfFclCollisionChecker::~UrdfFclCollisionChecker() = default;

ValidationResult UrdfFclCollisionChecker::Initialize(
    const std::string& urdf_xml, std::vector<std::string> joint_names) {
  if (urdf_xml.empty() || joint_names.empty()) {
    return {false, "URDF XML and planning joint names must not be empty"};
  }
  std::unordered_set<std::string> unique_names;
  for (const auto& name : joint_names) {
    if (name.empty() || !unique_names.insert(name).second) {
      return {false, "planning joint names must be non-empty and unique"};
    }
  }
  if (!impl_->model.initString(urdf_xml)) {
    return {false, "failed to parse URDF XML for collision checking"};
  }
  impl_->joint_names = std::move(joint_names);
  impl_->joint_indices.clear();
  for (std::size_t i = 0; i < impl_->joint_names.size(); ++i) {
    if (!impl_->model.getJoint(impl_->joint_names[i])) {
      return {false, "planning joint is missing from URDF: " +
                         impl_->joint_names[i]};
    }
    impl_->joint_indices.emplace(impl_->joint_names[i], i);
  }

  impl_->links.clear();
  std::vector<urdf::LinkConstSharedPtr> stack;
  const auto root = impl_->model.getRoot();
  if (!root) {
    return {false, "URDF has no root link"};
  }
  stack.push_back(root);
  while (!stack.empty()) {
    const auto link = stack.back();
    stack.pop_back();
    Impl::LinkGeometry item;
    item.name = link->name;
    item.link = link;
    item.parent_link = link->parent_joint ? link->parent_joint->parent_link_name : "";
    std::vector<urdf::CollisionSharedPtr> collisions = link->collision_array;
    if (collisions.empty() && link->collision) {
      collisions.push_back(link->collision);
    }
    for (const auto& collision : collisions) {
      if (!collision) {
        continue;
      }
      std::string error;
      GeometryPtr geometry = LoadGeometry(collision->geometry, &error);
      if (!geometry) {
        return {false, "failed to load collision geometry for link " +
                           link->name + ": " + error};
      }
      item.geometries.push_back({ToTransform(collision->origin),
                                 std::move(geometry)});
    }
    if (!item.geometries.empty()) {
      impl_->links.push_back(std::move(item));
    }
    for (const auto& child : link->child_links) {
      stack.push_back(child);
    }
  }
  impl_->initialized = true;
  return {true, ""};
}

CollisionCheckResult UrdfFclCollisionChecker::CheckState(
    const JointPositions& positions,
    const excavator_interfaces::msg::PlanningScene& scene) const {
  return impl_->Check(positions, scene);
}

CollisionCheckResult UrdfFclCollisionChecker::CheckPath(
    const JointPath& path,
    const excavator_interfaces::msg::PlanningScene& scene,
    double max_joint_step) const {
  CollisionCheckResult result;
  if (path.points.empty() || path.joint_names != impl_->joint_names ||
      !std::isfinite(max_joint_step) || max_joint_step <= 0.0) {
    result.message = "invalid path, joint names, or max_joint_step";
    return result;
  }
  std::size_t checked = 0;
  for (std::size_t segment = 0; segment < path.points.size(); ++segment) {
    const auto& start = path.points[segment];
    if (start.size() != impl_->joint_names.size()) {
      result.message = "path waypoint joint count mismatch";
      return result;
    }
    if (segment + 1 == path.points.size()) {
      result = impl_->Check(start, scene);
      ++checked;
      if (!result.valid || result.collision) {
        result.checked_states = static_cast<double>(checked);
        return result;
      }
      break;
    }
    const auto& end = path.points[segment + 1];
    if (end.size() != start.size()) {
      result.message = "path waypoint joint count mismatch";
      return result;
    }
    double max_delta = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint) {
      if (!std::isfinite(start[joint]) || !std::isfinite(end[joint])) {
        result.message = "path contains non-finite joint position";
        return result;
      }
      max_delta = std::max(max_delta, std::abs(end[joint] - start[joint]));
    }
    const std::size_t subdivisions =
        std::max<std::size_t>(1, static_cast<std::size_t>(
                                     std::ceil(max_delta / max_joint_step)));
    for (std::size_t sample = 0; sample < subdivisions; ++sample) {
      const double alpha = static_cast<double>(sample) /
                           static_cast<double>(subdivisions);
      JointPositions positions(start.size());
      for (std::size_t joint = 0; joint < start.size(); ++joint) {
        positions[joint] = start[joint] + alpha * (end[joint] - start[joint]);
      }
      result = impl_->Check(positions, scene);
      ++checked;
      if (!result.valid || result.collision) {
        result.checked_states = static_cast<double>(checked);
        return result;
      }
    }
  }
  result.valid = true;
  result.collision = false;
  result.message = "no collision detected at sampled path states";
  result.checked_states = static_cast<double>(checked);
  return result;
}

CollisionCheckResult UrdfFclCollisionChecker::CheckTrajectory(
    const JointTrajectory& trajectory,
    const excavator_interfaces::msg::PlanningScene& scene,
    double max_joint_step) const {
  CollisionCheckResult result;
  if (trajectory.points.empty() ||
      trajectory.joint_names != impl_->joint_names ||
      !std::isfinite(max_joint_step) || max_joint_step <= 0.0) {
    result.message = "invalid trajectory, joint names, or max_joint_step";
    return result;
  }
  JointPath path;
  path.joint_names = trajectory.joint_names;
  for (const auto& point : trajectory.points) {
    path.points.push_back(point.positions);
  }
  return CheckPath(path, scene, max_joint_step);
}

}  // namespace planning
}  // namespace tramcar
