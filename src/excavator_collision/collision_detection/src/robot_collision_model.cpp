#include "excavator_collision/internal/robot_collision_model.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <unordered_set>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf/model.h>

namespace tramcar {
namespace planning {
namespace {

Eigen::Isometry3d ToTransform(const urdf::Pose& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

  Eigen::Quaterniond quaternion(pose.rotation.w, pose.rotation.x,
                                pose.rotation.y, pose.rotation.z);
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < 1.0e-12) {
    transform.setIdentity();
    return transform;
  }
  quaternion.normalize();
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

bool IsFinite(const Eigen::Isometry3d& transform) {
  return transform.matrix().allFinite();
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

  const std::string package_name = resource.substr(0, separator);
  const std::string relative_path = resource.substr(separator + 1);
  try {
    return ament_index_cpp::get_package_share_directory(package_name) + "/" +
           relative_path;
  } catch (const std::exception&) {
    return {};
  }
}

bool LoadStl(const std::string& path, const Eigen::Vector3d& scale,
             CollisionGeometry* geometry, std::string* error) {
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
    input.read(reinterpret_cast<char*>(&triangle_count),
               sizeof(triangle_count));
    binary_stl =
        input.good() &&
        static_cast<std::uint64_t>(file_size) ==
            84ULL + static_cast<std::uint64_t>(triangle_count) * 50ULL;
  }
  input.clear();
  input.seekg(0, std::ios::beg);

  if (binary_stl) {
    input.seekg(84, std::ios::beg);
    for (std::uint32_t index = 0; index < triangle_count; ++index) {
      input.seekg(12, std::ios::cur);
      const std::size_t first = geometry->mesh_vertices.size();
      for (int vertex = 0; vertex < 3; ++vertex) {
        float xyz[3] = {0.0F, 0.0F, 0.0F};
        input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
        if (!input) {
          *error = "truncated binary STL collision mesh: " + path;
          return false;
        }
        geometry->mesh_vertices.emplace_back(
            xyz[0] * scale.x(), xyz[1] * scale.y(), xyz[2] * scale.z());
      }
      geometry->mesh_triangles.push_back({first, first + 1, first + 2});
      input.seekg(2, std::ios::cur);
    }
  } else {
    std::string token;
    std::vector<Eigen::Vector3d> facet;
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
        const std::size_t first = geometry->mesh_vertices.size();
        geometry->mesh_vertices.insert(geometry->mesh_vertices.end(),
                                       facet.begin(), facet.end());
        geometry->mesh_triangles.push_back({first, first + 1, first + 2});
        facet.clear();
      }
    }

    if (!facet.empty()) {
      *error = "incomplete facet in ASCII STL collision mesh: " + path;
      return false;
    }
  }

  if (geometry->mesh_vertices.empty() || geometry->mesh_triangles.empty()) {
    *error = "collision mesh contains no triangles: " + path;
    return false;
  }

  for (const Eigen::Vector3d& vertex : geometry->mesh_vertices) {
    if (!vertex.allFinite()) {
      *error = "collision mesh contains non-finite vertices: " + path;
      return false;
    }
  }
  return true;
}

bool IsMovable(const urdf::JointConstSharedPtr& joint) {
  return joint && joint->type != urdf::Joint::FIXED;
}

ValidationResult ConvertGeometry(const urdf::GeometrySharedPtr& source,
                                  CollisionGeometry* destination) {
  if (!source || destination == nullptr) {
    return {false, "URDF collision geometry is null"};
  }

  switch (source->type) {
    case urdf::Geometry::BOX: {
      const auto box = std::dynamic_pointer_cast<urdf::Box>(source);
      if (!box || !std::isfinite(box->dim.x) ||
          !std::isfinite(box->dim.y) || !std::isfinite(box->dim.z) ||
          box->dim.x <= 0.0 || box->dim.y <= 0.0 || box->dim.z <= 0.0) {
        return {false, "invalid URDF box collision dimensions"};
      }
      destination->type = CollisionGeometryType::kBox;
      destination->dimensions =
          Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z);
      return {true, ""};
    }
    case urdf::Geometry::SPHERE: {
      const auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(source);
      if (!sphere || !std::isfinite(sphere->radius) ||
          sphere->radius <= 0.0) {
        return {false, "invalid URDF sphere collision radius"};
      }
      destination->type = CollisionGeometryType::kSphere;
      destination->dimensions.x() = sphere->radius;
      return {true, ""};
    }
    case urdf::Geometry::CYLINDER: {
      const auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(source);
      if (!cylinder || !std::isfinite(cylinder->radius) ||
          !std::isfinite(cylinder->length) || cylinder->radius <= 0.0 ||
          cylinder->length <= 0.0) {
        return {false, "invalid URDF cylinder collision dimensions"};
      }
      destination->type = CollisionGeometryType::kCylinder;
      destination->dimensions.x() = cylinder->radius;
      destination->dimensions.y() = cylinder->length;
      return {true, ""};
    }
    case urdf::Geometry::MESH: {
      const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(source);
      if (!mesh) {
        return {false, "invalid URDF mesh collision geometry"};
      }

      const std::string path = ResolveMeshPath(mesh->filename);
      if (path.empty()) {
        return {false, "cannot resolve collision mesh URI: " + mesh->filename};
      }

      const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
      if (!scale.allFinite() || (scale.array().abs() < 1.0e-12).any()) {
        return {false, "invalid URDF mesh scale: " + mesh->filename};
      }

      destination->type = CollisionGeometryType::kTriangleMesh;
      std::string error;
      if (!LoadStl(path, scale, destination, &error)) {
        return {false, error};
      }
      return {true, ""};
    }
    default:
      return {false, "unsupported URDF collision geometry type"};
  }
}

}  // namespace

ValidationResult RobotCollisionModel::InitializeFromUrdf(
    const std::string& urdf_xml,
    const std::vector<std::string>& planning_joint_names,
    const std::string& reference_frame) {
  root_frame_.clear();
  reference_frame_.clear();
  root_to_reference_frame_.setIdentity();
  joint_names_.clear();
  links_.clear();

  if (urdf_xml.empty() || planning_joint_names.empty()) {
    return {false, "URDF XML and planning joint names must not be empty"};
  }

  std::unordered_set<std::string> unique_names;
  for (const std::string& name : planning_joint_names) {
    if (name.empty() || !unique_names.insert(name).second) {
      return {false, "planning joint names must be non-empty and unique"};
    }
  }

  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    return {false, "failed to parse URDF XML for collision model"};
  }

  const auto root = model.getRoot();
  if (!root) {
    return {false, "URDF has no root link"};
  }
  if (reference_frame.empty()) {
    return {false, "kinematics reference frame must not be empty"};
  }

  // KinematicsModel poses are expressed in a declared reference frame. The
  // planning scene uses the URDF root frame, so compute the fixed transform
  // between these frames once when loading the collision model.
  const auto reference_link = model.getLink(reference_frame);
  if (!reference_link) {
    return {false, "kinematics reference frame is missing from URDF: " +
                       reference_frame};
  }

  std::vector<urdf::LinkConstSharedPtr> reference_chain;
  auto current = reference_link;
  while (current && current->name != root->name) {
    const auto& joint = current->parent_joint;
    if (!joint || joint->type != urdf::Joint::FIXED) {
      return {false, "kinematics reference frame must be connected to the "
                     "URDF root by fixed joints: " + reference_frame};
    }
    reference_chain.push_back(current);
    current = model.getLink(joint->parent_link_name);
  }
  if (!current) {
    return {false, "kinematics reference frame is not connected to URDF root: " +
                       reference_frame};
  }

  Eigen::Isometry3d root_to_reference =
      Eigen::Isometry3d::Identity();
  for (auto iter = reference_chain.rbegin(); iter != reference_chain.rend();
       ++iter) {
    root_to_reference =
        root_to_reference *
        ToTransform((*iter)->parent_joint->parent_to_joint_origin_transform);
  }
  if (!IsFinite(root_to_reference)) {
    return {false, "root-to-reference transform is non-finite"};
  }

  for (const std::string& joint_name : planning_joint_names) {
    if (!model.getJoint(joint_name)) {
      return {false, "planning joint is missing from URDF: " + joint_name};
    }
  }

  std::vector<urdf::LinkConstSharedPtr> stack{root};
  while (!stack.empty()) {
    const auto link = stack.back();
    stack.pop_back();

    LinkCollisionGeometry link_geometry;
    link_geometry.link_name = link->name;
    link_geometry.parent_link_name =
        link->parent_joint ? link->parent_joint->parent_link_name : "";

    std::vector<urdf::CollisionSharedPtr> collisions = link->collision_array;
    if (collisions.empty() && link->collision) {
      collisions.push_back(link->collision);
    }

    for (const auto& collision : collisions) {
      if (!collision) {
        return {false, "null collision element in URDF link: " + link->name};
      }

      CollisionGeometryInstance instance;
      const ValidationResult converted =
          ConvertGeometry(collision->geometry, &instance.geometry);
      if (!converted.ok) {
        return {false, "failed to load collision geometry for link " +
                           link->name + ": " + converted.message};
      }

      instance.link_to_collision = ToTransform(collision->origin);
      if (!IsFinite(instance.link_to_collision)) {
        return {false, "non-finite collision origin for link: " + link->name};
      }
      link_geometry.geometries.push_back(std::move(instance));
    }

    if (!link_geometry.geometries.empty()) {
      auto current = link;
      while (current && current->parent_joint) {
        const auto& joint = current->parent_joint;
        if (IsMovable(joint) &&
            unique_names.find(joint->name) == unique_names.end()) {
          return {false, "collision-bearing link " + link->name +
                             " depends on unconfigured movable joint " +
                             joint->name};
        }
        current = model.getLink(joint->parent_link_name);
      }
      links_.push_back(std::move(link_geometry));
    }

    for (const auto& child : link->child_links) {
      stack.push_back(child);
    }
  }

  if (links_.empty()) {
    return {false, "URDF has no supported collision geometry"};
  }

  root_frame_ = root->name;
  reference_frame_ = reference_frame;
  root_to_reference_frame_ = root_to_reference;
  joint_names_ = planning_joint_names;
  return {true, ""};
}

}  // namespace planning
}  // namespace tramcar
