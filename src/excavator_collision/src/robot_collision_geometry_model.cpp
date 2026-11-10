#include "excavator_collision/robot_collision_geometry_model.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace excavator {
namespace planning {
namespace {

bool IsFinite(const Eigen::Isometry3d& transform) {
  return transform.matrix().allFinite();
}

bool ReadTransform(const YAML::Node& node, Eigen::Isometry3d* transform,
                   std::string* error) {
  *transform = Eigen::Isometry3d::Identity();
  if (!node) {
    return true;
  }
  if (!node.IsMap()) {
    *error = "origin must be a map with xyz and rpy";
    return false;
  }

  std::vector<double> xyz{0.0, 0.0, 0.0};
  std::vector<double> rpy{0.0, 0.0, 0.0};
  try {
    if (node["xyz"]) {
      xyz = node["xyz"].as<std::vector<double>>();
    }
    if (node["rpy"]) {
      rpy = node["rpy"].as<std::vector<double>>();
    }
  } catch (const std::exception& exception) {
    *error = std::string("invalid origin: ") + exception.what();
    return false;
  }
  if (xyz.size() != 3 || rpy.size() != 3) {
    *error = "origin xyz and rpy must each contain three values";
    return false;
  }

  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  transform->linear() = rotation;
  transform->translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  if (!IsFinite(*transform)) {
    *error = "origin contains non-finite values";
    return false;
  }
  return true;
}

bool ReadVector3(const YAML::Node& node, const char* key,
                 const Eigen::Vector3d& default_value, Eigen::Vector3d* value,
                 std::string* error) {
  *value = default_value;
  if (!node[key]) {
    return true;
  }
  try {
    const std::vector<double> values = node[key].as<std::vector<double>>();
    if (values.size() != 3) {
      *error = std::string("'") + key + "' must contain three values";
      return false;
    }
    *value = Eigen::Vector3d(values[0], values[1], values[2]);
  } catch (const std::exception& exception) {
    *error = std::string("invalid '") + key + "': " + exception.what();
    return false;
  }
  if (!value->allFinite()) {
    *error = std::string("'") + key + "' must be finite";
    return false;
  }
  return true;
}

bool ReadPositiveDouble(const YAML::Node& node, const char* key, double* value,
                        std::string* error) {
  if (!node[key]) {
    *error = std::string("missing required '") + key + "'";
    return false;
  }
  try {
    *value = node[key].as<double>();
  } catch (const std::exception& exception) {
    *error = std::string("invalid '") + key + "': " + exception.what();
    return false;
  }
  if (!std::isfinite(*value) || *value <= 0.0) {
    *error = std::string("'") + key + "' must be finite and positive";
    return false;
  }
  return true;
}

std::string ResolveMeshPath(const std::string& uri,
                            const std::string& base_directory) {
  const std::string prefix = "package://";
  if (uri.compare(0, prefix.size(), prefix) == 0) {
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

  if (!uri.empty() && uri.front() == '/') {
    return uri;
  }
  if (!base_directory.empty()) {
    return base_directory + "/" + uri;
  }
  return uri;
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

ValidationResult ParseGeometry(const YAML::Node& node,
                               const std::string& context,
                               const std::string& base_directory,
                               CollisionGeometryInstance* instance) {
  if (!node || !node.IsMap()) {
    return {false, context + ": geometry must be a map"};
  }

  std::string error;
  if (!ReadTransform(node["origin"], &instance->link_to_collision, &error)) {
    return {false, context + ": " + error};
  }

  std::string type;
  try {
    type = node["type"].as<std::string>();
  } catch (const std::exception& exception) {
    return {false, context + ": missing or invalid type: " + exception.what()};
  }

  CollisionGeometry& geometry = instance->geometry;
  if (type == "box") {
    Eigen::Vector3d size;
    if (!ReadVector3(node, "size", Eigen::Vector3d::Zero(), &size, &error)) {
      return {false, context + ": " + error};
    }
    if ((size.array() <= 0.0).any()) {
      return {false, context + ": box size must be positive"};
    }
    geometry.type = CollisionGeometryType::kBox;
    geometry.dimensions = size;
    return {true, ""};
  }

  if (type == "sphere") {
    double radius = 0.0;
    if (!ReadPositiveDouble(node, "radius", &radius, &error)) {
      return {false, context + ": " + error};
    }
    geometry.type = CollisionGeometryType::kSphere;
    geometry.dimensions.x() = radius;
    return {true, ""};
  }

  if (type == "cylinder" || type == "capsule") {
    double radius = 0.0;
    double length = 0.0;
    if (!ReadPositiveDouble(node, "radius", &radius, &error) ||
        !ReadPositiveDouble(node, "length", &length, &error)) {
      return {false, context + ": " + error};
    }
    geometry.type = type == "cylinder" ? CollisionGeometryType::kCylinder
                                       : CollisionGeometryType::kCapsule;
    geometry.dimensions.x() = radius;
    geometry.dimensions.y() = length;
    return {true, ""};
  }

  if (type == "mesh") {
    std::string resource;
    try {
      resource = node["resource"].as<std::string>();
    } catch (const std::exception& exception) {
      return {false, context + ": invalid mesh resource: " + exception.what()};
    }
    if (resource.empty()) {
      return {false, context + ": mesh resource must not be empty"};
    }

    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    if (!ReadVector3(node, "scale", Eigen::Vector3d::Ones(), &scale, &error)) {
      return {false, context + ": " + error};
    }
    if ((scale.array().abs() < 1.0e-12).any()) {
      return {false, context + ": mesh scale must not contain zero"};
    }

    const std::string path = ResolveMeshPath(resource, base_directory);
    if (path.empty()) {
      return {false, context + ": cannot resolve mesh resource: " + resource};
    }

    geometry.type = CollisionGeometryType::kTriangleMesh;
    if (!LoadStl(path, scale, &geometry, &error)) {
      return {false, context + ": " + error};
    }
    return {true, ""};
  }

  return {false, context + ": unsupported geometry type '" + type + "'"};
}

}  // namespace

ValidationResult RobotCollisionGeometryModel::InitializeFromYaml(
    const std::string& yaml,
    const std::vector<std::string>& planning_joint_names,
    const std::string& reference_frame,
    const std::string& base_directory) {
  root_frame_.clear();
  reference_frame_.clear();
  joint_names_.clear();
  links_.clear();

  if (yaml.empty() || planning_joint_names.empty()) {
    return {false, "collision YAML and planning joint names must not be empty"};
  }

  std::unordered_set<std::string> unique_joints;
  for (const std::string& name : planning_joint_names) {
    if (name.empty() || !unique_joints.insert(name).second) {
      return {false, "planning joint names must be non-empty and unique"};
    }
  }

  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const std::exception& exception) {
    return {false, std::string("failed to parse collision YAML: ") +
                       exception.what()};
  }
  if (!root || !root.IsMap()) {
    return {false, "collision YAML root must be a map"};
  }

  std::string root_frame = reference_frame;
  if (root["robot"]) {
    const YAML::Node robot = root["robot"];
    if (!robot.IsMap()) {
      return {false, "robot must be a map"};
    }
    if (robot["root_frame"]) {
      try {
        root_frame = robot["root_frame"].as<std::string>();
      } catch (const std::exception& exception) {
        return {false, std::string("invalid robot.root_frame: ") +
                           exception.what()};
      }
    }
  }
  if (root_frame.empty()) {
    root_frame = "base_link";
  }
  // Without a URDF there is no implicit frame tree. Kinematics output and
  // scene poses must use exactly the frame declared by the collision model.
  if (!reference_frame.empty() && root_frame != reference_frame) {
    return {false, "collision model root frame '" + root_frame +
                       "' does not match kinematics reference frame '" +
                       reference_frame + "'"};
  }

  const YAML::Node links_node = root["links"];
  if (!links_node || !links_node.IsSequence() || links_node.size() == 0) {
    return {false, "collision YAML must contain a non-empty links sequence"};
  }

  std::vector<LinkCollisionGeometry> links;
  std::unordered_set<std::string> unique_links;
  for (std::size_t link_index = 0; link_index < links_node.size();
       ++link_index) {
    const YAML::Node link_node = links_node[link_index];
    if (!link_node || !link_node.IsMap()) {
      return {false, "each links entry must be a map"};
    }

    LinkCollisionGeometry link;
    try {
      link.link_name = link_node["link_name"].as<std::string>();
    } catch (const std::exception& exception) {
      return {false, std::string("links entry: ") + exception.what()};
    }
    if (link.link_name.empty()) {
      return {false, "link_name must not be empty"};
    }
    if (!unique_links.insert(link.link_name).second) {
      return {false, "duplicate collision link name: " + link.link_name};
    }

    const YAML::Node parent_node =
        link_node["parent_link_name"] ? link_node["parent_link_name"]
                                      : link_node["parent"];
    if (parent_node) {
      try {
        link.parent_link_name = parent_node.as<std::string>();
      } catch (const std::exception& exception) {
        return {false, link.link_name + ": invalid parent link: " +
                           exception.what()};
      }
    }

    const YAML::Node geometries = link_node["geometries"];
    if (!geometries || !geometries.IsSequence() || geometries.size() == 0) {
      return {false, link.link_name +
                         ": geometries must be a non-empty sequence"};
    }

    for (std::size_t geometry_index = 0; geometry_index < geometries.size();
         ++geometry_index) {
      const YAML::Node geometry_node = geometries[geometry_index];
      std::string context = link.link_name;
      if (geometry_node && geometry_node["name"]) {
        context += "." + geometry_node["name"].as<std::string>();
      } else {
        context += ".geometry[" + std::to_string(geometry_index) + "]";
      }

      CollisionGeometryInstance instance;
      const ValidationResult parsed =
          ParseGeometry(geometry_node, context, base_directory, &instance);
      if (!parsed.ok) {
        return parsed;
      }
      link.geometries.push_back(std::move(instance));
    }

    links.push_back(std::move(link));
  }

  if (links.empty()) {
    return {false, "collision YAML contains no usable geometry"};
  }

  root_frame_ = root_frame;
  reference_frame_ = root_frame;
  joint_names_ = planning_joint_names;
  links_ = std::move(links);
  return {true, ""};
}

ValidationResult RobotCollisionGeometryModel::InitializeFromYamlFile(
    const std::string& path,
    const std::vector<std::string>& planning_joint_names,
    const std::string& reference_frame) {
  if (path.empty()) {
    return {false, "collision YAML path must not be empty"};
  }
  std::ifstream file(path);
  if (!file) {
    return {false, "failed to open collision YAML: " + path};
  }
  std::stringstream buffer;
  buffer << file.rdbuf();

  const std::size_t separator = path.find_last_of('/');
  const std::string base_directory =
      separator == std::string::npos ? std::string()
                                     : path.substr(0, separator);
  return InitializeFromYaml(buffer.str(), planning_joint_names,
                            reference_frame, base_directory);
}

}  // namespace planning
}  // namespace excavator
