#include "kinematics/dh_parameters.hpp"

#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace tramcar {
namespace planning {
namespace {

bool IsFinite(const Eigen::Isometry3d& transform) {
  return transform.matrix().allFinite();
}

bool ReadTransform(const YAML::Node& node, Eigen::Isometry3d* transform,
                   std::string* error) {
  if (transform == nullptr) {
    return false;
  }
  *transform = Eigen::Isometry3d::Identity();
  if (!node || !node.IsMap()) {
    if (error != nullptr) {
      *error = "transform must be a map with xyz and rpy";
    }
    return false;
  }
  std::vector<double> xyz{0.0, 0.0, 0.0};
  std::vector<double> rpy{0.0, 0.0, 0.0};
  if (node["xyz"]) {
    xyz = node["xyz"].as<std::vector<double>>();
  }
  if (node["rpy"]) {
    rpy = node["rpy"].as<std::vector<double>>();
  }
  if (xyz.size() != 3 || rpy.size() != 3) {
    if (error != nullptr) {
      *error = "xyz and rpy must each contain three values";
    }
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
    if (error != nullptr) {
      *error = "transform contains non-finite values";
    }
    return false;
  }
  return true;
}

bool ReadDouble(const YAML::Node& node, const char* key, double default_value,
                double* value, std::string* error) {
  if (value == nullptr) {
    return false;
  }
  if (!node[key]) {
    *value = default_value;
    return true;
  }
  try {
    *value = node[key].as<double>();
  } catch (const std::exception& exception) {
    if (error != nullptr) {
      *error = std::string("invalid '") + key + "': " + exception.what();
    }
    return false;
  }
  if (!std::isfinite(*value)) {
    if (error != nullptr) {
      *error = std::string("'") + key + "' must be finite";
    }
    return false;
  }
  return true;
}

}  // namespace

ValidationResult DhParameters::LoadFromFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return {false, "failed to open DH parameter file: " + path};
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return LoadFromString(buffer.str());
}

ValidationResult DhParameters::LoadFromString(const std::string& yaml) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const std::exception& exception) {
    return {false, std::string("failed to parse DH YAML: ") + exception.what()};
  }
  if (!root || !root.IsMap()) {
    return {false, "DH YAML root must be a map"};
  }
  if (root["convention"] &&
      root["convention"].as<std::string>() != "modified_dh") {
    return {false, "only the modified_dh convention is supported"};
  }

  std::string error;
  Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
  if (root["base"] && !ReadTransform(root["base"], &base, &error)) {
    return {false, "base: " + error};
  }
  if (!IsFinite(base)) {
    return {false, "base transform contains non-finite values"};
  }

  const YAML::Node joints_node = root["joints"];
  if (!joints_node || !joints_node.IsSequence() || joints_node.size() == 0) {
    return {false, "DH YAML must contain a non-empty joints sequence"};
  }

  std::vector<DhJoint> joints;
  std::set<std::string> names;
  std::set<std::string> links;
  for (std::size_t index = 0; index < joints_node.size(); ++index) {
    const YAML::Node node = joints_node[index];
    if (!node.IsMap()) {
      return {false, "each DH joint must be a map"};
    }
    DhJoint joint;
    try {
      joint.name = node["name"].as<std::string>();
      joint.link = node["link"].as<std::string>();
    } catch (const std::exception& exception) {
      return {false, std::string("joint: ") + exception.what()};
    }
    if (joint.name.empty() || joint.link.empty()) {
      return {false, "DH joints need non-empty name and link"};
    }
    if (!names.insert(joint.name).second) {
      return {false, "duplicate DH joint name: " + joint.name};
    }
    if (!links.insert(joint.link).second) {
      return {false, "duplicate DH link name: " + joint.link};
    }
    if (!ReadDouble(node, "theta_sign", 1.0, &joint.theta_sign, &error) ||
        !ReadDouble(node, "alpha_prev", 0.0, &joint.alpha_prev, &error) ||
        !ReadDouble(node, "a_prev", 0.0, &joint.a_prev, &error) ||
        !ReadDouble(node, "d", 0.0, &joint.d, &error) ||
        !ReadDouble(node, "theta_offset", 0.0, &joint.theta_offset, &error) ||
        !ReadDouble(node, "velocity", 1.0, &joint.velocity, &error)) {
      return {false, joint.name + ": " + error};
    }
    if (!node["limits"] || !node["limits"].IsSequence() ||
        node["limits"].size() != 2) {
      return {false, joint.name + ": limits must be [lower, upper]"};
    }
    try {
      joint.lower = node["limits"][0].as<double>();
      joint.upper = node["limits"][1].as<double>();
    } catch (const std::exception& exception) {
      return {false, joint.name + ": invalid limits: " + exception.what()};
    }
    if (!std::isfinite(joint.lower) || !std::isfinite(joint.upper) ||
        joint.lower > joint.upper) {
      return {false, joint.name + ": limits must be finite with lower <= upper"};
    }
    if (joint.velocity <= 0.0) {
      return {false, joint.name + ": velocity must be positive"};
    }
    if (std::abs(joint.theta_sign) < 1.0e-9) {
      return {false, joint.name + ": theta_sign must not be zero"};
    }
    joints.push_back(std::move(joint));
  }

  std::unordered_map<std::string, Eigen::Isometry3d> offsets;
  if (root["links"]) {
    if (!root["links"].IsMap()) {
      return {false, "links must be a map from link name to transform"};
    }
    for (const auto& entry : root["links"]) {
      const std::string link = entry.first.as<std::string>();
      Eigen::Isometry3d transform;
      if (!ReadTransform(entry.second, &transform, &error)) {
        return {false, "links." + link + ": " + error};
      }
      offsets[link] = transform;
    }
  }

  this->base = base;
  this->joints = std::move(joints);
  this->link_offsets = std::move(offsets);
  return {true, ""};
}

std::vector<std::string> DhParameters::JointNames() const {
  std::vector<std::string> names;
  names.reserve(joints.size());
  for (const DhJoint& joint : joints) {
    names.push_back(joint.name);
  }
  return names;
}

}  // namespace planning
}  // namespace tramcar
