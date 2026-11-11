#include "excavator_kinematics/model_converter/model_config_parser.hpp"

#include <cmath>
#include <exception>
#include <string>
#include <unordered_set>

namespace tramcar::planning {
namespace {

KinematicsResult ReadVector3(const YAML::Node& node,
                             Eigen::Vector3d* value,
                             const std::string& field) {
  if (!node || !node.IsSequence() || node.size() != 3 || value == nullptr) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        field + " must be a three-element sequence");
  }
  *value = Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(),
                           node[2].as<double>());
  if (!value->array().isFinite().all()) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     field + " must contain finite values");
  }
  return KinematicsResult::Success();
}

KinematicsResult ReadTransform(const YAML::Node& node, Transform* transform,
                               const std::string& field) {
  if (transform == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "transform output is null");
  }
  if (!node || !node.IsMap()) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     field + " must be a map");
  }
  KinematicsResult result =
      ReadVector3(node["xyz"], &transform->translation, field + ".xyz");
  if (!result.ok()) return result;

  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  if (node["rpy"]) {
    result = ReadVector3(node["rpy"], &rpy, field + ".rpy");
    if (!result.ok()) return result;
  }
  transform->rotation =
      Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
  transform->rotation.normalize();
  return KinematicsResult::Success();
}

JointType ParseJointType(const std::string& type, bool* valid) {
  *valid = true;
  if (type == "fixed") return JointType::kFixed;
  if (type == "revolute") return JointType::kRevolute;
  if (type == "continuous") return JointType::kContinuous;
  if (type == "prismatic") return JointType::kPrismatic;
  *valid = false;
  return JointType::kFixed;
}

}  // namespace

KinematicsResult ModelConfigParser::ParseFile(
    const std::string& path, ModelDescription* description) {
  if (path.empty() || description == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "YAML path and output are required");
  }
  try {
    return Parse(YAML::LoadFile(path), description);
  } catch (const std::exception& error) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "failed to load model YAML '" + path + "': " + error.what());
  }
}

KinematicsResult ModelConfigParser::Parse(
    const YAML::Node& input, ModelDescription* description) {
  if (description == nullptr || !input || !input.IsMap()) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "model YAML map and output are required");
  }
  const YAML::Node node =
      input["kinematics_model"] ? input["kinematics_model"] : input;
  if (!node.IsMap() || !node["name"] || !node["root_link"] ||
      !node["links"] || !node["links"].IsSequence() ||
      !node["joints"] || !node["joints"].IsSequence() ||
      !node["joint_order"] || !node["joint_order"].IsSequence()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "model YAML requires name, root_link, links, joints and joint_order");
  }

  ModelDescription converted;
  converted.name = node["name"].as<std::string>();
  converted.root_link = node["root_link"].as<std::string>();

  for (const YAML::Node& entry : node["links"]) {
    if (!entry["name"]) {
      return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                       "each Link requires a name");
    }
    converted.links.push_back({entry["name"].as<std::string>()});
  }

  for (const YAML::Node& entry : node["joints"]) {
    if (!entry["name"] || !entry["type"] || !entry["parent_link"] ||
        !entry["child_link"] || !entry["origin"] || !entry["axis"]) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "each Joint requires name, type, parent_link, child_link, origin "
          "and axis");
    }
    JointModel joint;
    joint.name = entry["name"].as<std::string>();
    bool valid_type = false;
    joint.type = ParseJointType(entry["type"].as<std::string>(), &valid_type);
    if (!valid_type) {
      return KinematicsResult::Failure(
          KinematicsStatus::kUnsupported,
          "unsupported Joint type: " + entry["type"].as<std::string>());
    }
    joint.parent_link = entry["parent_link"].as<std::string>();
    joint.child_link = entry["child_link"].as<std::string>();
    KinematicsResult result =
        ReadTransform(entry["origin"], &joint.origin,
                      "joints." + joint.name + ".origin");
    if (!result.ok()) return result;
    result = ReadVector3(entry["axis"], &joint.axis,
                         "joints." + joint.name + ".axis");
    if (!result.ok()) return result;

    const YAML::Node limit = entry["limit"];
    if (limit) {
      if (limit["lower"] || limit["upper"]) {
        if (!limit["lower"] || !limit["upper"]) {
          return KinematicsResult::Failure(
              KinematicsStatus::kInvalidArgument,
              "both lower and upper limits are required: " + joint.name);
        }
        joint.limit.has_position_limits = true;
        joint.limit.lower = limit["lower"].as<double>();
        joint.limit.upper = limit["upper"].as<double>();
      }
      if (limit["velocity"]) {
        joint.limit.has_velocity_limit = true;
        joint.limit.max_velocity = limit["velocity"].as<double>();
      }
    }
    converted.joints.push_back(std::move(joint));
  }

  std::unordered_set<std::string> ordered;
  for (const YAML::Node& entry : node["joint_order"]) {
    const std::string joint_name = entry.as<std::string>();
    if (!ordered.insert(joint_name).second) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "joint_order contains a duplicate: " + joint_name);
    }
    converted.joint_order.push_back(joint_name);
  }

  *description = std::move(converted);
  return KinematicsResult::Success();
}

}  // namespace tramcar::planning
