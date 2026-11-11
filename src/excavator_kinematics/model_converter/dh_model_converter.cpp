#include "excavator_kinematics/model_converter/dh_model_converter.hpp"

#include <cmath>
#include <exception>
#include <string>
#include <utility>

#include <Eigen/Geometry>

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

KinematicsResult ReadRpy(const YAML::Node& node, Eigen::Quaterniond* rotation,
                         const std::string& field) {
  if (!node) {
    *rotation = Eigen::Quaterniond::Identity();
    return KinematicsResult::Success();
  }
  Eigen::Vector3d rpy;
  KinematicsResult result = ReadVector3(node, &rpy, field);
  if (!result.ok()) return result;
  *rotation = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
  rotation->normalize();
  return KinematicsResult::Success();
}

JointModel MakeFixedJoint(const std::string& name, const std::string& parent,
                          const std::string& child,
                          const Transform& origin) {
  JointModel joint;
  joint.name = name;
  joint.type = JointType::kFixed;
  joint.parent_link = parent;
  joint.child_link = child;
  joint.origin = origin;
  return joint;
}

}  // namespace

KinematicsResult DhModelConverter::FromFile(
    const std::string& path, ModelDescription* description) {
  if (path.empty() || description == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "DH path and output are required");
  }
  try {
    return FromYaml(YAML::LoadFile(path), description);
  } catch (const std::exception& error) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "failed to load DH YAML '" + path + "': " + error.what());
  }
}

KinematicsResult DhModelConverter::FromYaml(
    const YAML::Node& node, ModelDescription* description) {
  if (description == nullptr || !node || !node.IsMap()) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "DH YAML and output are required");
  }
  const std::string convention =
      node["convention"] ? node["convention"].as<std::string>() : "";
  if (convention != "modified_dh") {
    return KinematicsResult::Failure(
        KinematicsStatus::kUnsupported,
        "only convention: modified_dh is supported");
  }
  if (!node["joints"] || !node["joints"].IsSequence() ||
      node["joints"].size() == 0) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "Modified DH requires a non-empty joints sequence");
  }

  ModelDescription converted;
  converted.name = node["name"] ? node["name"].as<std::string>()
                                 : "modified_dh_model";
  converted.root_link =
      node["root_link"] ? node["root_link"].as<std::string>() : "base_link";
  converted.links.push_back({converted.root_link});

  Transform base;
  if (node["base"]) {
    KinematicsResult result =
        ReadVector3(node["base"]["xyz"], &base.translation, "base.xyz");
    if (!result.ok()) return result;
    result = ReadRpy(node["base"]["rpy"], &base.rotation, "base.rpy");
    if (!result.ok()) return result;
  }

  const std::string base_frame = "__dh_base_frame";
  converted.links.push_back({base_frame});
  converted.joints.push_back(
      MakeFixedJoint("__dh_base_mount", converted.root_link, base_frame, base));
  std::string previous_frame = base_frame;

  std::size_t index = 0;
  for (const YAML::Node& entry : node["joints"]) {
    if (!entry["name"] || !entry["link"] || !entry["alpha_prev"] ||
        !entry["a_prev"] || !entry["d"]) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "each Modified DH row requires name, link, alpha_prev, a_prev and d");
    }
    const std::string name = entry["name"].as<std::string>();
    const std::string rotation_frame =
        "__dh_joint_rotation_" + std::to_string(index);
    const std::string d_frame = "__dh_d_offset_" + std::to_string(index);
    const std::string physical_link = entry["link"].as<std::string>();
    const double alpha = entry["alpha_prev"].as<double>();
    const double a = entry["a_prev"].as<double>();
    const double d = entry["d"].as<double>();
    const double theta_offset =
        entry["theta_offset"] ? entry["theta_offset"].as<double>() : 0.0;
    const double theta_sign =
        entry["theta_sign"] ? entry["theta_sign"].as<double>() : 1.0;
    if (!std::isfinite(alpha) || !std::isfinite(a) || !std::isfinite(d) ||
        !std::isfinite(theta_offset) || !std::isfinite(theta_sign) ||
        std::abs(theta_sign) < 1.0e-12) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "Modified DH parameters must be finite and theta_sign non-zero");
    }

    JointModel active;
    active.name = name;
    active.type = JointType::kRevolute;
    active.parent_link = previous_frame;
    active.child_link = rotation_frame;
    active.origin.translation = Eigen::Vector3d(a, 0.0, 0.0);
    active.origin.rotation =
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(theta_offset, Eigen::Vector3d::UnitZ());
    active.axis = Eigen::Vector3d(0.0, 0.0, theta_sign);

    if (entry["limits"]) {
      const YAML::Node limits = entry["limits"];
      if (!limits.IsSequence() || limits.size() != 2) {
        return KinematicsResult::Failure(
            KinematicsStatus::kInvalidArgument,
            "Modified DH limits must be [lower, upper]: " + name);
      }
      active.limit.has_position_limits = true;
      active.limit.lower = limits[0].as<double>();
      active.limit.upper = limits[1].as<double>();
    }
    if (entry["velocity"]) {
      active.limit.has_velocity_limit = true;
      active.limit.max_velocity = entry["velocity"].as<double>();
    }
    converted.links.push_back({rotation_frame});
    converted.joints.push_back(active);
    converted.joint_order.push_back(name);

    Transform d_offset;
    d_offset.translation = Eigen::Vector3d(0.0, 0.0, d);
    converted.links.push_back({d_frame});
    converted.joints.push_back(MakeFixedJoint(
        "__dh_post_joint_offset_" + std::to_string(index), rotation_frame,
        d_frame, d_offset));

    Transform link_offset;
    const YAML::Node link_node =
        node["links"] ? node["links"][physical_link] : YAML::Node();
    if (link_node) {
      KinematicsResult result = ReadVector3(
          link_node["xyz"], &link_offset.translation,
          "links." + physical_link + ".xyz");
      if (!result.ok()) return result;
      result = ReadRpy(link_node["rpy"], &link_offset.rotation,
                       "links." + physical_link + ".rpy");
      if (!result.ok()) return result;
    }
    converted.links.push_back({physical_link});
    converted.joints.push_back(MakeFixedJoint(
        "__dh_to_" + physical_link, d_frame, physical_link, link_offset));

    previous_frame = physical_link;
    ++index;
  }

  if (node["links"] && node["links"]["bucket_tip"]) {
    Transform tip_offset;
    const YAML::Node tip = node["links"]["bucket_tip"];
    KinematicsResult result = ReadVector3(
        tip["xyz"], &tip_offset.translation, "links.bucket_tip.xyz");
    if (!result.ok()) return result;
    result = ReadRpy(tip["rpy"], &tip_offset.rotation,
                     "links.bucket_tip.rpy");
    if (!result.ok()) return result;
    converted.links.push_back({"bucket_tip"});
    converted.joints.push_back(MakeFixedJoint(
        "__dh_to_bucket_tip", previous_frame, "bucket_tip", tip_offset));
  }

  *description = std::move(converted);
  return KinematicsResult::Success();
}

}  // namespace tramcar::planning
