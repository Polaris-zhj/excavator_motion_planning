#include "excavator_kinematics/model_converter/urdf_model_converter.hpp"

#include <functional>
#include <unordered_set>
#include <utility>

#include <urdf/model.h>

namespace tramcar::planning {
namespace {

Transform ToTransform(const urdf::Pose& pose) {
  Transform result;
  result.translation =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.rotation =
      Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                         pose.rotation.z)
          .normalized();
  return result;
}

Eigen::Vector3d ToVector(const urdf::Vector3& vector) {
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

KinematicsResult ConvertModel(const urdf::Model& urdf_model,
                              ModelDescription* description) {
  if (description == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "output description is null");
  }
  const urdf::LinkConstSharedPtr root = urdf_model.getRoot();
  if (!root) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "URDF does not contain a root Link");
  }

  ModelDescription converted;
  converted.name = "urdf_model";
  converted.root_link = root->name;
  std::unordered_set<std::string> visited_links;
  std::unordered_set<std::string> visited_joints;

  std::function<KinematicsResult(const urdf::LinkConstSharedPtr&)> visit;
  visit = [&](const urdf::LinkConstSharedPtr& link) -> KinematicsResult {
    if (!link || !visited_links.insert(link->name).second) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "URDF contains a cycle or duplicate Link traversal");
    }
    converted.links.push_back({link->name});

    for (const urdf::JointConstSharedPtr& urdf_joint : link->child_joints) {
      if (!urdf_joint ||
          !visited_joints.insert(urdf_joint->name).second) {
        return KinematicsResult::Failure(
            KinematicsStatus::kInvalidArgument,
            "URDF contains an invalid or duplicate Joint");
      }
      JointModel joint;
      joint.name = urdf_joint->name;
      joint.parent_link = urdf_joint->parent_link_name;
      joint.child_link = urdf_joint->child_link_name;
      joint.origin = ToTransform(urdf_joint->parent_to_joint_origin_transform);
      joint.axis = ToVector(urdf_joint->axis);

      switch (urdf_joint->type) {
        case urdf::Joint::FIXED:
          joint.type = JointType::kFixed;
          break;
        case urdf::Joint::REVOLUTE:
          joint.type = JointType::kRevolute;
          break;
        case urdf::Joint::CONTINUOUS:
          joint.type = JointType::kContinuous;
          break;
        case urdf::Joint::PRISMATIC:
          joint.type = JointType::kPrismatic;
          break;
        default:
          return KinematicsResult::Failure(
              KinematicsStatus::kUnsupported,
              "URDF Joint type is not supported: " + urdf_joint->name);
      }

      if (urdf_joint->limits) {
        if (joint.type != JointType::kContinuous &&
            joint.type != JointType::kFixed) {
          joint.limit.has_position_limits = true;
          joint.limit.lower = urdf_joint->limits->lower;
          joint.limit.upper = urdf_joint->limits->upper;
        }
        if (urdf_joint->limits->velocity > 0.0) {
          joint.limit.has_velocity_limit = true;
          joint.limit.max_velocity = urdf_joint->limits->velocity;
        }
      }
      converted.joints.push_back(joint);
      if (joint.type != JointType::kFixed) {
        converted.joint_order.push_back(joint.name);
      }

      const urdf::LinkConstSharedPtr child =
          urdf_model.getLink(urdf_joint->child_link_name);
      KinematicsResult result = visit(child);
      if (!result.ok()) return result;
    }
    return KinematicsResult::Success();
  };

  KinematicsResult result = visit(root);
  if (!result.ok()) return result;
  *description = std::move(converted);
  return KinematicsResult::Success();
}

}  // namespace

KinematicsResult UrdfModelConverter::FromFile(
    const std::string& path, ModelDescription* description) {
  if (path.empty() || description == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "URDF path and output are required");
  }
  urdf::Model model;
  if (!model.initFile(path)) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "failed to parse URDF file: " + path);
  }
  return ConvertModel(model, description);
}

KinematicsResult UrdfModelConverter::FromString(
    const std::string& xml, ModelDescription* description) {
  if (xml.empty() || description == nullptr) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "URDF XML and output are required");
  }
  urdf::Model model;
  if (!model.initString(xml)) {
    return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                     "failed to parse URDF XML string");
  }
  return ConvertModel(model, description);
}

}  // namespace tramcar::planning
