#include "excavator_kinematics/backend/kdl/kdl_model_converter.hpp"

#include <algorithm>
#include <unordered_set>
#include <utility>

#include "joint.hpp"
#include "segment.hpp"

namespace tramcar::planning {
namespace {

KDL::Frame ToKdlFrame(const Transform& transform) {
  const Eigen::Quaterniond rotation = transform.rotation.normalized();
  return KDL::Frame(
      KDL::Rotation::Quaternion(rotation.x(), rotation.y(), rotation.z(),
                                rotation.w()),
      KDL::Vector(transform.translation.x(), transform.translation.y(),
                  transform.translation.z()));
}

KDL::Joint ToKdlJoint(const JointModel& joint) {
  if (joint.type == JointType::kFixed) {
    return KDL::Joint(joint.name, KDL::Joint::Fixed);
  }
  const Eigen::Quaterniond origin_rotation = joint.origin.rotation.normalized();
  const Eigen::Vector3d axis_in_parent =
      origin_rotation * joint.axis.normalized();
  const KDL::Vector origin(joint.origin.translation.x(),
                           joint.origin.translation.y(),
                           joint.origin.translation.z());
  const KDL::Vector axis(axis_in_parent.x(), axis_in_parent.y(),
                         axis_in_parent.z());
  const KDL::Joint::JointType type =
      joint.type == JointType::kPrismatic ? KDL::Joint::TransAxis
                                          : KDL::Joint::RotAxis;
  return KDL::Joint(joint.name, origin, axis, type);
}

}  // namespace

KinematicsResult KdlModelConverter::BuildChain(
    const KinematicsModel& model, const std::string& base_link,
    const std::string& tip_link,
    KdlChainDescription* description) const {
  if (!model.initialized() || description == nullptr) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "initialized model and Chain output are required");
  }
  if (model.GetLink(base_link) == nullptr ||
      model.GetLink(tip_link) == nullptr) {
    return KinematicsResult::Failure(
        KinematicsStatus::kUnknownLink,
        "base_link or tip_link does not exist in the model");
  }
  if (base_link == tip_link) {
    *description = KdlChainDescription{};
    return KinematicsResult::Success();
  }

  std::vector<const JointModel*> reversed_path;
  std::string current_link = tip_link;
  std::unordered_set<std::string> visited;
  while (current_link != base_link) {
    if (!visited.insert(current_link).second) {
      return KinematicsResult::Failure(KinematicsStatus::kInvalidArgument,
                                       "cycle detected while extracting Chain");
    }
    const JointModel* joint = model.GetParentJoint(current_link);
    if (joint == nullptr) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "base_link is not an ancestor of tip_link");
    }
    reversed_path.push_back(joint);
    current_link = joint->parent_link;
  }
  std::reverse(reversed_path.begin(), reversed_path.end());

  KdlChainDescription converted;
  for (const JointModel* joint : reversed_path) {
    const KDL::Joint kdl_joint = ToKdlJoint(*joint);
    KDL::Frame frame_to_tip = ToKdlFrame(joint->origin);
    if (joint->type != JointType::kFixed) {
      // KDL::Joint::pose(q) already applies the joint-origin translation.
      // Only the fixed origin rotation belongs in Segment::f_tip here.
      frame_to_tip.p = KDL::Vector::Zero();
    }
    converted.chain.addSegment(
        KDL::Segment(joint->child_link, kdl_joint, frame_to_tip));
    if (joint->type != JointType::kFixed) {
      std::size_t index = 0;
      if (!model.GetActiveJointIndex(joint->name, &index)) {
        return KinematicsResult::Failure(
            KinematicsStatus::kInvalidArgument,
            "active Joint is missing from joint_order: " + joint->name);
      }
      converted.joint_indices.push_back(index);
      converted.joint_limits.push_back(joint->limit);
    }
  }
  if (converted.chain.getNrOfJoints() != converted.joint_indices.size()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "KDL Chain joint count does not match the project joint mapping");
  }
  *description = std::move(converted);
  return KinematicsResult::Success();
}

}  // namespace tramcar::planning
