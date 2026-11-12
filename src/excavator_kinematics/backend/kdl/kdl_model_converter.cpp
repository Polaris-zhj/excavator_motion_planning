#include "excavator_kinematics/backend/kdl/kdl_model_converter.hpp"

#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "joint.hpp"
#include "segment.hpp"

namespace excavator {
namespace planning {
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

KinematicsResult KdlModelConverter::BuildFullChain(
    const KinematicsModel& model,
    KdlChainDescription* description) const {
  if (!model.initialized() || description == nullptr) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "initialized model and Chain output are required");
  }

  std::unordered_map<std::string, std::vector<const JointModel*>> children;
  for (const JointModel& joint : model.GetJoints()) {
    children[joint.parent_link].push_back(&joint);
  }

  std::unordered_set<std::string> active_names(
      model.GetActiveJointNames().begin(),
      model.GetActiveJointNames().end());
  std::vector<const JointModel*> candidate;
  std::vector<const JointModel*> selected;
  std::size_t best_size = 0;
  bool ambiguous = false;

  std::function<void(const std::string&)> visit =
      [&](const std::string& link_name) {
        const auto found = children.find(link_name);
        if (found == children.end() || found->second.empty()) {
          std::unordered_set<std::string> path_active;
          for (const JointModel* joint : candidate) {
            if (active_names.count(joint->name) != 0U) {
              path_active.insert(joint->name);
            }
          }
          if (path_active.size() == active_names.size()) {
            if (candidate.size() > best_size) {
              best_size = candidate.size();
              selected = candidate;
              ambiguous = false;
            } else if (candidate.size() == best_size &&
                       candidate != selected) {
              ambiguous = true;
            }
          }
          return;
        }
        for (const JointModel* joint : found->second) {
          candidate.push_back(joint);
          visit(joint->child_link);
          candidate.pop_back();
        }
      };
  visit(model.root_link());

  if (selected.empty() && !active_names.empty()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kUnsupported,
        "active joints do not belong to one root-to-leaf serial chain");
  }
  if (ambiguous) {
    return KinematicsResult::Failure(
        KinematicsStatus::kUnsupported,
        "multiple equally long main chains contain all active joints; "
        "the single-chain backend requires an unambiguous main chain");
  }

  KdlChainDescription converted;
  converted.link_segment_indices.emplace(model.root_link(), 0U);
  for (const JointModel* joint : selected) {
    std::size_t active_index = 0;
    const bool is_active =
        joint->type != JointType::kFixed &&
        model.GetActiveJointIndex(joint->name, &active_index);
    const KDL::Joint kdl_joint =
        is_active ? ToKdlJoint(*joint)
                  : KDL::Joint(joint->name, KDL::Joint::Fixed);
    // KDL::Segment factors out Joint::pose(0), so pass the complete
    // zero-position Parent-Link-to-Child-Link transform.
    converted.chain.addSegment(
        KDL::Segment(joint->child_link, kdl_joint, ToKdlFrame(joint->origin)));
    converted.link_segment_indices.emplace(
        joint->child_link,
        static_cast<unsigned int>(converted.chain.getNrOfSegments()));
    if (is_active) {
      converted.joint_indices.push_back(active_index);
      converted.joint_limits.push_back(joint->limit);
    }
  }

  if (converted.chain.getNrOfJoints() != converted.joint_indices.size() ||
      converted.joint_indices.size() != converted.joint_limits.size()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kBackendFailure,
        "KDL Chain joint count does not match joint index/limit mappings");
  }
  *description = std::move(converted);
  return KinematicsResult::Success();
}

}  // namespace planning
}  // namespace excavator
