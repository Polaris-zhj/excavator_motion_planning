#include "excavator_kinematics/kinematics_model.hpp"

#include <cmath>
#include <functional>
#include <unordered_set>
#include <utility>

namespace tramcar::planning {
namespace {

bool IsFinite(const Transform& transform) {
  return transform.translation.array().isFinite().all() &&
         transform.rotation.coeffs().array().isFinite().all() &&
         transform.rotation.norm() > 1.0e-12;
}

}  // namespace

KinematicsResult KinematicsModel::Initialize(
    const ModelDescription& description) {
  initialized_ = false;
  name_.clear();
  root_link_.clear();
  links_.clear();
  joints_.clear();
  joint_order_.clear();
  link_indices_.clear();
  joint_indices_.clear();
  parent_joint_indices_.clear();
  active_joint_indices_.clear();

  if (description.name.empty() || description.root_link.empty() ||
      description.links.empty()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "model name, root_link and at least one Link are required");
  }

  for (std::size_t i = 0; i < description.links.size(); ++i) {
    const LinkModel& link = description.links[i];
    if (link.name.empty() ||
        !link_indices_.emplace(link.name, i).second) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "Link names must be non-empty and unique");
    }
  }
  if (link_indices_.count(description.root_link) == 0U) {
    return KinematicsResult::Failure(KinematicsStatus::kUnknownLink,
                                     "root_link does not exist");
  }

  std::unordered_set<std::string> child_links;
  for (std::size_t i = 0; i < description.joints.size(); ++i) {
    const JointModel& joint = description.joints[i];
    if (joint.name.empty() || joint.parent_link.empty() ||
        joint.child_link.empty() ||
        !joint_indices_.emplace(joint.name, i).second) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "Joint names and parent/child Link names must be non-empty; "
          "Joint names must be unique");
    }
    if (link_indices_.count(joint.parent_link) == 0U ||
        link_indices_.count(joint.child_link) == 0U) {
      return KinematicsResult::Failure(
          KinematicsStatus::kUnknownLink,
          "Joint references a Link that is not in the model: " + joint.name);
    }
    if (joint.parent_link == joint.child_link ||
        !child_links.insert(joint.child_link).second) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "each non-root Link must have exactly one parent Joint");
    }
    if (!IsFinite(joint.origin)) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "Joint origin must contain a finite translation and valid rotation: " +
              joint.name);
    }
    if (joint.type != JointType::kFixed) {
      if (!joint.axis.array().isFinite().all() ||
          joint.axis.norm() < 1.0e-12) {
        return KinematicsResult::Failure(
            KinematicsStatus::kInvalidArgument,
            "movable Joint axis must be finite and non-zero: " + joint.name);
      }
      if (joint.limit.has_position_limits &&
          (!std::isfinite(joint.limit.lower) ||
           !std::isfinite(joint.limit.upper) ||
           joint.limit.lower > joint.limit.upper)) {
        return KinematicsResult::Failure(
            KinematicsStatus::kInvalidArgument,
            "Joint position limits are invalid: " + joint.name);
      }
      if (joint.limit.has_velocity_limit &&
          (!std::isfinite(joint.limit.max_velocity) ||
           joint.limit.max_velocity <= 0.0)) {
        return KinematicsResult::Failure(
            KinematicsStatus::kInvalidArgument,
            "Joint velocity limit must be positive and finite: " +
                joint.name);
      }
    }
    parent_joint_indices_.emplace(joint.child_link, i);
  }

  if (child_links.count(description.root_link) != 0U) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "root_link cannot have a parent Joint");
  }
  if (description.joint_order.size() !=
      static_cast<std::size_t>(std::count_if(
          description.joints.begin(), description.joints.end(),
          [](const JointModel& joint) {
            return joint.type != JointType::kFixed;
          }))) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "joint_order must contain every movable Joint exactly once");
  }

  for (std::size_t i = 0; i < description.joint_order.size(); ++i) {
    const std::string& joint_name = description.joint_order[i];
    const auto found = joint_indices_.find(joint_name);
    if (found == joint_indices_.end() ||
        description.joints[found->second].type == JointType::kFixed ||
        !active_joint_indices_.emplace(joint_name, i).second) {
      return KinematicsResult::Failure(
          KinematicsStatus::kInvalidArgument,
          "joint_order contains an unknown, fixed or duplicate Joint: " +
              joint_name);
    }
  }

  std::unordered_set<std::string> visited;
  std::function<bool(const std::string&)> visit =
      [&](const std::string& link_name) {
        if (!visited.insert(link_name).second) return false;
        for (const JointModel& joint : description.joints) {
          if (joint.parent_link == link_name &&
              !visit(joint.child_link)) {
            return false;
          }
        }
        return true;
      };
  if (!visit(description.root_link) ||
      visited.size() != description.links.size()) {
    return KinematicsResult::Failure(
        KinematicsStatus::kInvalidArgument,
        "model topology must be a connected acyclic tree rooted at root_link");
  }

  name_ = description.name;
  root_link_ = description.root_link;
  links_ = description.links;
  joints_ = description.joints;
  joint_order_ = description.joint_order;
  initialized_ = true;
  return KinematicsResult::Success();
}

const LinkModel* KinematicsModel::GetLink(const std::string& name) const {
  const auto found = link_indices_.find(name);
  return found == link_indices_.end() ? nullptr : &links_[found->second];
}

const JointModel* KinematicsModel::GetJoint(const std::string& name) const {
  const auto found = joint_indices_.find(name);
  return found == joint_indices_.end() ? nullptr : &joints_[found->second];
}

const JointModel* KinematicsModel::GetParentJoint(
    const std::string& child_link) const {
  const auto found = parent_joint_indices_.find(child_link);
  return found == parent_joint_indices_.end() ? nullptr
                                               : &joints_[found->second];
}

bool KinematicsModel::GetActiveJointIndex(const std::string& joint_name,
                                          std::size_t* index) const {
  if (index == nullptr) return false;
  const auto found = active_joint_indices_.find(joint_name);
  if (found == active_joint_indices_.end()) return false;
  *index = found->second;
  return true;
}

}  // namespace tramcar::planning
