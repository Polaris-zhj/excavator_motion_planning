#ifndef EXCAVATOR_KINEMATICS__KINEMATICS_MODEL_HPP_
#define EXCAVATOR_KINEMATICS__KINEMATICS_MODEL_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "excavator_kinematics/kinematics_types.hpp"

namespace excavator {
namespace planning {

class KinematicsModel {
 public:
  KinematicsResult Initialize(const ModelDescription& description);

  bool initialized() const { return initialized_; }
  const std::string& name() const { return name_; }
  const std::string& root_link() const { return root_link_; }
  const std::vector<std::string>& GetActiveJointNames() const {
    return joint_order_;
  }
  const LinkModel* GetLink(const std::string& name) const;
  const JointModel* GetJoint(const std::string& name) const;
  const JointModel* GetParentJoint(const std::string& child_link) const;
  bool GetActiveJointIndex(const std::string& joint_name,
                           std::size_t* index) const;

 private:
  bool initialized_{false};
  std::string name_;
  std::string root_link_;
  std::vector<LinkModel> links_;
  std::vector<JointModel> joints_;
  std::vector<std::string> joint_order_;
  std::unordered_map<std::string, std::size_t> link_indices_;
  std::unordered_map<std::string, std::size_t> joint_indices_;
  std::unordered_map<std::string, std::size_t> parent_joint_indices_;
  std::unordered_map<std::string, std::size_t> active_joint_indices_;
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_KINEMATICS__KINEMATICS_MODEL_HPP_
