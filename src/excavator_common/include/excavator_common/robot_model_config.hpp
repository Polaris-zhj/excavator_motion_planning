#ifndef EXCAVATOR_COMMON__ROBOT_MODEL_CONFIG_HPP_
#define EXCAVATOR_COMMON__ROBOT_MODEL_CONFIG_HPP_

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include "excavator_common/joint_limits.hpp"

namespace tramcar {
namespace planning {

struct RobotModelConfig {
  std::string model_description;
  std::string base_frame{"base_link"};
  std::string end_effector_link{"bucket_tip"};
  std::vector<std::string> joint_names;
  std::vector<JointLimit> joint_limits;

  ValidationResult Validate() const {
    if (base_frame.empty() || end_effector_link.empty()) {
      return {false, "base frame and end-effector link must be specified"};
    }
    if (joint_names.empty() || joint_names.size() != joint_limits.size()) {
      return {false, "joint names and limits must be non-empty and aligned"};
    }
    std::unordered_set<std::string> unique_names;
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      if (joint_names[index].empty() ||
          !unique_names.insert(joint_names[index]).second) {
        return {false, "joint names must be non-empty and unique"};
      }
      const JointLimit& limit = joint_limits[index];
      if (!limit.continuous && limit.lower > limit.upper) {
        return {false, "joint lower limit exceeds upper limit"};
      }
      if (limit.max_velocity <= 0.0 || limit.max_acceleration <= 0.0) {
        return {false, "joint velocity and acceleration limits must be positive"};
      }
    }
    return {true, ""};
  }
};

}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_COMMON__ROBOT_MODEL_CONFIG_HPP_
