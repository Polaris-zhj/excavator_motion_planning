#ifndef EXCAVATOR_KINEMATICS__KDL_MODEL_CONVERTER_HPP_
#define EXCAVATOR_KINEMATICS__KDL_MODEL_CONVERTER_HPP_

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "excavator_kinematics/kinematics_model.hpp"
#include "chain.hpp"

namespace excavator {
namespace planning {

struct KdlChainDescription {
  KDL::Chain chain;
  std::vector<std::size_t> joint_indices;
  std::vector<JointLimit> joint_limits;
  std::unordered_map<std::string, unsigned int> link_segment_indices;
};

class KdlModelConverter {
 public:
  // Builds the longest root-to-leaf path containing every active joint.
  // A branched active-joint topology cannot be represented by one KDL Chain.
  KinematicsResult BuildFullChain(
      const KinematicsModel& model,
      KdlChainDescription* description) const;
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_KINEMATICS__KDL_MODEL_CONVERTER_HPP_
