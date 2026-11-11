#ifndef EXCAVATOR_KINEMATICS__KDL_MODEL_CONVERTER_HPP_
#define EXCAVATOR_KINEMATICS__KDL_MODEL_CONVERTER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "excavator_kinematics/kinematics_model.hpp"
#include "chain.hpp"

namespace tramcar::planning {

struct KdlChainDescription {
  KDL::Chain chain;
  std::vector<std::size_t> joint_indices;
  std::vector<JointLimit> joint_limits;
};

class KdlModelConverter {
 public:
  KinematicsResult BuildChain(const KinematicsModel& model,
                              const std::string& base_link,
                              const std::string& tip_link,
                              KdlChainDescription* description) const;
};

}  // namespace tramcar::planning

#endif  // EXCAVATOR_KINEMATICS__KDL_MODEL_CONVERTER_HPP_
