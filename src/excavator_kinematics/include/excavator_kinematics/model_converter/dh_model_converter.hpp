#ifndef EXCAVATOR_KINEMATICS__DH_MODEL_CONVERTER_HPP_
#define EXCAVATOR_KINEMATICS__DH_MODEL_CONVERTER_HPP_

#include <string>

#include <yaml-cpp/yaml.h>

#include "excavator_kinematics/kinematics_types.hpp"

namespace tramcar::planning {

class DhModelConverter {
 public:
  static KinematicsResult FromFile(const std::string& path,
                                   ModelDescription* description);
  static KinematicsResult FromYaml(const YAML::Node& node,
                                   ModelDescription* description);
};

}  // namespace tramcar::planning

#endif  // EXCAVATOR_KINEMATICS__DH_MODEL_CONVERTER_HPP_
