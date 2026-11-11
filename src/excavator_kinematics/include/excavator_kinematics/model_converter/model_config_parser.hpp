#ifndef EXCAVATOR_KINEMATICS__MODEL_CONFIG_PARSER_HPP_
#define EXCAVATOR_KINEMATICS__MODEL_CONFIG_PARSER_HPP_

#include <string>

#include <yaml-cpp/yaml.h>

#include "excavator_kinematics/kinematics_types.hpp"

namespace tramcar::planning {

class ModelConfigParser {
 public:
  static KinematicsResult ParseFile(const std::string& path,
                                    ModelDescription* description);
  static KinematicsResult Parse(const YAML::Node& node,
                                ModelDescription* description);
};

}  // namespace tramcar::planning

#endif  // EXCAVATOR_KINEMATICS__MODEL_CONFIG_PARSER_HPP_
