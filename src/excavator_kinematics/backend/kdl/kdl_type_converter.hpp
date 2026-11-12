#ifndef EXCAVATOR_KINEMATICS__KDL_TYPE_CONVERTER_HPP_
#define EXCAVATOR_KINEMATICS__KDL_TYPE_CONVERTER_HPP_

#include <cstddef>
#include <vector>

#include "excavator_kinematics/kinematics_types.hpp"
#include "frames.hpp"
#include "jacobian.hpp"
#include "jntarray.hpp"

namespace excavator {
namespace planning {

class KdlTypeConverter {
 public:
  static KDL::Frame ToKdlFrame(const Pose& pose);
  static Pose FromKdlFrame(const KDL::Frame& frame);
  static bool ToKdlJntArray(const JointVector& q,
                            const std::vector<std::size_t>& joint_indices,
                            KDL::JntArray* output);
  static bool FromKdlJntArray(const KDL::JntArray& input,
                              const std::vector<std::size_t>& joint_indices,
                              JointVector* output);
  static JacobianMatrix FromKdlJacobian(const KDL::Jacobian& jacobian);
};

}  // namespace planning
}  // namespace excavator

#endif  // EXCAVATOR_KINEMATICS__KDL_TYPE_CONVERTER_HPP_
