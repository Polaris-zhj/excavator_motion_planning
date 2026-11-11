#include "excavator_kinematics/backend/kdl/kdl_type_converter.hpp"

namespace tramcar::planning {

KDL::Frame KdlTypeConverter::ToKdlFrame(const Pose& pose) {
  const Eigen::Quaterniond orientation = pose.orientation.normalized();
  return KDL::Frame(
      KDL::Rotation::Quaternion(orientation.x(), orientation.y(),
                                orientation.z(), orientation.w()),
      KDL::Vector(pose.position.x(), pose.position.y(), pose.position.z()));
}

Pose KdlTypeConverter::FromKdlFrame(const KDL::Frame& frame) {
  Pose pose;
  pose.position = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  frame.M.GetQuaternion(x, y, z, w);
  pose.orientation = Eigen::Quaterniond(w, x, y, z).normalized();
  return pose;
}

bool KdlTypeConverter::ToKdlJntArray(
    const JointVector& q, const std::vector<std::size_t>& joint_indices,
    KDL::JntArray* output) {
  if (output == nullptr) return false;
  output->resize(joint_indices.size());
  for (std::size_t i = 0; i < joint_indices.size(); ++i) {
    if (joint_indices[i] >= static_cast<std::size_t>(q.size())) return false;
    (*output)(static_cast<unsigned int>(i)) =
        q(static_cast<Eigen::Index>(joint_indices[i]));
  }
  return true;
}

bool KdlTypeConverter::FromKdlJntArray(
    const KDL::JntArray& input, const std::vector<std::size_t>& joint_indices,
    JointVector* output) {
  if (output == nullptr || input.rows() != joint_indices.size()) return false;
  for (std::size_t i = 0; i < joint_indices.size(); ++i) {
    if (joint_indices[i] >= static_cast<std::size_t>(output->size())) {
      return false;
    }
    (*output)(static_cast<Eigen::Index>(joint_indices[i])) =
        input(static_cast<unsigned int>(i));
  }
  return true;
}

JacobianMatrix KdlTypeConverter::FromKdlJacobian(
    const KDL::Jacobian& jacobian) {
  return jacobian.data;
}

}  // namespace tramcar::planning
