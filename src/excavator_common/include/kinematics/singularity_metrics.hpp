#ifndef KINEMATICS__SINGULARITY_METRICS_HPP_
#define KINEMATICS__SINGULARITY_METRICS_HPP_

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "kinematics/kinematics_interface.hpp"

namespace tramcar {
namespace planning {

struct SingularityMetrics {
  bool valid{false};
  double minimum_singular_value{0.0};
  double condition_number{std::numeric_limits<double>::infinity()};
  double manipulability{0.0};
};

inline SingularityMetrics EvaluateSingularity(const Jacobian& jacobian,
                                              double epsilon = 1.0e-9) {
  SingularityMetrics result;
  if (jacobian.rows.empty() || !std::isfinite(epsilon) || epsilon <= 0.0) {
    return result;
  }
  const std::size_t column_count = jacobian.rows.front().size();
  if (column_count == 0) {
    return result;
  }

  Eigen::MatrixXd matrix(static_cast<Eigen::Index>(jacobian.rows.size()),
                        static_cast<Eigen::Index>(column_count));
  for (std::size_t row = 0; row < jacobian.rows.size(); ++row) {
    if (jacobian.rows[row].size() != column_count) {
      return result;
    }
    for (std::size_t column = 0; column < column_count; ++column) {
      const double value = jacobian.rows[row][column];
      if (!std::isfinite(value)) {
        return result;
      }
      matrix(static_cast<Eigen::Index>(row),
             static_cast<Eigen::Index>(column)) = value;
    }
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() == 0 || !singular_values.allFinite()) {
    return result;
  }
  result.minimum_singular_value = singular_values.minCoeff();
  result.manipulability = singular_values.prod();
  if (result.minimum_singular_value > epsilon) {
    result.condition_number =
        singular_values.maxCoeff() / result.minimum_singular_value;
  }
  result.valid = std::isfinite(result.minimum_singular_value) &&
                 std::isfinite(result.manipulability);
  return result;
}

}  // namespace planning
}  // namespace tramcar

#endif  // KINEMATICS__SINGULARITY_METRICS_HPP_
