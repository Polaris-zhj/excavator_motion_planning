#ifndef TOPPRA_ALGORITHM_TOPPRA_HPP
#define TOPPRA_ALGORITHM_TOPPRA_HPP

#include "toppra/src/algorithm.hpp"
#include "toppra/src/constraint.hpp"
#include "toppra/src/geometric_path.hpp"
#include "toppra/src/toppra.hpp"

namespace toppra {
namespace algorithm {
class TOPPRA : public PathParametrizationAlgorithm {
 public:
  TOPPRA(LinearConstraintPtrs constraints, const GeometricPathPtr& path);

 protected:
  ReturnCode computeForwardPass(value_type vel_start);
};
}  // namespace algorithm
}  // namespace toppra

#endif
