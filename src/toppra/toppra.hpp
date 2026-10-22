#ifndef TOPPRA_TOPPRA_HPP
#define TOPPRA_TOPPRA_HPP

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <toppra/export.hpp>
#include <toppra/logging.hpp>

// The TOPPRA_LOG* macros used to be compiled in or out depending on
// -DTOPPRA_DEBUG_ON / -DTOPPRA_WARN_ON, which meant an already-built
// library could not be made to log. They now route through the
// runtime-configurable logger in toppra/logging.hpp, so the same
// binary can log at any level. The macro names and call syntax are
// unchanged (see logging.hpp for the definitions); the compile-time
// flags now only pick the *default* level.

#define TOPPRA_UNUSED(x) (void)(x)

// Use for checking if a quantity is very close to zero
#define TOPPRA_NEARLY_ZERO 1e-8
#define TOPPRA_REL_TOL 1e-6
#define TOPPRA_ABS_TOL 1e-8

/// The TOPP-RA namespace
namespace toppra {
/// The scalar type
typedef double value_type;

constexpr value_type infty = std::numeric_limits<value_type>::infinity();

/// Column vector type
typedef Eigen::Matrix<value_type, Eigen::Dynamic, 1> Vector;
/// Matrix type
typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic> Matrix;

/// Vector of Vector
typedef std::vector<Vector> Vectors;
/// Vector of Matrix
typedef std::vector<Matrix> Matrices;

// internal data rep for Matrix
typedef std::tuple<Eigen::Index, Eigen::Index, std::vector<value_type>>
    MatrixData;
typedef std::vector<MatrixData> MatricesData;

/// 2D vector that stores the upper and lower bound of a variable.
typedef Eigen::Matrix<value_type, 1, 2> Bound;
/// Vector of Bound
typedef std::vector<Bound> Bounds;

class LinearConstraint;
/// Shared pointer to a LinearConstraint
typedef std::shared_ptr<LinearConstraint> LinearConstraintPtr;
/// Vector of LinearConstraintPtr
typedef std::vector<LinearConstraintPtr> LinearConstraintPtrs;
namespace constraint {
class LinearJointVelocity;
class LinearJointAcceleration;
class JointTorque;
} // namespace constraint

class Solver;
typedef std::shared_ptr<Solver> SolverPtr;
namespace solver {
class qpOASESWrapper;
} // namespace solver

class GeometricPath;
typedef std::shared_ptr<GeometricPath> GeometricPathPtr;

class PathParametrizationAlgorithm;
typedef std::shared_ptr<PathParametrizationAlgorithm>
    PathParametrizationAlgorithmPtr;

} // namespace toppra

#endif
