#include "excavator_planning/toppra_time_parameterizer.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/solver/seidel.hpp>
#include <utility>
#include <vector>

#include "toppra/src/toppra.hpp"

namespace excavator {
namespace planning {
namespace {

// Consecutive waypoints closer than this are merged so the spline parameter
// stays strictly increasing.
constexpr double kDuplicatePointTolerance = 1.0e-9;
// Uniform time margin. Besides absorbing the TOPP-RA grid discretization
// residual, it covers the nanosecond quantization applied when the trajectory
// is converted to a ROS message, which can move the controller's finite
// difference estimate by ~1e-4 relative.
constexpr double kTimeScaleMargin = 1.0e-3;
constexpr std::size_t kMinGridPoints = 2;
constexpr std::size_t kMaxGridPoints = 100000;

bool IsFiniteVector(const JointPositions& values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

double MaxAbsDifference(const JointPositions& lhs, const JointPositions& rhs) {
  double difference = 0.0;
  for (std::size_t joint = 0; joint < lhs.size(); ++joint) {
    difference = std::max(difference, std::abs(lhs[joint] - rhs[joint]));
  }
  return difference;
}

// Returns the uniform time scale k >= 1 needed for the sampled trajectory to
// pass the same finite-difference limit check the controller applies. Scaling
// time by k divides velocities by k and accelerations by k^2 without changing
// the geometric path.
double RequiredTimeScale(const JointTrajectory& trajectory,
                         const JointPositions& max_velocity,
                         const JointPositions& max_acceleration) {
  double scale = 1.0;
  const std::vector<JointTrajectoryPoint>& points = trajectory.points;
  for (std::size_t index = 1; index < points.size(); ++index) {
    const double dt = points[index].t - points[index - 1].t;
    for (std::size_t joint = 0; joint < max_velocity.size(); ++joint) {
      const double velocity = std::abs(points[index].positions[joint] -
                                       points[index - 1].positions[joint]) /
                              dt;
      scale = std::max(scale, velocity / max_velocity[joint]);
    }
  }
  for (std::size_t index = 1; index + 1 < points.size(); ++index) {
    const double previous_dt = points[index].t - points[index - 1].t;
    const double next_dt = points[index + 1].t - points[index].t;
    const double total_dt = previous_dt + next_dt;
    for (std::size_t joint = 0; joint < max_acceleration.size(); ++joint) {
      const double previous_velocity = (points[index].positions[joint] -
                                        points[index - 1].positions[joint]) /
                                       previous_dt;
      const double next_velocity = (points[index + 1].positions[joint] -
                                    points[index].positions[joint]) /
                                   next_dt;
      const double acceleration =
          2.0 * (next_velocity - previous_velocity) / total_dt;
      scale = std::max(
          scale, std::sqrt(std::abs(acceleration) / max_acceleration[joint]));
    }
  }
  // The analytic derivatives should already satisfy the limits; include them
  // so the result is consistent even if the solver is slightly optimistic.
  for (const JointTrajectoryPoint& point : points) {
    for (std::size_t joint = 0; joint < max_velocity.size(); ++joint) {
      scale = std::max(scale,
                       std::abs(point.velocities[joint]) / max_velocity[joint]);
      scale = std::max(scale, std::sqrt(std::abs(point.accelerations[joint]) /
                                        max_acceleration[joint]));
    }
  }
  return scale;
}

void ScaleTrajectoryTime(JointTrajectory* trajectory, double scale) {
  const double inverse = 1.0 / scale;
  const double inverse_squared = inverse * inverse;
  for (JointTrajectoryPoint& point : trajectory->points) {
    point.t *= scale;
    for (double& velocity : point.velocities) {
      velocity *= inverse;
    }
    for (double& acceleration : point.accelerations) {
      acceleration *= inverse_squared;
    }
  }
}

}  // namespace

ToppraTimeParameterizer::ToppraTimeParameterizer(std::size_t grid_points)
    : grid_points_(
          std::min(std::max(grid_points, kMinGridPoints), kMaxGridPoints)) {}

TimeParameterizationResponse ToppraTimeParameterizer::Parameterize(
    const TimeParameterizationRequest& request) {
  TimeParameterizationResponse response;
  std::cout << "====>path.size: " << request.path.points.size() << std::endl;
  const ValidationResult path_validation = ValidateJointPath(request.path);
  if (!path_validation.ok) {
    response.message = path_validation.message;
    return response;
  }
  const std::size_t joint_count = request.path.joint_names.size();
  if (request.max_velocity.size() != joint_count ||
      request.max_acceleration.size() != joint_count ||
      !IsFiniteVector(request.max_velocity) ||
      !IsFiniteVector(request.max_acceleration)) {
    response.message =
        "velocity and acceleration limits must match joint count";
    return response;
  }
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    if (request.max_velocity[joint] <= 0.0 ||
        request.max_acceleration[joint] <= 0.0) {
      response.message = "velocity and acceleration limits must be positive";
      return response;
    }
  }

  // Merge consecutive duplicates; a zero-length segment makes the spline
  // parameter non-increasing.
  std::vector<JointPositions> waypoints;
  waypoints.reserve(request.path.points.size());
  for (const JointPositions& point : request.path.points) {
    if (waypoints.empty() ||
        MaxAbsDifference(waypoints.back(), point) > kDuplicatePointTolerance) {
      waypoints.push_back(point);
    }
  }
  if (waypoints.size() < 2) {
    response.message = "toppra requires at least two distinct path points";
    return response;
  }

  try {
    toppra::Vectors positions;
    positions.reserve(waypoints.size());
    for (const JointPositions& point : waypoints) {
      toppra::Vector vector(static_cast<Eigen::Index>(joint_count));
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        vector[static_cast<Eigen::Index>(joint)] = point[joint];
      }
      positions.push_back(std::move(vector));
    }

    // Use cumulative joint-space chord length as the geometric path
    // parameter, normalized to [0, 1]. Normalization only rescales s; it does
    // not change the resulting joint trajectory.
    toppra::Vector times(static_cast<Eigen::Index>(waypoints.size()));
    times[0] = 0.0;
    double length = 0.0;
    for (std::size_t index = 1; index < positions.size(); ++index) {
      length += (positions[index] - positions[index - 1]).norm();
      times[static_cast<Eigen::Index>(index)] = length;
    }
    if (!(length > 0.0)) {
      response.message = "path has zero length in joint space";
      return response;
    }
    times /= length;

    // Natural boundary conditions set the path curvature to zero at the
    // endpoints. A straight two-point path therefore stays exactly straight
    // instead of becoming an S-curve with high endpoint curvature, which would
    // unnecessarily slow TOPP-RA down. Rest-to-rest motion is still enforced
    // by the zero start/end path velocity passed to the solver.
    // Note that the default BoundaryCond type (NotAKnot) is not implemented
    // by the C++ backend.
    const toppra::BoundaryCondFull boundary_conditions{
        toppra::BoundaryCond("natural"), toppra::BoundaryCond("natural")};
    const auto path = std::make_shared<toppra::PiecewisePolyPath>(
        toppra::PiecewisePolyPath::CubicSpline(positions, times,
                                               boundary_conditions));

    toppra::Vector max_velocity(static_cast<Eigen::Index>(joint_count));
    toppra::Vector max_acceleration(static_cast<Eigen::Index>(joint_count));
    for (std::size_t joint = 0; joint < joint_count; ++joint) {
      max_velocity[static_cast<Eigen::Index>(joint)] =
          request.max_velocity[joint];
      max_acceleration[static_cast<Eigen::Index>(joint)] =
          request.max_acceleration[joint];
    }

    toppra::LinearConstraintPtrs constraints{
        std::make_shared<toppra::constraint::LinearJointVelocity>(-max_velocity,
                                                                  max_velocity),
        std::make_shared<toppra::constraint::LinearJointAcceleration>(
            -max_acceleration, max_acceleration)};

    auto problem =
        std::make_unique<toppra::algorithm::TOPPRA>(constraints, path);
    problem->setN(static_cast<int>(grid_points_));
    // Seidel is bundled with TOPP-RA and needs no external LP/QP library.
    problem->solver(std::make_shared<toppra::solver::Seidel>());
    const toppra::ReturnCode return_code =
        problem->computePathParametrization(0.0, 0.0);
    if (return_code != toppra::ReturnCode::OK) {
      std::ostringstream message;
      message << "toppra failed with return code "
              << static_cast<int>(return_code);
      const std::string details = problem->getErrorMessage();
      if (!details.empty()) {
        message << ": " << details;
      }
      response.message = message.str();
      return response;
    }

    const toppra::ParametrizationData& data =
        problem->getParameterizationData();

    std::cout << "====>getParameterizationData: " << data.gridpoints.size()
              << std::endl;
    toppra::parametrizer::ConstAccel parametrizer{path, data.gridpoints,
                                                  data.parametrization};
    if (!parametrizer.validate()) {
      response.message =
          "toppra output failed parametrizer validation (numerical error)";
      return response;
    }

    const toppra::Vector& sample_times = parametrizer.getTimes();
    const toppra::Vectors sample_positions = parametrizer.eval(sample_times, 0);
    const toppra::Vectors sample_velocities =
        parametrizer.eval(sample_times, 1);
    const toppra::Vectors sample_accelerations =
        parametrizer.eval(sample_times, 2);

    JointTrajectory trajectory;
    trajectory.path_id = request.path.path_id;
    trajectory.joint_names = request.path.joint_names;
    trajectory.points.reserve(static_cast<std::size_t>(sample_times.size()));
    double previous_time = -1.0;
    for (Eigen::Index index = 0; index < sample_times.size(); ++index) {
      const double time = index == 0 ? 0.0 : sample_times[index];
      if (!(time > previous_time)) {
        response.message = "toppra produced a non-increasing trajectory time";
        return response;
      }
      previous_time = time;

      JointTrajectoryPoint point;
      point.t = time;
      point.positions.resize(joint_count);
      point.velocities.resize(joint_count);
      point.accelerations.resize(joint_count);
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        const Eigen::Index eigen_joint = static_cast<Eigen::Index>(joint);
        point.positions[joint] = sample_positions[index][eigen_joint];
        point.velocities[joint] = sample_velocities[index][eigen_joint];
        point.accelerations[joint] = sample_accelerations[index][eigen_joint];
      }
      trajectory.points.push_back(std::move(point));
    }

    // TOPP-RA enforces the limits at the collocation grid points. The
    // controller re-checks the sampled positions with finite differences, so
    // stretch time just enough to make that stricter check pass as well. The
    // fixed margin is applied even when required_scale is one because ROS
    // nanosecond time quantization perturbs the finite differences slightly.
    const double required_scale = RequiredTimeScale(
        trajectory, request.max_velocity, request.max_acceleration);
    ScaleTrajectoryTime(
        &trajectory, std::max(1.0, required_scale) * (1.0 + kTimeScaleMargin));

    const ValidationResult limit_validation = ValidateJointTrajectoryLimits(
        trajectory, request.max_velocity, request.max_acceleration);
    if (!limit_validation.ok) {
      response.message = "toppra trajectory failed limit validation: " +
                         limit_validation.message;
      return response;
    }

    const double duration = trajectory.points.back().t;
    std::ostringstream message;
    message << "toppra time-optimal rest-to-rest profile generated ("
            << grid_points_ << " intervals, " << trajectory.points.size()
            << " samples, duration " << duration << " s)";
    response.success = true;
    response.trajectory = std::move(trajectory);
    response.message = message.str();
    return response;
  } catch (const std::exception& error) {
    response.message =
        std::string("toppra time parameterization failed: ") + error.what();
    return response;
  }
}

}  // namespace planning
}  // namespace excavator
