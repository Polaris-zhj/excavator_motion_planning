#include "toppra/src/toppra.hpp"

#include <memory>
#include <toppra/algorithm/toppra.hpp>

#include "toppra/src/algorithm.hpp"

namespace toppra {
namespace algorithm {

TOPPRA::TOPPRA(LinearConstraintPtrs constraints, const GeometricPathPtr& path)
    : PathParametrizationAlgorithm{std::move(constraints), path} {
  m_solver = Solver::createDefault();
}

ReturnCode TOPPRA::computeForwardPass(value_type vel_start) {
  TOPPRA_LOG_DEBUG("computeForwardPass");
  bool solver_ret;
  Vector g_upper{2}, solution{2};
  Matrix H;
  auto deltas = m_solver->deltas();
  Bound x, x_next;
  m_data.parametrization(0) = vel_start;
  // Hoisted out of the stage loop: TOPPRA_LOG_DEBUG would otherwise be a
  // call per stage. See TOPPRA_LOG_UNCONDITIONAL in logging.hpp.
  const bool log_dbg = log::isEnabled(log::DEBUG);
  for (std::size_t i = 0; i < m_data.parametrization.size() - 1; i++) {
    if (log_dbg) TOPPRA_LOG_UNCONDITIONAL(log::DEBUG, "i: " << i);
    g_upper << -2 * deltas(i), -1;
    x.setConstant(m_data.parametrization(i));
    x_next = m_data.controllable_sets.row(i + 1);
    solver_ret =
        m_solver->solveStagewiseOptim(i, H, g_upper, x, x_next, solution);
    if (!solver_ret) {
      TOPPRA_LOG_WARN("Fail: forward pass, idx: " << i);
      m_errorStream << "Forward pass failed at idx " << i
                    << "\ng_upper = " << g_upper.transpose() << "\nx = " << x
                    << "\nx_next = " << x_next << '\n';
      return ReturnCode::ERR_FAIL_FORWARD_PASS;
    }
    /// \todo This can be optimized further by solving a 1D problem instead of
    /// 2D
    // Claim the output to be within the controllable sets.
    m_data.parametrization(i + 1) = std::min(
        m_data.controllable_sets(i + 1, 1),
        std::max(m_data.controllable_sets(i + 1, 0),
                 m_data.parametrization(i) + 2 * deltas(i) * solution(0)));
    if (log_dbg)
      TOPPRA_LOG_UNCONDITIONAL(
          log::DEBUG, "Ok: u[" << i << "]= " << solution(0) << "x[" << i + 1
                               << "]=" << m_data.parametrization(i + 1));
  }

  return ReturnCode::OK;
};

}  // namespace algorithm
}  // namespace toppra
