// 本文件实现时间参数化算法基类 PathParametrizationAlgorithm 的通用流程。
//
// 算法整体基于「可达性分析 / 可控集（controllable set）」思想，在按网格点
// 离散化后的路径上进行两个方向的遍历：
//   1. 反向遍历（backward pass）：从终点速度约束出发，逐段向前回推，得到每个
//      网格点上满足约束且能到达终点的「路径速度平方」可行区间（可控集）；
//   2. 正向遍历（forward pass，由子类实现具体目标，如最短时间），在每个网格点
//      上于可控集约束内向前求解最优速度剖面。
//
// 本文件只负责：构造、模板方法 computePathParametrization、以及两个公共辅助流程
// computeControllableSets 与 computeFeasibleSets。具体前向目标由派生类实现。
#include "toppra/src/algorithm.hpp"

#include <algorithm>
#include <cstddef>
#include <iostream>

#include "excavator_common/include/g3log.h"
#include "toppra/src/toppra.hpp"

namespace toppra {

// 构造函数：保存约束列表与几何路径。
// constraints 以值传递后 move 进成员，避免不必要的拷贝；path 直接持有共享指针。
// 注意：此时并不会初始化求解器或网格点，真正的工作在 initialize() 中完成。
PathParametrizationAlgorithm::PathParametrizationAlgorithm(
    LinearConstraintPtrs constraints, const GeometricPathPtr& path)
    : m_constraints(std::move(constraints)), m_path(path){};

// 算法主入口（模板方法）：计算给定路径的时间参数化。
// vel_start / vel_end 分别为路径起点、终点的路径速度（\dot{s}），默认 0。
// 流程：初始化 -> 建立求解器 -> 反向计算可控集 -> 正向求解最优速度剖面。
// 返回值：ReturnCode::OK 表示成功；否则为具体错误码，可经 getErrorMessage()
// 查看详情。
ReturnCode PathParametrizationAlgorithm::computePathParametrization(
    value_type vel_start, value_type vel_end) {
  initialize();  // 准备网格点、结果容器并初始化求解器
  m_solver->setupSolver();  // 求解器自身的一次性设置（如 warm-start 准备）
  A3WARN << "computePathParametrization";
  // 终点处速度平方的上/下界（向量每个分量都等于 vel_end）
  Bound vel_ends;
  vel_ends.setConstant(vel_end);

  // 反向遍历：由终点约束回推每个网格点的可控集（可行路径速度平方区间）
  m_data.ret_code = computeControllableSets(vel_ends);
  // ReturnCode::OK == 0，其余错误码均为正数，故 > 0 表示失败，直接返回错误码
  if ((int)m_data.ret_code > 0) {
    return m_data.ret_code;
  }

  // 正向遍历：在可控集约束内向前求解目标（由子类实现，例如最短时间）
  m_data.ret_code = computeForwardPass(vel_start);
  return m_data.ret_code;
};

// 反向遍历计算「可控集」（controllable sets）。
// 含义：对每个网格点 i，求出满足所有约束、且从该点出发最终能到达终点
// 速度约束 vel_ends 的路径速度平方 x = \dot{s}^2 的可行区间 [lo, hi]。
// 结果写入 m_data.controllable_sets，每行为一个网格点（列 0 为下界，列 1
// 为上界）。
//
// 关键变量约定（求解器变量顺序为 [u, x, v]）：
//   - u：路径加速度相关的控制量（\ddot{s} 的离散化形式）；
//   - x：当前网格点的路径速度平方；
//   - solution[1] 即对应 x，因此取 solution[1] 作为求解结果。
//   - H 为空矩阵表示使用线性目标（LP），此处仅优化 x，故无需二次项。
ReturnCode PathParametrizationAlgorithm::computeControllableSets(
    const Bound& vel_ends) {
  TOPPRA_LOG_DEBUG("computeControllableSets");
  bool solver_ret;
  Vector g_upper{2}, g_lower{2}, solution;
  // g_upper = [ε, -1]：最小化 -x（等价于最大化 x），ε>>0 的极小项用于避免
  //   目标在 u 方向上完全退化（数值稳定性），从而求得上界。
  // g_lower = [-ε, 1]：最小化 x（即 -ε*u + x），从而求得下界。
  g_upper << 1e-9, -1;
  g_lower << -1e-9, 1;
  // 终点网格点（索引 m_N）处的可控集即给定的终点速度平方约束
  m_data.controllable_sets.row(m_N) = vel_ends.cwiseAbs2();

  Matrix H;  // 空矩阵 -> 线性目标
  // x 为当前网格点的速度平方取值范围，初值使用 m_initXBound（默认 [0,100]）；
  // x_next 为下一网格点（已计算出的可控集）的上下界。
  Bound x(m_initXBound), x_next;
  // Hoisted out of the stage loop: TOPPRA_LOG_DEBUG would otherwise be a
  // call per stage. See TOPPRA_LOG_UNCONDITIONAL in logging.hpp.
  // 译：将日志开关判断提到循环外，避免每个 stage 都调用一次日志宏。
  const bool log_dbg = log::isEnabled(log::DEBUG);
  // 从倒数第二个网格点反向遍历到第 0 个（i 为无符号，用 (size_t)-1
  // 作终止条件防下溢）
  for (std::size_t i = m_N - 1; i != (std::size_t)-1; i--) {
    if (log_dbg) TOPPRA_LOG_UNCONDITIONAL(log::DEBUG, i << ", " << m_N);
    // 下一网格点的可控集区间作为本 stage 的转移约束
    x_next = m_data.controllable_sets.row(i + 1);
    // 求上界：最大化 x，满足本 stage 可行性及 x -> x_next 的转移约束
    solver_ret =
        m_solver->solveStagewiseOptim(i, H, g_upper, x, x_next, solution);

    if (!solver_ret) {
      TOPPRA_LOG_DEBUG("Fail: controllable, upper problem, idx: " << i);
      m_errorStream
          << "Failed to compute controllable set, upper problem, at idx " << i
          << "\ng_upper = " << g_upper.transpose() << "\nx = " << x
          << "\nx_next = " << x_next << '\n';
      return ReturnCode::ERR_FAIL_CONTROLLABLE;
    }

    m_data.controllable_sets(i, 1) = solution[1];  // 记录上界

    // 求下界：最小化 x
    solver_ret =
        m_solver->solveStagewiseOptim(i, H, g_lower, x, x_next, solution);

    if (log_dbg)
      TOPPRA_LOG_UNCONDITIONAL(log::DEBUG, "down: " << solution.transpose());
    if (!solver_ret) {
      TOPPRA_LOG_DEBUG("Fail: controllable, lower problem, idx: " << i);
      m_errorStream
          << "Failed to compute controllable set, lower problem, at idx " << i
          << "\ng_lower = " << g_lower.transpose() << "\nx = " << x
          << "\nx_next = " << x_next << '\n';
      return ReturnCode::ERR_FAIL_CONTROLLABLE;
    }

    // For whatever reason, sometimes the solver return negative
    // solution despite having a set of positve bounds. This readjust
    // if the solution is negative.
    // 译：求解器偶尔会在本应为正的边界下返回负解，这里对负值做截断为 0 的修正。
    m_data.controllable_sets(i, 0) =
        std::max(0.0, solution[1]);  // 记录下界，钳制非负
  }
  return ReturnCode::OK;
}

// 正向遍历计算「可行集」（feasible sets）。
// 含义：从路径起点出发，逐网格点向前求出满足所有约束的路径速度平方 x
// 的可行区间。
// 与可控集的区别在于方向与边界条件：可控集从终点约束反向回推，可行集从起点向前
// 推进（这里起点速度平方初值取 [0,100]，即 m_initXBound 的默认值）。
// 结果写入 m_data.feasible_sets，列 0 为下界，列 1 为上界。
// 该流程主要用于诊断/辅助（例如判断问题可行性），主求解流程使用的是可控集。
ReturnCode PathParametrizationAlgorithm::computeFeasibleSets() {
  initialize();
  bool solver_ret;
  Vector g_upper{2}, g_lower{2}, solution;
  // 目标函数与 computeControllableSets 相同：分别求 x 的上、下界
  g_upper << 1e-9, -1;
  g_lower << -1e-9, 1;

  Matrix H;  // 空矩阵 -> 线性目标
  // 起点处速度平方的初始可行范围，两个变量初值均取 [0,100]
  Bound x, x_next;
  x << 0, 100;
  x_next << 0, 100;
  // 从第 0 个网格点正向遍历到第 m_N 个（共 m_N+1 个网格点）
  for (std::size_t i = 0; i < m_N + 1; i++) {
    solver_ret =
        m_solver->solveStagewiseOptim(i, H, g_upper, x, x_next, solution);

    if (!solver_ret) {
      TOPPRA_LOG_DEBUG("Fail: feasible, upper problem, idx: " << i);
      m_errorStream << "Failed to compute feasible set, upper problem, at idx "
                    << i << "\ng_upper = " << g_upper.transpose()
                    << "\nx = " << x << "\nx_next = " << x_next << '\n';
      return ReturnCode::ERR_FAIL_FEASIBLE;
    }

    m_data.feasible_sets(i, 1) = solution[1];  // 记录上界

    // 求下界：最小化 x
    solver_ret =
        m_solver->solveStagewiseOptim(i, H, g_lower, x, x_next, solution);

    if (!solver_ret) {
      TOPPRA_LOG_DEBUG("Fail: feasible, lower problem, idx: " << i);
      m_errorStream << "Failed to compute feasible set, lower problem, at idx "
                    << i << "\ng_lower = " << g_lower.transpose()
                    << "\nx = " << x << "\nx_next = " << x_next << '\n';
      return ReturnCode::ERR_FAIL_FEASIBLE;
    }

    m_data.feasible_sets(i, 0) = solution[1];  // 记录下界
  }
  return ReturnCode::OK;
}

// 手动设置离散化网格点（路径参数 s 的取值序列）。
// 会在写入前做两项校验，保证网格点可用于后续离散化求解：
//   1. 首、末网格点必须与路径参数区间 [s_min, s_max] 的端点一致；
//   2. 网格点必须严格单调递增（相邻差值必须 > 0）。
// 设置后把 m_initialized 置为 false，强制下次求解重新初始化。
void PathParametrizationAlgorithm::setGridpoints(const Vector& gridpoints) {
  Bound I(m_path->pathInterval());  // 路径参数区间 [s_min, s_max]
  auto s = gridpoints.size();
  m_N = s - 1;  // 网格点数量 = 段数 + 1
  // 校验：首末点需与路径区间端点吻合（允许 TOPPRA_NEARLY_ZERO 级别的数值误差）
  if (std::fabs(gridpoints[0] - I(0)) > TOPPRA_NEARLY_ZERO ||
      std::fabs(gridpoints[gridpoints.size() - 1] - I(1)) > TOPPRA_NEARLY_ZERO)
    throw std::invalid_argument(
        "Path interval does not match first and last gridpoints.");
  // 校验：相邻网格点差值必须严格大于 0（tail - head 得到相邻差向量）
  if (((gridpoints.tail(s - 1) - gridpoints.head(s - 1)).array() <= 0).any())
    throw std::invalid_argument("gridpoints should be strictly increasing.");
  m_data.gridpoints = gridpoints;
  m_initialized = false;  // 需要重新初始化
}

// 初始化：准备网格点、分配结果容器，并初始化求解器。
// 该函数是幂等的——若已初始化过（m_initialized 为 true）则直接返回。
// 网格点的确定规则（按优先级）：
//   1. 若已通过 setGridpoints 手动设置，则直接使用（但数量必须与 m_N 匹配）；
//   2. 否则若 m_N == 0，则由路径自身 proposeGridpoints() 自动推荐；
//   3. 否则在路径参数区间 [s_min, s_max] 上线性等分生成 m_N + 1 个点。
void PathParametrizationAlgorithm::initialize() {
  m_errorStream = std::stringstream();  // 清空上一轮的错误信息
  if (m_initialized) return;            // 已初始化，无需重复
  if (!m_solver)
    throw std::logic_error(
        "You must set a solver first.");  // 必须先设置 solver
  Bound I(m_path->pathInterval());        // 路径参数区间 [s_min, s_max]

  // Intiialize the gridpoints based on N
  // 译：根据 N 初始化网格点
  if (m_data.gridpoints.size() == 0) {
    // If the number of gridpoint is set to 0, try to find automatically a
    // reasonable gridpoint
    // 译：若网格点数量为 0（且 N 也为 0），则由路径自动推荐一组合理网格点
    if (m_N == 0) {
      m_data.gridpoints = m_path->proposeGridpoints();
      m_N = m_data.gridpoints.size() - 1;
    }

    // Otherwise use linearly spaced gridpoints
    // 译：否则在路径区间上线性等分生成网格点
    else
      m_data.gridpoints = Vector::LinSpaced(m_N + 1, I(0), I(1));

    TOPPRA_LOG_DEBUG("gridpoints used:" << m_data.gridpoints.transpose());
  } else if (m_data.gridpoints.size() != m_N + 1)
    // 手动设置的网格点数量必须与 m_N 一致，否则数据不一致
    throw std::invalid_argument(
        "number of gridpoints does not match attribute N.");

  // 分配输出容器：每个网格点对应一行
  auto s = m_data.gridpoints.size();
  m_data.parametrization.resize(s);  // 最终速度剖面（路径速度平方）
  m_data.controllable_sets.resize(s, 2);  // 可控集：列 0 下界，列 1 上界
  m_data.feasible_sets.resize(s, 2);  // 可行集：列 0 下界，列 1 上界
  // 将约束、路径与网格点交给具体求解器做内部预计算
  m_solver->initialize(m_constraints, m_path, m_data.gridpoints);
  m_initialized = true;
}

}  // namespace toppra
