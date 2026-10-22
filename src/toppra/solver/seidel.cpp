#include <cassert>

#include <toppra/solver/seidel.hpp>
#include <toppra/solver/seidel-internal.hpp>

namespace toppra {
namespace solver {

namespace seidel {

// 不可行 2D LP 解的常量
const LpSol INFEASIBLE { false };

/// 计算约束 coeffs * [v, 1] 的值，处理 v 中无穷值的情况。
/// 当 v[i] 为正负无穷且 coeffs[i] 为 0 时，将 coeffs[i]*v[i] 视为 0
/// （因为 0 * inf 在浮点运算中为 NaN，需要特殊处理）。
/// \param coeffs 约束系数行向量 [c0, c1, ..., cN, c_const]
/// \param vars 变量列向量 [v0, v1, ..., vN]^T
/// \return 约束值 = c_const + sum(coeffs[i] * vars[i])
template<class Coeffs, class Vars>
typename Coeffs::Scalar value(const Eigen::MatrixBase<Coeffs>& coeffs,
    const Eigen::MatrixBase<Vars>& vars)
{
  // 静态断言：coeffs 为 1x(N+1)，vars 为 Nx1
  static_assert(Coeffs::RowsAtCompileTime == 1
      && Vars::ColsAtCompileTime == 1
      && Coeffs::ColsAtCompileTime == Vars::RowsAtCompileTime + 1,
      "Size mismatch between coefficient (1x(N+1)) and vars (Nx1)");
  // 初始化结果为常数项
  typename Coeffs::Scalar res = coeffs(coeffs.cols()-1);
  for (int i = 0; i < Vars::RowsAtCompileTime; ++i)
    // 若系数为 0 且变量为无穷，则该项贡献为 0，避免 NaN
    res += (coeffs[i]==0 && !std::isfinite(vars[i]) ? 0 : coeffs[i]*vars[i]);
  return res;
}

namespace internal {
  /// 将线性约束投影到切线上，生成 1D LP 的约束系数
  /// \param Aj 原始 2D 约束行向量 [a, b, c]（表示 a*x0 + b*x1 + c <= 0）
  /// \param d_tan 切线方向向量（沿约束直线的方向）
  /// \param zero_prj 原点在约束直线上的投影点
  /// \param Aj_1d_ 输出：投影后的 1D 约束 [a_1d, b_1d]
  ///
  /// 投影公式：
  ///   a_1d = Aj.head<2> * d_tan       （约束沿切线方向的系数）
  ///   b_1d = value(Aj, zero_prj)       （约束在投影点处的值）
  template<typename Derived, typename Derived2>
  inline void project_linear_constraint (const Eigen::MatrixBase<Derived>& Aj,
      const Vector2& d_tan, const Vector2& zero_prj,
      const Eigen::MatrixBase<Derived2>& Aj_1d_)
  {
    Derived2& Aj_1d = const_cast<Derived2&>(Aj_1d_.derived());
    Aj_1d <<
      Aj.template head<2>() * d_tan,
      value(Aj, zero_prj);
  }
}

// 调试用的日志宏：打印 2D LP 问题的信息
#define TOPPRA_SEIDEL_LP2D(w,X)                         \
  TOPPRA_LOG_##w("Seidel LP 2D:\n"                      \
      << "v: " << v << '\n'                             \
      << "A:\n" << A << '\n'                            \
      << low[0] << "\t<= x[0] <= " << high[0] << '\n'    \
      << low[1] << "\t<= x[1] <= " << high[1] << '\n'    \
      << X);

namespace detail {

// 以下报告函数从 solve_lp2d 和 solve_lp1d 中调用，这两个函数是
// 求解器的热点路径。因此遵循两条规则：
//
//  1. 它们被声明为非内联（out-of-line），这样构建流式表达式不会
//     给调用方添加异常展开状态。在 solve_lp2d 中声明一个带析构函数
//     的局部对象会导致约 16% 的性能下降（即使运行时该分支从未执行）。
//  2. 它们只接收廉价的、不抛异常的参数：引用、int 和 const char*。
//     在调用处格式化 std::string（即传入 std::string 所需的操作）
//     会导致每次调用都进行堆分配，绝非免费。
//
// 在 DEBUG 未启用时，以下函数均为空操作。

/// 报告 2D LP 的输入参数（调试用）
void reportLp2d(const RowVector2 &v, const MatrixX3 &A, const Vector2 &low,
                const Vector2 &high) {
  if (!::toppra::log::isEnabled(::toppra::log::DEBUG)) return;
  TOPPRA_LOG_UNCONDITIONAL(::toppra::log::DEBUG,
      "Seidel LP 2D:\n"
      << "v: " << v << '\n'
      << "A:\n" << A << '\n'
      << low[0] << "\t<= x[0] <= " << high[0] << '\n'
      << low[1] << "\t<= x[1] <= " << high[1]);
}

/// 报告约束违反情况（调试用）
/// \param i 约束索引
/// \param amount 违反量（应 <= 0，正值表示违反）
void reportLp2dViolation(int i, value_type amount) {
  if (!::toppra::log::isEnabled(::toppra::log::DEBUG)) return;
  TOPPRA_LOG_UNCONDITIONAL(::toppra::log::DEBUG,
      "Seidel 2D: constraint " << i << " violated (" << amount
      << " should be <= 0).");
}

/// 报告 2D LP 的解（调试用）
/// \param optvar 最优变量
/// \param active0 第一个活动约束索引
/// \param active1 第二个活动约束索引
void reportLp2dSol(const Vector2 &optvar, int active0, int active1) {
  if (!::toppra::log::isEnabled(::toppra::log::DEBUG)) return;
  TOPPRA_LOG_UNCONDITIONAL(::toppra::log::DEBUG,
      "Seidel solution: " << optvar.transpose()
      << "\n active constraints " << active0 << " " << active1);
}

}  // namespace detail

/// 报告 1D LP 的解（调试用）
/// \param sol 1D LP 解结构体
void reportLp1dSol(const LpSol1d &sol) {
  if (!::toppra::log::isEnabled(::toppra::log::DEBUG)) return;
  TOPPRA_LOG_UNCONDITIONAL(::toppra::log::DEBUG,
                           "Seidel LP 1D solution:\n" << sol);
}

/// 求解双变量线性规划（2D LP）—— Seidel 算法核心实现
/// \param v 目标函数系数向量
/// \param A 约束矩阵，每行 [a, b, c] 表示约束 a*x0 + b*x1 + c <= 0
/// \param low 变量下界
/// \param high 变量上界
/// \param A_1d 预分配的 1D LP 约束矩阵（行数 = A.rows() + 4）
/// \return LP 解结构体
LpSol solve_lp2d(const RowVector2& v, const MatrixX3& A,
    const Vector2& low, const Vector2& high,
    MatrixX2& A_1d)
{
  // A_1d 需要有足够空间容纳所有线性约束 + 4 个固定边界约束
  assert(A_1d.rows() == A.rows()+4);

  // 约束行数
  unsigned int nrows = A.rows();
  Vector2 cur_optvar;

  // 用于求解 1D 子问题的解
  LpSol sol;

  // 性能优化：将日志级别检查提升到循环外，避免每次调用都重新检测
  // 这样禁用日志时整个函数只需一次加载和一次分支判断
  const bool log_dbg = ::toppra::log::isEnabled(::toppra::log::DEBUG);
  if (log_dbg) detail::reportLp2d(v, A, low, high);

  // 处理固定边界（low, high）。遵循以下约定：
  // 固定边界被赋予编号 -1, -2, -3, -4，分别对应：
  // low[0], high[0], low[1], high[1]
  for (int i = 0; i < 2; ++i) {
    if (low[i] > high[i]) {
      // 下界大于上界
      // 若差异足够大，则返回不可行
      if (   low[i] - high[i] > std::max(std::abs(low[i]),std::abs(high[i]))* REL_TOLERANCE
          || low[i] == infinity
          || high[i] == -infinity) {
        TOPPRA_SEIDEL_LP2D(WARN,
            "-> incoherent bounds. high - low = " << (high - low).transpose());
        return INFEASIBLE;
      }
      // 否则，认为变量 i 被固定，问题降维为 1D LP
      int j = 1-i;  // 另一个变量
      // 固定变量 i 的值为上下界中点
      sol.optvar[i] = (low[i]+high[i])/2;
      // 将线性约束投影到变量 j 的轴上
      // A_1d 的前 nrows 行：[A.col(j), A.col(2) + optvar[i]*A.col(i)]
      A_1d.topRows(nrows) << A.col(j), A.col(2) + sol.optvar[i] * A.col(i);
      // 添加变量 j 的固定边界约束
      A_1d.middleRows<2>(nrows) << -1,   low[j],   // low[j] <= x_j
                                    1, -high[j];   // x_j <= high[j]

      // 求解降维后的 1D LP
      LpSol1d sol_1d = solve_lp1d({ v[j], 0. }, A_1d.topRows(nrows+2));
      if (!sol_1d.feasible) {
        TOPPRA_SEIDEL_LP2D(WARN, "-> infeasible");
        return INFEASIBLE;
      }
      sol.optvar[j] = sol_1d.optvar;
      sol.feasible = true;
      sol.optval = v * sol.optvar;
      // 记录活动约束
      sol.active_c[0] = v[i] > 0 ? HIGH(i) : LOW(i);  // 变量 i 的固定边界
      // 变量 j 的活动约束取决于 1D LP 的结果
      switch(sol_1d.active_c - nrows) {
        case 0: sol.active_c[1] = LOW (j); break;   // 下界生效
        case 1: sol.active_c[1] = HIGH(j); break;   // 上界生效
        default: sol.active_c[1] = sol_1d.active_c;  // 线性约束生效
      }
      return sol;
    }

    // 变量未被固定：初始化当前最优解为边界极值点
    // v[i] > 0 时取上界（最大化），v[i] <= 0 时取下界
    cur_optvar[i]   = v[i] > 0 ? high[i] : low[i];
    sol.active_c[i] = v[i] > 0 ? HIGH(i) : LOW(i);
  }

  // 预处理不等式，移除冗余约束（注释说明，实际通过 continue 跳过满足的约束）

  // 逐个处理线性约束
  for (int i = 0; i < nrows; ++i) {
    // 若当前最优解满足第 i 个约束，跳过
    if (value(A.row(i), cur_optvar) < ABS_TOLERANCE)
      continue;
    // 否则，将所有约束投影到第 i 个约束定义的直线上
    sol.active_c[0] = i;
    // 将原点 (0, 0) 投影到新约束直线上
    // 设约束为 a*x + b*y + c = 0，则投影点为：
    //     zero_prj = 1 / (a^2 + b^2) * [a  -b] * [-c]
    //                                 [b   a]   [ 0]
    // 这可利用垂直性质推导得出
    Vector2 zero_prj (A.row(i).head<2>());
    zero_prj *= - A(i,2) / zero_prj.squaredNorm();

    // 设 x = zero_prj + d_tan * t，其中 t 是 1D LP 的变量
    // d_tan 是与约束直线平行的方向向量
    Vector2 d_tan { -A(i,1), A(i,0) }; // 平行于直线的方向向量
    // 1D 目标函数：v 在切线方向上的投影
    Vector2 v_1d { v * d_tan, 0 };

    // 1D LP 的约束规模：4（固定边界）+ i（已处理的线性约束）
    // 计算各线性约束在 1D LP 中对应的约束参数
    for (int j = 0; j < i; ++j)
      internal::project_linear_constraint(A.row(j),
              d_tan, zero_prj, A_1d.row(j));
    //A_1d.topRows(k) << A.topRows(k) * d_tan, value(A.topRows(k), zero_prj);
    // 计算 4 个固定边界约束在 1D LP 中的投影
    // 处理 low[0] <= x0
    internal::project_linear_constraint(RowVector3 { -1., 0., low[0] },
        d_tan, zero_prj, A_1d.row(i));
    // 处理 x0 <= high[0]
    internal::project_linear_constraint(RowVector3 { 1., 0., -high[0] },
        d_tan, zero_prj, A_1d.row(i+1));
    // 处理 low[1] <= x1
    internal::project_linear_constraint(RowVector3 { 0., -1., low[1] },
        d_tan, zero_prj, A_1d.row(i+2));
    // 处理 x1 <= high[1]
    internal::project_linear_constraint(RowVector3 { 0., 1., -high[1] },
        d_tan, zero_prj, A_1d.row(i+3));

    // 求解投影后的 1D LP
    LpSol1d sol_1d = solve_lp1d(v_1d, A_1d.topRows(4+i));
    if (log_dbg) reportLp1dSol(sol_1d);

    if (!sol_1d.feasible) {
      TOPPRA_SEIDEL_LP2D(WARN, "-> infeasible");
      return INFEASIBLE;
    }

    // 可行，更新当前最优解
    // 将 1D 解映射回 2D 空间：x = zero_prj + t * d_tan
    cur_optvar = zero_prj + sol_1d.optvar * d_tan;
    // 记录活动约束索引
    assert(sol_1d.active_c >= -2 && sol_1d.active_c < i+4);
    if (sol_1d.active_c < 0) // 无界情况
      sol.active_c[i] = sol_1d.active_c;
    else if (sol_1d.active_c >= i) // 固定边界约束
      sol.active_c[1] = i - sol_1d.active_c - 1;
    else // 线性约束
      sol.active_c[1] = sol_1d.active_c;
  }

  // 调试用完整性检查：验证所有约束是否满足
  // 消息只格式化一次，在循环外构建，然后逐条发出；
  // 在循环内构建会导致每次迭代都进行堆分配
  if (log_dbg) {
    TOPPRA_LOG_MSG(violated_msg, ::toppra::log::DEBUG,
                   "Seidel 2D: constraint violated (should be <= 0).");
    for (int i = 0; i < nrows; ++i) {
      value_type v = value(A.row(i), cur_optvar);
      if (v > 0) TOPPRA_LOG_SEND(violated_msg);
    }
  }

  if (log_dbg) detail::reportLp2dSol(cur_optvar, sol.active_c[0], sol.active_c[1]);

  // 填充解结构体
  sol.feasible = true;
  sol.optvar = cur_optvar;
  sol.optval = v * sol.optvar;
  return sol;
}

#undef TOPPRA_SEIDEL_LP2D

} // namespace seidel

/// 初始化 Seidel 求解器，预计算各阶段的约束系数
/// \param constraints 规范线性约束列表
/// \param path 几何路径对象
/// \param times 时间序列
void Seidel::initialize (const LinearConstraintPtrs& constraints, const GeometricPathPtr& path,
        const Vector& times)
{
  // 调用基类初始化
  Solver::initialize (constraints, path, times);

  // 当前仅支持规范线性约束，决策变量数为 2（u 和 x）
  assert(nbVars() == 2);

  // 计算约束总数
  // 前 2 行是动力学约束（x_next 上下界），后续行来自各线性约束
  int nC = 2;
  for (const Solver::LinearConstraintParams& linParam : m_constraintsParams.lin)
    nC += linParam.F[0].rows();

  int N (nbStages());  // 总阶段数

  // 为 2D LP 预分配约束系数矩阵 m_A、下界 m_low 和上界 m_high
  // 这些数组的第一维均为 N+1（对应 N+1 个阶段点）
  m_A.assign(N+1, MatrixX3::Zero(nC, 3));               // 每阶段一个 nC×3 的约束矩阵
  m_low  = MatrixX2::Constant(N + 1, 2, -seidel::infinity);  // 下界初始化为负无穷
  m_high = MatrixX2::Constant(N + 1, 2,  seidel::infinity);  // 上界初始化为正无穷
  int cur_index = 2;  // 从第 3 行（索引 2）开始填充线性约束

  // 填充各线性约束的系数到 m_A 的对应行
  // 约束形式：F * (a_i*u + b_i*x + c_i) <= g_i
  // 即：F*a_i*u + F*b_i*x + (F*c_i - g_i) <= 0
  // 对应 m_A 的列为 [F*a_i, F*b_i, F*c_i - g_i]
  for (const Solver::LinearConstraintParams& p : m_constraintsParams.lin) {
    assert(N+1 == p.a.size());

    auto nC_ = p.F[0].rows();
    // 若所有阶段共享同一 F 矩阵，则 identical 为 true
    const bool identical = (p.F.size() == 1);
    for (int i = 0; i < N+1; ++i) {
      int k = (identical ? 0 : i);
      // 填充约束系数：[F[k]*a[i], F[k]*b[i], F[k]*c[i] - g[k]]
      m_A[i].middleRows(cur_index, nC_) <<
        p.F[k] * p.a[i], p.F[k] * p.b[i], p.F[k] * p.c[i] - p.g[k];
    }
    cur_index += nC_;
  }

  // 处理盒约束，更新 m_low 和 m_high
  for (const Solver::BoxConstraintParams& p : m_constraintsParams.box) {
    if (!p.u.empty()) {
      // u 的盒约束：取与现有边界的交集
      assert(p.u.size() == N+1);
      for (int i = 0; i < N+1; ++i) {
        m_low (i, 0) = std::max(m_low (i, 0), p.u[i][0]);   // 下界取较大值
        m_high(i, 0) = std::min(m_high(i, 0), p.u[i][1]);   // 上界取较小值
      }
    }
    if (!p.x.empty()) {
      // x 的盒约束：取与现有边界的交集
      assert(p.x.size() == N+1);
      for (int i = 0; i < N+1; ++i) {
        m_low (i, 1) = std::max(m_low (i, 1), p.x[i][0]);
        m_high(i, 1) = std::min(m_high(i, 1), p.x[i][1]);
      }
    }
  }

  // 为 1D LP 子问题预分配约束矩阵
  // 行数 = nC + 4（线性约束行 + 4 个固定边界约束）
  m_A_1d = MatrixX2::Zero(nC + 4, 2);
}

/// 求解第 i 阶段的线性规划问题
/// \param i 阶段索引
/// \param H 二次项矩阵（Seidel 不支持，必须为空）
/// \param g 线性目标系数向量
/// \param x 位置变量 x 的上下界
/// \param xNext 下一阶段位置变量的上下界
/// \param solution 输出的解向量
/// \return 求解成功返回 true
bool Seidel::solveStagewiseOptim(std::size_t i,
        const Matrix& H, const Vector& g,
        const Bound& x, const Bound& xNext,
        Vector& solution)
{
  // Seidel 仅支持 LP，若 H 非空则抛出异常
  if (H.size() > 0)
    throw std::invalid_argument("Seidel can only solve LPs.");

  int N (nbStages());
  assert (i <= N && 0 <= i);

  // 提取当前阶段的变量边界
  Vector2 low (m_low.row(i)),
          high (m_high.row(i));

  // 处理 x 的上下界约束：x_min <= x_i <= x_max
  low[1] = std::max(low[1], x[0]);
  high[1] = std::min(high[1], x[1]);

  // 处理下一阶段约束：x_next_min <= 2*delta*u + x_i <= x_next_max
  if (i < N) {
    value_type delta = deltas()[i];
    // 第一行约束：-(2*delta*u + x) <= -xNext_min
    // 即 a*u + b*x + c <= 0，其中 a=-2*delta, b=-1, c=xNext_min
    if (xNext[0] <= -seidel::infinity) // TODO isnan(x_next_min)
      m_A[i].row(0) << 0, 0, -1;  // x_next_min 为负无穷，约束无效
    else
      m_A[i].row(0) << -2*delta, -1, xNext[0];
    // 第二行约束：2*delta*u + x <= xNext_max
    // 即 a*u + b*x + c <= 0，其中 a=2*delta, b=1, c=-xNext_max
    if (xNext[1] >= seidel::infinity) // TODO isnan(x_next_max)
      m_A[i].row(1) << 0, 0, -1;  // x_next_max 为正无穷，约束无效
    else
      m_A[i].row(1) << 2*delta, 1, -xNext[1];
  } else {
    // 最后一个阶段，无下一阶段约束，将前两行设为无效约束
    m_A[i].topLeftCorner<2,2>().setZero();
    m_A[i].topRightCorner<2,1>().setConstant(-1);
  }

  // 目标函数：Seidel 求解器执行最大化，因此取 g 的负值
  // 原问题 min g^T y → Seidel 的 max (-g)^T y
  RowVector2 v (-g.head<2>());

  // 注意：m_A[i] 的行曾经被复制到一个 m_A_ordered 缓冲区中，
  // 将之前活动的约束移到前面，期望 solve_lp2d 能利用此作为热启动。
  // 但 solve_lp2d 从未利用这一信息——它直接接收 m_A[i]——
  // 因此这种重排只是每个阶段一次 O(k) 的复制，没有任何代码读取它。
  // 已移除。

  // 调用 Seidel 算法求解 2D LP
  seidel::LpSol lpsol = seidel::solve_lp2d(v, m_A[i], low, high, m_A_1d);
  if (lpsol.feasible) {
    // 求解成功，提取最优变量
    solution = lpsol.optvar;
    return true;
  }
  // 求解失败，输出警告日志
  TOPPRA_LOG_WARN("Seidel: solver fails"
      "\ni = " << i <<
      "\ng = " << g.transpose() <<
      "\nx = " << x.transpose() <<
      "\nxNext = " << xNext.transpose() <<
      "\n"
  );
  return false;
}

} // namespace solver
} // namespace toppra
