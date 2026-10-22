#ifndef TOPPRA_SOLVER_SEIDEL_INTERNAL_HPP
#define TOPPRA_SOLVER_SEIDEL_INTERNAL_HPP

#include <toppra/solver/seidel.hpp>

namespace toppra {
namespace solver {

// ===== 类型别名定义 =====
// 2 维列向量
typedef Eigen::Matrix<value_type, 2, 1> Vector2;
// 2 维行向量
typedef Eigen::Matrix<value_type, 1, 2> RowVector2;
// 3 维列向量
typedef Eigen::Matrix<value_type, 3, 1> Vector3;
// 3 维行向量
typedef Eigen::Matrix<value_type, 1, 3> RowVector3;
// 动态行数、2 列的矩阵（用于 1D LP 约束）
typedef Eigen::Matrix<value_type, Eigen::Dynamic, 2> MatrixX2;
// 动态行数、3 列的矩阵（用于 2D LP 约束）
typedef Eigen::Matrix<value_type, Eigen::Dynamic, 3> MatrixX3;

namespace seidel {

// ===== 固定边界的索引常量 =====
// Seidel 算法中，固定边界（low, high）被赋予特殊的负索引编号，
// 以便在活动约束记录中区分固定边界和线性约束。
// 编号规则：low[0]=-1, high[0]=-2, low[1]=-3, high[1]=-4
constexpr int LOW_0  = -1,   // x[0] 的下界
              HIGH_0 = -2,   // x[0] 的上界
              LOW_1  = -3,   // x[1] 的下界
              HIGH_1 = -4;   // x[1] 的上界

/// 返回变量 i 的下界索引
/// \param i 变量索引（0 或 1）
/// \return LOW_0 或 LOW_1
constexpr int LOW (int i) { return (i == 0) ? LOW_0  : LOW_1 ; }

/// 返回变量 i 的上界索引
/// \param i 变量索引（0 或 1）
/// \return HIGH_0 或 HIGH_1
constexpr int HIGH(int i) { return (i == 0) ? HIGH_0 : HIGH_1; }

/// 1 维 LP 的解结构体
struct LpSol1d {
  bool feasible;          // 是否可行
  value_type optvar;      // 最优变量值
  int active_c;           // 活动约束的索引（负值表示固定边界）
};

/// 2 维 LP 的解结构体
struct LpSol {
  bool feasible;          // 是否可行
  value_type optval;      // 最优目标值
  Vector2 optvar;         // 最优变量向量
  std::array<int, 2> active_c;  // 两个活动约束的索引
};

/// 输出 1D LP 解到流（调试用）
inline std::ostream& operator<< (std::ostream& os, const LpSol1d& sol)
{
  if (!sol.feasible) return os << "infeasible";
  return os << "feasible. x* = " << sol.optvar
    << ", active_c = " << sol.active_c;
}

/// 输出 2D LP 解到流（调试用）
inline std::ostream& operator<< (std::ostream& os, const LpSol& sol)
{
  if (!sol.feasible) return os << "infeasible";
  return os << "feasible: " << sol.optval
    << "\n\toptvar: " << sol.optvar.transpose()
    << "\n\tactive_c: " << sol.active_c[0] << ' ' << sol.active_c[1]
    ;
}

// 不可行 1D LP 解的常量
constexpr LpSol1d INFEASIBLE_1D { false };

/// 避免除以小于 TINY 的值
constexpr double TINY = 1e-10;
/// 允许的约束违反阈值（相对容差）
constexpr double REL_TOLERANCE = 1e-10;
/// 允许的约束违反阈值（绝对容差）
constexpr double ABS_TOLERANCE = 1e-13;

// 正无穷常量
constexpr value_type infinity = std::numeric_limits<value_type>::infinity();

// 调试用的日志宏：打印 1D LP 问题的信息
#define TOPPRA_SEIDEL_LP1D(w,X)                         \
  TOPPRA_LOG_##w("Seidel LP 1D:\n"                      \
      << "max  v * [ x, 1 ]\n"                          \
      << "s.t. A * [ x, 1 ] <= 0\n"                     \
      << "v: " << v << '\n'                             \
      << "A:\n" << A << '\n'                            \
      << X);

/** \brief 在 DEBUG 模式下记录已求解的 1D LP 结果。
 *
 *  以 const 引用接收普通结构体，调用本身是无开销且不抛异常的，
 *  不会给 solve_lp1d 的循环增加异常展开状态。1D LP 的*输入*参数
 *  在此处故意不被记录——它们可以从 reportLp2d 已经输出的 2D LP
 *  输入中推导出来，而每次迭代都实例化 Eigen 表达式的代价不值得
 *  这种重复。
 */
void reportLp1dSol(const LpSol1d &sol);

/**
 * 求解单变量线性规划（1D LP）。
 *
 * 问题形式：
 *   max   v[0] * x + v[1]
 *   s.t.  A * [ x, 1 ]^T <= 0
 *
 * 即：对于每行约束 a[i]*x + b[i] <= 0，求使目标函数最大的 x 值。
 *
 * \param v 目标函数的系数 [v[0], v[1]]，其中 v[0] 决定优化方向
 * \param A 约束矩阵，每行 [a[i], b[i]] 表示约束 a[i]*x + b[i] <= 0
 * \return LpSol1d 结构体，包含可行性、最优解和活动约束索引
 *
 * 算法说明：
 * - 遍历所有约束，根据 a[i] 的正负判断约束方向
 * - a[i] > 0：约束限制 x 的上界
 * - a[i] < 0：约束限制 x 的下界
 * - a[i] ≈ 0：检查 b[i] 是否满足（纯可行性约束）
 * - 最终根据 v[0] 的符号选择取上界或下界作为最优解
 **/
template<typename Derived>
LpSol1d solve_lp1d(const RowVector2& v, const Eigen::MatrixBase<Derived>& A)
{
  // 初始边界为正负无穷
  value_type cur_min = -infinity,
             cur_max = infinity;
  // 记录使上下界生效的约束索引
  int active_c_min = -1,
      active_c_max = -2;
  // 提取约束矩阵的两列：a（x 的系数）和 b（常数项）
  auto a (A.col(0)), b (A.col(1));

  // 根据目标函数系数的符号决定优化方向
  bool maximize { v[0] > 0 };

  for (int i = 0; i < A.rows(); ++i) {
    // 约束：a[i] * x + b[i] <= 0
    // 简记 a[i] -> a，b[i] -> b
    //
    // 情况 a == 0.0：
    //   当 b < ABS_TOLERANCE 时可行（允许微小违反）
    // 否则：
    //   若 a>0, b<0：则 x <= +|b/a|，按正常约束处理
    //   若 a<0, b<0：则 x >= -|b/a|，按正常约束处理
    //
    //   若 a>0, b>0：则 x <= -|b/a|，当 |b/a| > 1/REL_TOLERANCE 时不可行
    //   若 a<0, b>0：则 x >= +|b/a|，当 |b/a| > 1/REL_TOLERANCE 时不可行
    if (b[i] * REL_TOLERANCE > std::abs(a[i])) {
      // b[i] 明显为正，违反约束
      TOPPRA_SEIDEL_LP1D(WARN, "-> constraint " << i << " infeasible.");
      if (std::abs(a[i]) == 0.0 && b[i] < ABS_TOLERANCE) {
        // a[i]==0 且 b[i] 极小，视为可行
        TOPPRA_LOG_WARN("but considered feasible because a["<<i<<"]==0 and b["<<i<<"]"<<ABS_TOLERANCE<<".");
        continue;
      }
      return INFEASIBLE_1D;
    }
    if (a[i] > 0) {
      // a[i] > 0：约束限制 x 的上界
      if (a[i] * cur_max + b[i] > 0) { // 当前上界违反约束
        cur_max = std::min(-b[i]/a[i], cur_max);  // 更新上界
        active_c_max = i;
      }
    } else if (a[i] < 0 && a[i] * cur_min + b[i] > 0) { // a[i] < 0：约束限制 x 的下界
      // 当前下界违反约束
      cur_min = std::max(-b[i]/a[i], cur_min);  // 更新下界
      active_c_min = i;
    }
  }

  // 若上界小于下界，但差异不超过 2*ABS_TOLERANCE，
  // 则扩展两边界以避免因数值误差导致问题不可行
  if (cur_max < cur_min && cur_min - cur_max < 2 * ABS_TOLERANCE) {
    cur_min -= ABS_TOLERANCE;
    cur_max += ABS_TOLERANCE;
  }

  // 检查边界一致性：若下界明显大于上界，则不可行
  if (   cur_min - cur_max > std::max(std::abs(cur_min),std::abs(cur_max))*REL_TOLERANCE
      || cur_min == infinity
      || cur_max == -infinity) {
    TOPPRA_SEIDEL_LP1D(WARN, "-> incoherent bounds. high - low = "
        << cur_max << " - " << cur_min << " = " << cur_max - cur_min);
    return INFEASIBLE_1D;
  }

  if (v[0] == 0) {
    // 目标函数不依赖 x，返回任意可行点
    value_type x;
    if (cur_min != -infinity)
      if (cur_max != infinity) return { true, (cur_max/2+cur_min/2), active_c_min };  // 返回中点
      else return { true, cur_min, active_c_min };  // 仅有下界，返回下界
    else return { true, cur_max, active_c_max };  // 仅有上界，返回上界
  }
  if (maximize)
    // 最大化方向：取上界
    return LpSol1d{ true, cur_max, active_c_max };
  else
    // 优化方向垂直于约束线，或指向负方向：取下界
    return LpSol1d{ true, cur_min, active_c_min };
}

#undef TOPPRA_SEIDEL_LP1D

/**
 * 求解双变量线性规划（2D LP）。
 *
 * LP 问题形式：
 *   max     v^T x
 *   s.t.    A * [ x^T, 1 ]^T <= 0     // 线性不等式约束
 *           low <= x <= high          // 变量盒约束
 *
 * 注意：该函数的一个可能优化是预先裁剪明显不可行的线性约束，
 * 但目前未实现，因为在当前代码中瓶颈不在求解 TOPP-RA 本身，
 * 而在于设置参数。
 *
 * \param v 目标函数的系数向量
 * \param A 约束矩阵，每行 [a, b, c] 表示约束 a*x0 + b*x1 + c <= 0
 * \param low 变量下界 [low[0], low[1]]
 * \param high 变量上界 [high[0], high[1]]
 * \param A_1d 预分配的 1D LP 约束矩阵视图，用于避免重复初始化开销
 *             其行数应为 A.rows() + 4（线性约束 + 4 个固定边界约束）
 *
 * \return LpSol 结构体。当 LpSol.feasible 为 false 时，其他字段无意义。
 *
 * 算法说明（Seidel 算法）：
 * 1. 首先检查固定边界是否使某一变量被固定（low[i] ≈ high[i]），
 *    若是，则降维为 1D LP 求解。
 * 2. 否则，初始化当前最优解为各变量边界的极值点。
 * 3. 逐个加入线性约束：若当前最优解违反新约束，则将问题投影到
 *    该约束定义的直线上，递归求解 1D LP 子问题。
 * 4. 投影方法：找到原点到约束直线的垂足（zero_prj）和切线方向（d_tan），
 *    将所有其他约束投影到切线上形成 1D 约束。
 **/
LpSol solve_lp2d(const RowVector2& v, const MatrixX3& A,
    const Vector2& low, const Vector2& high,
    MatrixX2& A_1d);

} // namespace seidel
} // namespace solver
} // namespace toppra

#endif
