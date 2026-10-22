#include <toppra/solver/glpk-wrapper.hpp>

#ifdef BUILD_WITH_GLPK
# include <glpk.h>
#endif

#include <cassert>
#include <iostream>

namespace toppra {
namespace solver {

#ifdef BUILD_WITH_GLPK

/// 计算两个区间的交集，结果存入 c
/// \param a 第一个区间 [a[0], a[1]]
/// \param b 第二个区间 [b[0], b[1]]
/// \param c 输出：交集区间 [c[0], c[1]]
void intersection (const Bound& a, const Bound& b, Bound& c)
{
  // 下界取两者的较大值
  c[0] = std::max(a[0], b[0]);
  // 上界取两者的较小值
  c[1] = std::min(a[1], b[1]);
}

/// 根据 lower 和 upper 的值确定 GLPK 边界类型常量
/// \param l 下界
/// \param u 上界
/// \return GLPK 边界类型常量：
///   - GLP_FR：自由变量（无上下界）
///   - GLP_UP：仅有上界
///   - GLP_LO：仅有下界
///   - GLP_FX：固定值（上下界相等）
///   - GLP_DB：双边界（有上下界且不相等）
int bnd_type(const value_type& l, const value_type& u)
{
  if (l == -infty) {
    // 下界为负无穷
    if (u == infty) return GLP_FR;  // 无上下界 → 自由变量
    else return GLP_UP;              // 仅有上界
  } else {
    // 有下界
    if (u < l) {
      // 上界小于下界，边界非法
      TOPPRA_LOG_DEBUG("GLPK: invalid bounds:"
          "\nlower: " << l <<
          "\nupper: " << u);
      // 不在此处直接停止，以便检查所有不等式约束
      // GLPK 后续会报告求解失败
    }
    if (l == u) return GLP_FX;           // 上下界相等 → 固定变量
    else if (u == infty) return GLP_LO;   // 上界为无穷 → 仅有下界
    else return GLP_DB;                   // 双边界
  }
}

/// 设置列（变量）的边界
/// \param lp GLPK 问题对象
/// \param i 列索引
/// \param ub 边界值 [下界, 上界]
void set_col_bnds(glp_prob* lp, int i, const Bound& ub)
{
  glp_set_col_bnds(lp, i, bnd_type(ub[0], ub[1]), ub[0], ub[1]);
}

/// 设置行（约束）的边界（通过下界和上界值）
/// \param lp GLPK 问题对象
/// \param i 行索引
/// \param l 下界
/// \param u 上界
void set_row_bnds(glp_prob* lp, int i, const value_type& l, const value_type& u)
{
  glp_set_row_bnds(lp, i, bnd_type(l,u), l, u);
}

/// 设置行（约束）的边界（通过 Bound 对象）
/// \param lp GLPK 问题对象
/// \param i 行索引
/// \param b 边界值 [下界, 上界]
void set_row_bnds(glp_prob* lp, int i, const Bound& b) { set_row_bnds(lp, i, b[0], b[1]); }
#endif

/// 初始化 GLPK 求解器，构建 LP 问题的基本框架
/// \param constraints 规范线性约束列表
/// \param path 几何路径
/// \param times 时间序列
void GLPKWrapper::initialize (const LinearConstraintPtrs& constraints, const GeometricPathPtr& path,
        const Vector& times)
{
#ifdef BUILD_WITH_GLPK
  // 调用基类初始化
  Solver::initialize (constraints, path, times);
  // 如果已存在 LP 问题对象，先释放
  if (m_lp != NULL) glp_delete_prob(m_lp);
  // 创建新的 GLPK 问题对象
  m_lp = glp_create_prob();

  // 当前仅支持规范线性约束，决策变量数为 2（u 和 x）
  assert(nbVars() == 2);

  // 设置目标函数方向为最小化
  glp_set_obj_dir(m_lp, GLP_MIN);

  // 添加 2 个列（辅助变量/决策变量）
  glp_add_cols(m_lp, 2);
  glp_set_col_name(m_lp, 1, "u");   // 第 1 列：u = s_dot^2（平方速度）
  glp_set_col_name(m_lp, 2, "x");   // 第 2 列：x = s（路径位置）
  glp_set_col_bnds(m_lp, 1, GLP_FR, 0., 0.);  // u 初始设为自由变量

  // 计算约束行数（结构变量行数）
  // 第一行是动力学约束 x_next = x + 2*delta*u，后续行来自各线性约束参数
  int nC = 1;
  for (const Solver::LinearConstraintParams& linParam : m_constraintsParams.lin)
    nC += linParam.F[0].rows();  // 累加每个线性约束的行数

  // 添加约束行
  glp_add_rows(m_lp, nC);
  glp_set_row_name(m_lp, 1, "x_next");  // 第一行：下一阶段位置约束
#else
  // 未启用 GLPK 编译支持，忽略参数并抛出异常
  TOPPRA_UNUSED(constraints);
  TOPPRA_UNUSED(path);
  TOPPRA_UNUSED(times);
  throw std::invalid_argument("TOPPRA was not built with GLPK support.");
#endif
}

/// 析构函数，释放 GLPK 问题对象
GLPKWrapper::~GLPKWrapper ()
{
#ifdef BUILD_WITH_GLPK
  if (m_lp != NULL) glp_delete_prob(m_lp);
#endif
}

/// 求解第 i 阶段的线性规划问题
/// \param i 阶段索引
/// \param H 二次项矩阵（GLPK 不支持，必须为空）
/// \param g 线性目标系数向量 [g_u, g_x]
/// \param x 位置变量 x 的上下界
/// \param xNext 下一阶段位置变量 x_next 的上下界
/// \param solution 输出的解向量
/// \return 求解成功返回 true
bool GLPKWrapper::solveStagewiseOptim(std::size_t i,
        const Matrix& H, const Vector& g,
        const Bound& x, const Bound& xNext,
        Vector& solution)
{
#ifdef BUILD_WITH_GLPK
  // GLPK 仅支持 LP，若 H 非空则抛出异常
  if (H.size() > 0)
    throw std::invalid_argument("GLPK can only solve LPs.");

  int N (nbStages());  // 总阶段数
  assert (i <= N);

  // GLPK 约束矩阵的列索引和系数数组（1-based 索引）
  const int ind[] = {0, 1, 2};
  double val[] = {0., 0., 1.};

  if (i < N) {
    // 设置第一行约束：x_next = x + 2*delta*u
    // 系数：u 的系数为 2*delta，x 的系数为 1
    val[1] = 2*deltas()[i];
    glp_set_mat_row(m_lp, 1, 2, ind, val);
    set_row_bnds(m_lp, 1, xNext);  // 设置 x_next 的上下界
  } else {
    // 最后一个阶段，无下一阶段约束
    glp_set_mat_row(m_lp, 1, 0, NULL, NULL);
    glp_set_row_bnds(m_lp, 1, GLP_FR, 0., 0.);
  }

  // 设置各线性约束行
  int iC = 2;  // 从第 2 行开始（第 1 行是动力学约束）
  for (const Solver::LinearConstraintParams& lin : m_constraintsParams.lin)
  {
    // 若 F 只有一个元素（所有阶段共享），则取索引 0；否则取当前阶段 i
    std::size_t j (lin.F.size() == 1 ? 0 : i);
    const Matrix& _F (lin.F[j]);
    const Vector& _g (lin.g[j]);
    int nC (_F.rows());

    // 对该线性约束的每一行设置系数
    for (int k = 0; k < _F.rows(); ++k) {
      // 约束：F_k * (a_i * u + b_i * x + c_i) <= g_k
      // u 的系数 = F_k * a_i，x 的系数 = F_k * b_i
      val[1] = _F.row(k) * lin.a[i];
      val[2] = _F.row(k) * lin.b[i];
      glp_set_mat_row(m_lp, iC, 2, ind, val);
      // 右端项：g_k - F_k * c_i（下界为负无穷，上界为该值）
      set_row_bnds(m_lp, iC, -infty, _g[k] - _F.row(k) * lin.c[i]);
      ++iC;
    }
  }

  // 设置 u 和 x 的上下界（盒约束）
  Bound xb(x), ub;
  ub << -infty, infty;  // u 的初始边界设为无界

  // 与盒约束取交集
  for (const Solver::BoxConstraintParams& box : m_constraintsParams.box)
  {
    if (!box.u.empty()) intersection(ub, box.u[i], ub);  // u 的边界取交集
    if (!box.x.empty()) intersection(xb, box.x[i], xb);  // x 的边界取交集
  }

  // 设置列边界
  set_col_bnds(m_lp, 1, ub);  // u 的边界
  set_col_bnds(m_lp, 2, xb);  // x 的边界

  // 设置目标函数系数：min g[0]*u + g[1]*x
  glp_set_obj_coef(m_lp, 1, g[0]);
  glp_set_obj_coef(m_lp, 2, g[1]);

  // TODO 可以尝试使用 glp_interior（内点法）替代单纯形法
  glp_smcp parm;
  glp_init_smcp(&parm);
  parm.msg_lev = GLP_MSG_ERR;  // 仅输出错误信息
  int ret = glp_simplex(m_lp, &parm);  // 执行单纯形法求解
  if (ret == 0) {
    // 求解返回正常，检查求解状态
    int status = glp_get_status(m_lp);
    switch (status) {
      case GLP_OPT:
        // 最优解
        break;
      case GLP_FEAS:
        // 解可行但非最优
        TOPPRA_LOG_DEBUG("GLPK: solution is feasible");
        break;
      case GLP_INFEAS:
        // 解不可行
        TOPPRA_LOG_DEBUG("GLPK: solution is infeasible");
        break;
      case GLP_NOFEAS:
        // 问题无可行解
        TOPPRA_LOG_DEBUG("GLPK: problem has no feasible solution");
        return false;
      case GLP_UNBND:
        // 问题有无界解
        TOPPRA_LOG_DEBUG("GLPK: problem has unbounded solution");
        break;
      case GLP_UNDEF:
        // 解未定义
        TOPPRA_LOG_DEBUG("GLPK: solution is undefined");
        return false;
    }
    // 提取解向量 [u, x]
    solution.resize(2);
    solution << glp_get_col_prim(m_lp, 1), glp_get_col_prim(m_lp, 2);
    return true;
  }
  // 求解器调用失败
  TOPPRA_LOG_DEBUG("GLPK failed. glp_simplex return value: " << ret);
  return false;
#else
  // 未启用 GLPK 编译支持
  TOPPRA_UNUSED(i);
  TOPPRA_UNUSED(H);
  TOPPRA_UNUSED(g);
  TOPPRA_UNUSED(x);
  TOPPRA_UNUSED(xNext);
  TOPPRA_UNUSED(solution);
  throw std::invalid_argument("TOPPRA was not built with GLPK support.");
#endif
}

} // namespace solver
} // namespace toppra
