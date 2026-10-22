#include <cassert>

#include <toppra/solver/qpOASES-wrapper.hpp>

#ifdef BUILD_WITH_qpOASES
#include <qpOASES.hpp>
#endif
#include <toppra/toppra.hpp>

namespace toppra {
namespace solver {

/// qpOASES 求解器的内部实现结构体（Pimpl 惯用法）
/// 封装了 qpOASES::SQProblem 对象，避免在头文件中暴露 qpOASES 头文件依赖
struct qpOASESWrapper::Impl {
#ifdef BUILD_WITH_qpOASES
  qpOASES::SQProblem qp;  // qpOASES 二次/线性规划求解器对象

  /// 构造函数，初始化求解器并设置选项
  /// \param nV 变量数
  /// \param nC 约束数
  Impl(Eigen::Index nV, Eigen::Index nC)
    : qp (nV, nC)
  {
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;  // 关闭输出
    qp.setOptions( options );
  }
#endif
};

// 静态成员变量初始化：默认边界值为 1e16（用于替代无穷大）
value_type qpOASESWrapper::m_defaultBoundary = 1e16;

/// 设置默认边界值（静态方法）
/// \param v 新的默认边界值
void qpOASESWrapper::setDefaultBoundary (const value_type& v)
{
  m_defaultBoundary = v;
}

// 默认构造函数
qpOASESWrapper::qpOASESWrapper () {}

/// 初始化 qpOASES 求解器，预分配约束矩阵和边界向量
/// \param constraints 规范线性约束列表
/// \param path 几何路径对象
/// \param times 时间序列
void qpOASESWrapper::initialize (const LinearConstraintPtrs& constraints, const GeometricPathPtr& path,
        const Vector& times)
{
#ifdef BUILD_WITH_qpOASES
  // 调用基类初始化
  Solver::initialize (constraints, path, times);
  // 设置实例边界值为默认边界值
  m_boundary = m_defaultBoundary;

  // 当前仅支持规范线性约束，决策变量数为 2（u 和 x）
  // 第一个约束：x + 2*D*u <= xnext_max（下一阶段位置上界）
  // 第二个约束：xnext_min <= x + 2*D*u（下一阶段位置下界）
  Eigen::Index nC = 2;
  // 累加各线性约束的行数
  for (const Solver::LinearConstraintParams& linParam : m_constraintsParams.lin)
    nC += linParam.F[0].rows();

  Eigen::Index nV (nbVars());
  assert(nV == 2);
  // 预分配约束矩阵和边界向量
  m_A  = RMatrix::Zero(nC, nV);   // 约束系数矩阵，初始化为零
  m_lA = -Vector::Ones(nC);        // 约束下界，初始化为 -1
  m_hA = -Vector::Ones(nC);        // 约束上界，初始化为 -1

  // 创建 qpOASES 求解器实例
  m_impl = std::unique_ptr<Impl>(new Impl(nV, nC));
#else
  // 未启用 qpOASES 编译支持，忽略参数并抛出异常
  TOPPRA_UNUSED(constraints);
  TOPPRA_UNUSED(path);
  TOPPRA_UNUSED(times);
  throw std::invalid_argument("TOPPRA was not built with qpOASES support.");
#endif
}

// 析构函数（m_impl 的 unique_ptr 会自动释放资源）
qpOASESWrapper::~qpOASESWrapper ()
{
}

/// 求解第 i 阶段的 QP/LP 问题
/// \param i 阶段索引
/// \param H Hessian 矩阵（为空时求解 LP）
/// \param g 线性目标系数向量
/// \param x 位置变量 x 的上下界
/// \param xNext 下一阶段位置变量的上下界
/// \param solution 输出的解向量
/// \return 求解成功返回 true
bool qpOASESWrapper::solveStagewiseOptim(std::size_t i,
        const Matrix& H, const Vector& g,
        const Bound& x, const Bound& xNext,
        Vector& solution)
{
#ifdef BUILD_WITH_qpOASES
  TOPPRA_LOG_DEBUG("stage: i="<<i);
  Eigen::Index N (nbStages());  // 总阶段数
  assert (i <= N);
  assert(x(0) <= x(1));

  // 初始化变量的盒约束边界，默认为 ±m_boundary（替代无穷大）
  Bound l (Bound::Constant(-m_boundary)),
        h (Bound::Constant( m_boundary));

  // 使用 x 的上下界更新位置变量 x 的边界
  l[1] = std::max(l[1], x[0]);
  h[1] = std::min(h[1], x[1]);

  if (i < N) {
    // 非最后阶段：设置动力学约束
    // 约束 1：-(2*delta*u + x) <= -xNext_min  →  -2*delta*u - x <= -xNext_min
    value_type delta = deltas()[i];
    m_A.row(0) << -2 * delta, -1;
    m_hA[0] = std::min(- xNext[0],m_boundary);  // 上界
    m_lA[0] = - m_boundary;                       // 下界（负无穷替代值）

    // 约束 2：2*delta*u + x <= xNext_max
    m_A.row(1) << 2 * delta, 1;
    m_hA[1] = std::min(xNext[1],m_boundary);  // 上界
    m_lA[1] = -m_boundary;                     // 下界（负无穷替代值）
  } else {
    // 最后一个阶段：无动力学约束，将前两行约束置零
    m_A.topRows<2>().setZero();
    m_lA.head<2>().setConstant(-1);
    m_hA.head<2>().setOnes();
  }
  // 当前约束行索引，从第 3 行（索引 2）开始设置线性约束
  Eigen::Index cur_index = 2;
  for (const Solver::LinearConstraintParams& lin : m_constraintsParams.lin)
  {
    // 若 F 只有一个元素（所有阶段共享），取索引 0；否则取当前阶段 i
    std::size_t j (lin.F.size() == 1 ? 0 : i);
    const Matrix& _F (lin.F[j]);
    const Vector& _g (lin.g[j]);
    Eigen::Index nC (_F.rows());

    // 设置约束矩阵：A = [F*a_i, F*b_i]
    // u 的系数列 = F * a_i，x 的系数列 = F * b_i
    m_A.block(cur_index, 0, nC, 1) = _F * lin.a[i];
    m_A.block(cur_index, 1, nC, 1) = _F * lin.b[i];
    // 上界 = g - F*c_i，取与 m_boundary 的较小值
    m_hA.segment(cur_index, nC) = (_g - _F * lin.c[i]).cwiseMin(m_boundary);
    // 下界 = -m_boundary（负无穷替代值）
    m_lA.segment(cur_index, nC).setConstant(-m_boundary);
    cur_index += nC;
  }
  // 处理盒约束（u 和 x 的独立上下界）
  for (const Solver::BoxConstraintParams& box : m_constraintsParams.box)
  {
    if (!box.u.empty()) {
      // u 的边界取交集
      l[0] = std::max(l[0], box.u[i][0]);
      h[0] = std::min(h[0], box.u[i][1]);
    }
    if (!box.x.empty()) {
      // x 的边界取交集
      l[1] = std::max(l[1], box.x[i][0]);
      h[1] = std::min(h[1], box.x[i][1]);
    }
  }

  // 调试日志：输出完整的 QP 问题参数
  TOPPRA_LOG_DEBUG("qpOASES QP:\n"
      << "g: " << g.transpose() << '\n'
      << "lA: " << m_lA.transpose() << '\n'
      << "hA: " << m_hA.transpose() << '\n'
      << "l: " << l << '\n'
      << "h: " << h << '\n'
      << "A:\n" << m_A);

  qpOASES::returnValue res;
  // TODO 我假设 1000 是 SQProblem.init 函数的 nWSR（最大工作集递归次数）参数。
  //res = self.solver.init(
  //    H, g, self._A, l, h, self._lA, self._hA, np.array([1000])
  //)
  int nWSR = 1000;  // 最大工作集递归次数
  // 检查盒约束边界是否合法（上界 >= 下界）
  if ((h.array() < l.array()).any()) {
    TOPPRA_LOG_DEBUG("qpOASES: invalid box boundaries:"
        "\nlower: " << l <<
        "\nupper: " << h);
    return false;
  }
  // 检查线性约束边界是否合法（上界 >= 下界）
  if ((m_hA.array() < m_lA.array()).any()) {
    TOPPRA_LOG_DEBUG("qpOASES: invalid linear inequality bounds:"
        "\nlower: " << m_lA.transpose() <<
        "\nupper: " << m_hA.transpose());
    return false;
  }
  if (H.size() == 0) {
    // H 为空 → 求解 LP，设置 Hessian 类型为零矩阵
    m_impl->qp.setHessianType(qpOASES::HST_ZERO);
    res = m_impl->qp.init (NULL, g.data(),
        m_A.data(),
        l.data(), h.data(),
        m_lA.data(), m_hA.data(),
        nWSR);
  } else {
    // H 非空 → 求解 QP，将 H 转换为行主序存储
    m_H = H; // 转换为行主序（Row-Major）
    res = m_impl->qp.init (m_H.data(), g.data(),
        m_A.data(),
        l.data(), h.data(),
        m_lA.data(), m_hA.data(),
        nWSR);
  }

  if (res == qpOASES::SUCCESSFUL_RETURN) {
    // 求解成功，提取原始解
    solution.resize(nbVars());
    m_impl->qp.getPrimalSolution(solution.data());
    TOPPRA_LOG_DEBUG("solution: " << solution.transpose());
#ifdef TOPPRA_DEBUG_ON
    {
      // 调试模式：提取对偶解（拉格朗日乘子）
      Vector lambdas (m_impl->qp.getNV() + m_impl->qp.getNC());
      m_impl->qp.getDualSolution(lambdas.data());
      TOPPRA_LOG_DEBUG("dual solution: " << lambdas.transpose());
    }
#endif
    // 将解限制在盒约束边界范围内（防止数值误差导致越界）
    solution = solution.cwiseMax(l.transpose());  // 解 >= 下界
    solution = solution.cwiseMin(h.transpose());  // 解 <= 上界
    assert((solution.transpose().array() <= h.array()).all());
    assert((solution.transpose().array() >= l.array()).all());
    return true;
  }
  // 求解失败，输出错误信息
  TOPPRA_LOG_DEBUG("qpOASES failed. Error code: " <<
      qpOASES::MessageHandling::getErrorCodeMessage(res) << " (" << res << ')');
  return false;
#else
  // 未启用 qpOASES 编译支持
  TOPPRA_UNUSED(i);
  TOPPRA_UNUSED(H);
  TOPPRA_UNUSED(g);
  TOPPRA_UNUSED(x);
  TOPPRA_UNUSED(xNext);
  TOPPRA_UNUSED(solution);
  throw std::invalid_argument("TOPPRA was not built with qpOASES support.");
#endif
}

} // namespace solver
} // namespace toppra
