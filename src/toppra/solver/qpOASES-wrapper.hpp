#ifndef TOPPRA_SOLVER_QPOASES_WRAPPER_HPP
#define TOPPRA_SOLVER_QPOASES_WRAPPER_HPP

#include <memory.h>
#include <toppra/solver.hpp>

namespace toppra {
namespace solver {

/** qpOASES::SQProblem 的封装类。
 *
 *  该类将 TOPP-RA 中的分段优化问题转化为 qpOASES 可求解的
 *  二次规划（QP）/线性规划（LP）问题。
 *
 *  内部的问题形式化表示如下：
 *  \f{eqnarray}
 *  min   & 0.5 y^T H y + g^T y \\       // 目标函数：二次项 + 线性项
 *  s.t   & lA <= Ay <= hA      \\        // 线性不等式约束
 *        & l  <=  y <= h       \\        // 变量盒约束
 *  \f}
 *
 *  \todo 添加一个继承自 qpOASESWrapper 的求解器，利用 qpOASES 的
 *        热启动（warm start）能力来加速连续阶段的求解
 *
 * */
class qpOASESWrapper : public Solver {
  public:
    // 默认构造函数
    qpOASESWrapper ();

    /// 初始化求解器，设置约束参数、几何路径和时间序列
    /// \param constraints 规范线性约束列表
    /// \param path 几何路径对象
    /// \param times 各阶段对应的时间点序列
    void initialize (const LinearConstraintPtrs& constraints, const GeometricPathPtr& path,
        const Vector& times);

    /// 求解第 i 阶段的优化问题
    /// \param i 当前阶段索引
    /// \param H 二次目标项的 Hessian 矩阵（为空时求解 LP）
    /// \param g 线性目标项的系数向量
    /// \param x 当前阶段位置变量的上下界
    /// \param xNext 下一阶段位置变量的上下界
    /// \param solution 输出参数，存储求得的解向量
    /// \return 求解成功返回 true，失败返回 false
    bool solveStagewiseOptim(std::size_t i,
        const Matrix& H, const Vector& g,
        const Bound& x, const Bound& xNext,
        Vector& solution);

    // 虚析构函数
    virtual ~qpOASESWrapper();

    /// 获取当前的边界值
    /// \return 当前用于替代无穷大的边界值
    value_type setBoundary () const
    {
      return m_boundary;
    }

    /// 设置边界值
    /// \param v 新的边界值，用于替代无穷大
    void setBoundary (const value_type& v)
    {
      m_boundary = v;
    }

    /// 设置默认边界值（静态方法，影响所有实例）
    /// \param v 默认边界值
    static void setDefaultBoundary (const value_type& v);

  private:
    /// qpOASES 使用行主序（Row-Major）存储矩阵，
    /// 因此使用 Eigen 的 RowMajor 选项来匹配 qpOASES 的内存布局
    typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic,
            Eigen::RowMajor> RMatrix;
    RMatrix m_H, m_A;       // m_H: Hessian 矩阵，m_A: 约束系数矩阵
    Vector m_lA, m_hA;      // 线性约束的下界和上界
    value_type m_boundary;   // 用于替代无穷大的数值边界

    static value_type m_defaultBoundary;  // 默认边界值（静态）

    struct Impl;  // qpOASES 求解器实现的前向声明（Pimpl 惯用法）
    std::unique_ptr<Impl> m_impl;  // qpOASES 求解器实例
}; // class qpOASESWrapper

} // namespace solver
} // namespace toppra

#endif
