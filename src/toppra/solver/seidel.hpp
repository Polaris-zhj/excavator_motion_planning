#ifndef TOPPRA_SOLVER_SEIDEL_HPP
#define TOPPRA_SOLVER_SEIDEL_HPP

#include <memory>
#include <array>
#include <toppra/solver.hpp>
#include <Eigen/Core>

namespace toppra {
namespace solver {

/** Seidel 线性规划算法的实现。
 *
 *  Seidel 算法是一种递归式 LP 求解算法，特别适合低维（2 变量）问题。
 *  其核心思想是通过逐步加入约束，将 2D LP 递归地转化为 1D LP 子问题来求解。
 *  对于 TOPP-RA 中每阶段 2 个变量（u, x）的 LP 问题，该算法具有很高的效率。
 *
 *  \note 该求解器仅支持线性规划（LP），不支持二次规划（QP）。
 * */
class Seidel : public Solver {
  public:
    // 默认构造函数
    Seidel () = default;

    /// 初始化求解器，设置约束参数、几何路径和时间序列
    /// \param constraints 规范线性约束列表
    /// \param path 几何路径对象
    /// \param times 各阶段对应的时间点序列
    void initialize (const LinearConstraintPtrs& constraints, const GeometricPathPtr& path,
        const Vector& times);

    /// 求解第 i 阶段的线性规划问题
    /// \param i 当前阶段索引
    /// \param H 二次项矩阵（Seidel 不支持，必须为空）
    /// \param g 线性目标项的系数向量
    /// \param x 当前阶段位置变量的上下界
    /// \param xNext 下一阶段位置变量的上下界
    /// \param solution 输出参数，存储求得的解向量
    /// \return 求解成功返回 true，失败返回 false
    bool solveStagewiseOptim(std::size_t i,
        const Matrix& H, const Vector& g,
        const Bound& x, const Bound& xNext,
        Vector& solution);

  private:
    // 2 列动态矩阵类型（用于 1D LP 的约束矩阵）
    typedef Eigen::Matrix<value_type, Eigen::Dynamic, 2> MatrixX2;
    // 3 列动态矩阵类型（用于 2D LP 的约束矩阵 [a, b, c]）
    typedef Eigen::Matrix<value_type, Eigen::Dynamic, 3> MatrixX3;
    // 对齐分配器管理的 MatrixX3 向量容器
    typedef std::vector<MatrixX3, Eigen::aligned_allocator<MatrixX3> > MatricesX3;

    MatricesX3 m_A;     // 各阶段的 2D LP 约束矩阵：A[i] * [x, 1]^T <= 0
    MatrixX2 m_low, m_high;  // 各阶段变量的下界和上界

    MatrixX2 m_A_1d;     // 1D LP 子问题的约束矩阵（预分配以避免重复初始化开销）
}; // class Seidel

} // namespace solver
} // namespace toppra

#endif
