#ifndef TOPPRA_SOLVER_GLPK_WRAPPER_HPP
#define TOPPRA_SOLVER_GLPK_WRAPPER_HPP

#include <memory.h>
#include <toppra/solver.hpp>

// 前向声明 glpk 求解器结构体，避免在头文件中引入 GLPK 头文件依赖
struct glp_prob;

namespace toppra {
namespace solver {

/** GLPK（GNU Linear Programming Kit）库的封装类。
 *
 *  该类将 TOPP-RA 中的分段优化问题转化为 GLPK 可求解的线性规划（LP）问题。
 *
 *  内部的问题形式化表示如下：
 *  \f{eqnarray}
 *  min   & g^T y       \\              // 目标函数：最小化 g^T y
 *  s.t   & z = A y     \\              // 约束：z = A y（等式/不等式约束的统一表示）
 *        & x_{min} <= x <= x_{max} \\   // 位置变量 x 的上下界约束
 *        & l_1 <= z <= h_1 \\           // 中间变量 z 的上下界约束
 *  \f}
 *  其中：
 *  \f{eqnarray}
 *  y =& \begin{pmatrix} u & x \end{pmatrix}^T     \\  // 决策向量 y = (u, x)^T
 *  // u = s_dot^2（路径参数的平方速度），x = s（路径位置参数）
 *  A =& \begin{pmatrix}
 *     2\delta &       1 \\      // 第一行：离散动力学约束 x_next = x + 2*delta*u
 *     F_i a_i & F_i b_i \\      // 后续行：规范线性约束 F_i*(a_i*u + b_i*x + c_i) <= g_i
 *     \vdots  & \vdots \\
 *     \end{pmatrix}\\
 *  \f}
 *
 *  \note GLPK 仅支持线性规划（LP），不支持二次规划（QP）。
 * */
class GLPKWrapper : public Solver {
  public:
    // 默认构造函数
    GLPKWrapper () = default;

    /// 初始化求解器，设置约束参数、几何路径和时间序列
    /// \param constraints 规范线性约束的指针列表
    /// \param path 几何路径对象的共享指针
    /// \param times 各阶段对应的时间点序列
    void initialize (const LinearConstraintPtrs& constraints, const GeometricPathPtr& path,
        const Vector& times);

    /// 求解第 i 阶段的优化问题
    /// \param i 当前阶段索引
    /// \param H 二次目标项的 Hessian 矩阵（GLPK 不支持，必须为空）
    /// \param g 线性目标项的系数向量
    /// \param x 当前阶段位置变量的上下界 [x_min, x_max]
    /// \param xNext 下一阶段位置变量的上下界 [x_next_min, x_next_max]
    /// \param solution 输出参数，存储求得的解向量
    /// \return 求解成功返回 true，失败返回 false
    bool solveStagewiseOptim(std::size_t i,
        const Matrix& H, const Vector& g,
        const Bound& x, const Bound& xNext,
        Vector& solution);

    // 虚析构函数，释放 GLPK 问题对象
    virtual ~GLPKWrapper();

  private:
    // GLPK 线性规划问题对象的指针，初始化为 NULL
    glp_prob* m_lp = NULL;
}; // class GLPKWrapper

} // namespace solver
} // namespace toppra

#endif
