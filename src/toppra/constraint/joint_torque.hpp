#ifndef TOPPRA_CONSTRAINT_JOINT_TORQUE_HPP
#define TOPPRA_CONSTRAINT_JOINT_TORQUE_HPP

#include <cassert>

#include <toppra/constraint.hpp>

namespace toppra {
namespace constraint {

/** \brief 关节力矩约束基类。
 *
 * 关节力矩约束的物理方程为：
 * \f[
 *   A(q) \ddot q + \dot q^\top B(q) \dot q + C(q) + D( \dot q )= \tau
 * \f]
 *
 * 其中 τ（力矩）需满足多面体约束：
 * \f[ F \tau \leq g \f]
 *
 * 注意：inverseDynamics(q, qd, qdd) = τ，且 F、g 与几何路径无关（静态）。
 *
 * 在几何路径 p(s) 上展开，可得二阶方程：
 *
 * \f{eqnarray}
 *   A(q) p'(s) \ddot s &+ [A(q) p''(s) + p'(s)^\top B(q) p'(s)] \dot s^2 &+ C(q) + D( \dot q ) &= \tau \\
 *         a(s) \ddot s &+                                  b(s) \dot s^2 &+ c(s) &= \tau
 * \f}
 *
 * 其中：
 *   - a(s) 为 s̈ 的系数向量
 *   - b(s) 为 ṡ² 的系数向量
 *   - c(s) 为常数项（含重力、科氏力、摩擦等）
 *
 * 子类必须实现 computeInverseDynamics 以提供逆动力学计算。
 * */
class JointTorque : public LinearConstraint {
  public:
    /// \brief 虚析构函数，确保安全的多态析构。
    virtual ~JointTorque () {}

    /// \brief 将约束信息输出到流，用于调试与日志。
    virtual std::ostream& print(std::ostream& os) const;

    /** \brief 计算关节力矩（逆动力学）。
     * \param q 机器人位形（关节角）。
     * \param v 机器人速度（关节角速度）。
     * \param a 机器人加速度（关节角加速度）。
     * \param[out] tau 输出：计算得到的关节力矩。
     *
     * 子类需提供具体的逆动力学实现（如基于 Pinocchio 的 RNEA）。
     * */
    virtual void computeInverseDynamics (const Vector& q, const Vector& v, const Vector& a,
        Vector& tau) = 0;

    /// \brief 返回力矩下限向量。
    const Vector& lowerBounds () const
    { return m_lower; }

    /// \brief 设置力矩下限向量（尺寸需匹配）。
    void lowerBounds (const Vector& lb)
    {
      assert(lb.size() == m_lower.size());
      m_lower = lb;
    }

    /// \brief 返回力矩上限向量。
    const Vector& upperBounds () const
    { return m_upper; }

    /// \brief 设置力矩上限向量（尺寸需匹配）。
    void upperBounds (const Vector& ub)
    {
      assert(ub.size() == m_upper.size());
      m_upper = ub;
    }

    /// \brief 返回干摩擦系数向量。
    const Vector& frictionCoeffs () const
    { return m_frictionCoeffs; }

    /// \brief 设置干摩擦系数向量（尺寸需匹配）。
    void frictionCoeffs (const Vector& fc)
    {
      assert(fc.size() == m_frictionCoeffs.size());
      m_frictionCoeffs = fc;
    }

  protected:
    /** \brief 受保护构造函数。
     * \param lowerTlimit 力矩下限向量。
     * \param upperTlimit 力矩上限向量。
     * \param frictionCoeffs 各关节干摩擦系数，尺寸为 0 表示不考虑摩擦。
     *
     * 传入 LinearConstraint 基类的参数含义：
     *   - k = ndofs * 2：不等式约束数量（上下界各 ndofs 条）。
     *   - m = ndofs：内部变量维度（力矩维度）。
     *   - constantF = true：F、g 静态不随路径变化。
     *   - uBound = false, xBound = false：不直接约束 u 和 x。
     * */
    JointTorque (const Vector& lowerTlimit, const Vector& upperTlimit,
        const Vector& frictionCoeffs)
      : LinearConstraint (2*lowerTlimit.size(), lowerTlimit.size(), true, false, false)
      , m_lower (lowerTlimit)
      , m_upper (upperTlimit)
      , m_frictionCoeffs (frictionCoeffs)
    {
      check();
    }

    /// \brief 移动构造函数，转移所有权。
    JointTorque (JointTorque&& other)
      : LinearConstraint (other)
      , m_lower(std::move(other.m_lower))
      , m_upper(std::move(other.m_upper))
      , m_frictionCoeffs(std::move(other.m_frictionCoeffs))
    {}

  private:
    /// \brief 校验力矩限值和摩擦系数的合法性。
    void check();

    /// \brief 计算规范线性约束的系数 a, b, c, F, g。
    ///
    /// 利用逆动力学将力矩约束分解为路径上的二阶方程系数。
    /// 通过三次调用 computeInverseDynamics 分离各项：
    ///   - c[i] = ID(q, 0, 0)：仅含重力的常数项
    ///   - a[i] = ID(q, 0, v) - c[i]：s̈ 的系数（惯性项）
    ///   - b[i] = ID(q, v, a) - c[i]：ṡ² 的系数（科氏+离心项）
    /// 若考虑摩擦，c[i] 还需加上 sign(v)·frictionCoeffs。
    void computeParams_impl(const GeometricPath& path,
        const Vector& gridpoints,
        Vectors& a, Vectors& b, Vectors& c,
        Matrices& F, Vectors& g,
        Bounds& ubound, Bounds& xbound);

    Vector m_lower;          ///< 各关节力矩下限。
    Vector m_upper;          ///< 各关节力矩上限。
    Vector m_frictionCoeffs; ///< 各关节干摩擦系数（空向量表示无摩擦）。
}; // class JointTorque
} // namespace constraint
} // namespace toppra

#endif
