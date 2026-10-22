#ifndef TOPPRA_CONSTRAINT_LINEAR_JOINT_VELOCITY_HPP
#define TOPPRA_CONSTRAINT_LINEAR_JOINT_VELOCITY_HPP

#include <cassert>

#include <toppra/constraint.hpp>

namespace toppra {
namespace constraint {

/**
\brief 关节速度约束类（线性形式）。

该类实现常数关节速度上下限约束：lowerVlimit <= q̇ <= upperVlimit。
也可作为基类被继承，以实现随路径参数变化的速度限值。例如：
\code
class LinearJointVelocityVarying : public LinearJointVelocity {
public:
  LinearJointVelocityVarying(...) : LinearJointVelocity (nDof) {}
protected:
  void computeVelocityLimits(value_type time)
  {
    m_lower = ...;
    m_upper = ...;
  }
};
\endcode

在 TOPPRA 框架中，关节速度 q̇ = p'(s)·ṡ，因此速度约束
转化为对 ṡ 的直接上下界（即 x = ṡ² 的边界 xbound）。
该约束不产生不等式约束（k=0, m=0），仅通过 xbound 生效。
*/
class LinearJointVelocity : public LinearConstraint {
  public:
    /// \brief 构造函数（常数速度限值）。
    /// \param lowerVlimit 各关节速度下限向量。
    /// \param upperVlimit 各关节速度上限向量。
    ///
    /// 传入 LinearConstraint 基类的参数含义：
    ///   - k = 0：无不等式约束。
    ///   - m = 0：无内部变量 v。
    ///   - constantF = true：F、g 为空但视为静态。
    ///   - uBound = false：不对 u（s̈）设置边界。
    ///   - xBound = true：对 x（ṡ²）设置边界。
    LinearJointVelocity (const Vector& lowerVlimit, const Vector& upperVlimit)
      : LinearConstraint (0, 0, true, false, true)
      , m_lower (lowerVlimit)
      , m_upper (upperVlimit)
      , m_maxsd (1e8)
    {
      check();
    }

    /** \brief 设置 ṡ 的最大允许值。
     * \param maxsd 应为严格正数。
     *
     * 用于限制路径参数速度的绝对上界，避免数值发散。
     * */
    void maxSDot (value_type maxsd)
    {
      assert(maxsd > 0);
      m_maxsd = maxsd;
    }

    /// \brief 将约束信息输出到流，用于调试与日志。
    virtual std::ostream& print(std::ostream& os) const;

  protected:
    /// \brief 受保护构造函数，供子类使用（变速度限值场景）。
    /// \param nDof 自由度数量，用于预分配 m_lower 和 m_upper。
    ///
    /// 子类需在 computeVelocityLimits 中填充实际的速度限值。
    LinearJointVelocity (const int nDof)
      : LinearConstraint (0, 0, true, false, true)
      , m_lower (nDof)
      , m_upper (nDof)
      , m_maxsd (1e8)
    {
      check();
    }

    /**
      \brief 计算给定路径参数 \c time 处的速度限值。

      子类应覆写此方法，将结果存入
      LinearJointVelocity::m_lower 和 LinearJointVelocity::m_upper。
      默认实现为空（即使用构造时已设定的常数限值）。
      */
    virtual void computeVelocityLimits(value_type time) { (void)time; }

    /// 各关节速度下限（可被子类在 computeVelocityLimits 中更新）
    Vector m_lower;
    /// 各关节速度上限（可被子类在 computeVelocityLimits 中更新）
    Vector m_upper;

  private:
    /// \brief 校验速度上下限的合法性。
    /// 检查上下限向量尺寸一致且下限不大于上限。
    void check();

    /// \brief 计算规范线性约束的系数。
    ///
    /// 对于关节速度约束，核心是将 q̇ = p'(s)·ṡ 的上下限
    /// 转换为 ṡ 的上下界，进而得到 x = ṡ² 的边界 xbound。
    ///
    /// 对每个网格点 i：
    ///   1. 调用 computeVelocityLimits 更新限值（常数限值时无操作）。
    ///   2. 计算 v = p'(s_i)（路径一阶导）。
    ///   3. 对每个关节 k，根据 v[k] 的符号计算 ṡ 的可行区间 [sdmin, sdmax]。
    ///   4. 将 ṡ 的边界平方后存入 xbound[i]。
    void computeParams_impl(const GeometricPath& path,
        const Vector& gridpoints,
        Vectors& a, Vectors& b, Vectors& c,
        Matrices& F, Vectors& g,
        Bounds& ubound, Bounds& xbound);

    /// ṡ 的最大允许绝对值（默认 1e8），用于防止数值发散
    value_type m_maxsd;
}; // class LinearJointVelocity
} // namespace constraint
} // namespace toppra

#endif
