#ifndef TOPPRA_CONSTRAINT_LINEAR_JOINT_ACCELERATION_HPP
#define TOPPRA_CONSTRAINT_LINEAR_JOINT_ACCELERATION_HPP

#include <toppra/constraint.hpp>

namespace toppra {
namespace constraint {

/// \brief 关节加速度约束类（线性形式）。
///
/// 该约束限制机器人在几何路径上各关节的加速度，使其满足：
///   lowerAlimit <= q̈ <= upperAlimit
///
/// 在 TOPPRA 的规范线性约束框架中，路径参数 s 满足 q(s)，
/// 关节加速度可写作 q̈ = p'(s) s̈ + p''(s) ṡ²。因此该约束化为：
///   a(s) s̈ + b(s) ṡ² + c(s) <= g （通过 F 矩阵表达上下界）
///
/// 由于上下界为常数，矩阵 F 和向量 g 与路径参数 s 无关，
/// 故 constantF 设为 true。
class LinearJointAcceleration : public LinearConstraint {
  public:
    /// \brief 构造函数。
    /// \param lowerAlimit 各关节加速度下限向量。
    /// \param upperAlimit 各关节加速度上限向量。
    ///
    /// 传入 LinearConstraint 基类的参数含义：
    ///   - k = ndofs * 2：不等式约束数量（上下界各 ndofs 条）。
    ///   - m = ndofs：内部变量 v 的维度（即关节加速度维度）。
    ///   - constantF = true：F、g 不随路径点变化。
    ///   - uBound = false：不对 u（s̈）直接设置边界。
    ///   - xBound = false：不对 x（ṡ²）直接设置边界。
    LinearJointAcceleration (const Vector& lowerAlimit, const Vector& upperAlimit)
      : LinearConstraint (lowerAlimit.size() * 2, lowerAlimit.size(), true, false, false)
      , m_lower (lowerAlimit)
      , m_upper (upperAlimit)
    {
      check();
    }

    /// \brief 将约束信息输出到流，用于调试与日志。
    virtual std::ostream& print(std::ostream& os) const;

  private:
    /// \brief 校验加速度上下限的合法性。
    /// 若任意自由度的下限大于上限则抛出异常。
    void check();

    /// \brief 计算规范线性约束的数值系数。
    ///
    /// 对路径上的每个网格点 i，计算：
    ///   - a[i] = p'(s_i)：路径一阶导，作为 s̈ 的系数。
    ///   - b[i] = p''(s_i)：路径二阶导，作为 ṡ² 的系数。
    ///   - c[i] = 0：常数项为零。
    ///   - F[0]：静态矩阵，前 ndofs 行为单位阵（对应上界），
    ///           后 ndofs 行为负单位阵（对应下界取反）。
    ///   - g[0]：静态向量，前 ndofs 分量为上限，后 ndofs 分量为下限取负。
    ///
    /// 这样约束 F·v <= g 等价于：
    ///   q̈ <= upper  且  -q̈ <= -lower  即  q̈ >= lower
    void computeParams_impl(const GeometricPath& path,
        const Vector& gridpoints,
        Vectors& a, Vectors& b, Vectors& c,
        Matrices& F, Vectors& g,
        Bounds& ubound, Bounds& xbound);

    Vector m_lower; ///< 各关节加速度下限。
    Vector m_upper; ///< 各关节加速度上限。
}; // class LinearJointAcceleration
} // namespace constraint
} // namespace toppra

#endif
