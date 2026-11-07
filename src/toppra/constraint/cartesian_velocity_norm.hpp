#ifndef TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_HPP
#define TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_HPP

#include "toppra/src/constraint.hpp"

namespace toppra {
namespace constraint {

/**
\brief 笛卡尔速度范数约束类。

给定路径 p(s)，该类约束某一坐标系的线速度/角速度向量 v 的加权范数：
\f$||v||_S^2 = v^T S v \leq limit \f$。

由于 \f$ v = J(q) p'(s) \dot{s} \f$（雅可比乘以路径导数再乘路径速度），
该约束可纳入规范线性约束框架：

\f{eqnarray}
    p'(s)^T J(q)^T S J(q) p'(s) \dot{s}^2 &= w \\
    w & \leq \mathbf g_i
\f}

即 b(s)·ṡ² <= limit，其中 b(s) = vᵀ·S·v。

本类实现常数速度限值，也可被继承以实现随路径变化的速度限值：
\code
class CartesianVelocityVarying : public CartesianVelocityNorm {
public:
  CartesianVelocityVarying(...) : CartesianVelocityNorm () {}
protected:
  void computeVelocityLimit(value_type time)
  {
    // 若某一项为常数，可在构造函数中设置
    m_limit = ...;
    m_S = ...;
  }
};
\endcode
*/
class CartesianVelocityNorm : public LinearConstraint {
 public:
  /// \brief 将约束信息输出到流，用于调试与日志。
  virtual std::ostream& print(std::ostream& os) const;

 protected:
  /// \brief 构造函数（常数速度限值）。
  /// \param S 6×6 选择矩阵，用于加权线速度和角速度的各分量。
  /// \param limit 速度范数上限（标量）。
  ///
  /// 由于限值为常数，F、g 静态不变，constantF = true。
  CartesianVelocityNorm(const Matrix& S, const double& limit)
      : LinearConstraint(1, 1, true, false, false), m_limit(limit), m_S(S) {
    check();
  }

  /// \brief 构造函数（变速度限值）。
  /// \note 属性 \ref m_S 和 \ref m_limit **必须**在
  /// \ref computeVelocityLimit 中被计算赋值。
  ///
  /// 由于限值随路径变化，constantF = false，F、g 逐网格点不同。
  CartesianVelocityNorm()
      : LinearConstraint(1, 1, false, false, false), m_limit(1.), m_S(6, 6) {
    check();
  }

  /// \brief 纯虚函数：计算坐标系的 6D 速度向量。
  /// \param q 当前位形（关节角）。
  /// \param qdot 当前速度（关节角速度）。
  /// \param[out] v 输出：6 维坐标系速度（3 线速度 + 3 角速度）。
  ///
  /// 子类需提供具体的前向运动学速度计算实现。
  virtual void computeVelocity(const Vector& q, const Vector& qdot,
                               Vector& v) = 0;

  /**
    \brief 计算给定路径参数 \c time 处的速度限值。

    结果存入属性 CartesianVelocityNorm::m_limit。
    默认实现为空（使用构造时设定的常数限值）。
    */
  virtual void computeVelocityLimit(value_type time) { (void)time; }

  /// 速度范数上限（标量）
  value_type m_limit;
  /// 6×6 选择/加权矩阵，用于选择线速度或角速度分量并加权
  Matrix m_S;

 private:
  /// \brief 校验限值和选择矩阵的合法性。
  /// 限值须非负，S 矩阵须为 6×6。
  void check();

  /// \brief 计算规范线性约束的系数。
  ///
  /// 对每个网格点 i：
  ///   1. 调用 computeVelocityLimit 更新限值（常数时无操作）。
  ///   2. 从路径提取 q 和 q̇（p'(s)）。
  ///   3. 调用 computeVelocity 计算 6D 速度 v。
  ///   4. 计算 S·v，再得 b[i] = vᵀ·S·v（标量），作为 ṡ² 的系数。
  ///   5. a[i] = 0, c[i] = 0（该约束仅含 ṡ² 项）。
  ///   6. F 为 1×1 单位阵，g 为限值标量。
  void computeParams_impl(const GeometricPath& path, const Vector& gridpoints,
                          Vectors& a, Vectors& b, Vectors& c, Matrices& F,
                          Vectors& g, Bounds& ubound, Bounds& xbound);
};  // class CartesianVelocityNorm
}  // namespace constraint
}  // namespace toppra

#endif
