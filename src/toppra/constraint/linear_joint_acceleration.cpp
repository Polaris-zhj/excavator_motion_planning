#include <cassert>
#include <toppra/constraint/linear_joint_acceleration.hpp>

#include "toppra/src/geometric_path.hpp"

namespace toppra {
namespace constraint {

/// \brief 打印关节加速度约束信息到输出流。
/// 先输出类名，再调用基类打印方法，最后显示上下限向量。
std::ostream& LinearJointAcceleration::print(std::ostream& os) const {
  os << "LinearJointAcceleration\n";
  return LinearConstraint::print(os)
         << "    Lower acceleration limit: " << m_lower.transpose()
         << "\n"
            "    Upper acceleration limit: "
         << m_upper.transpose() << "\n";
}

/// \brief 校验加速度上下限。
/// 若任意自由度的下限大于上限，则约束不可行，抛出异常。
void LinearJointAcceleration::check() {
  if ((m_lower.array() > m_upper.array()).any())
    throw std::invalid_argument("Bad acceleration limits.");
}

/// \brief 计算规范线性约束的系数 a, b, c, F, g。
///
/// \param path 几何路径对象。
/// \param times 网格点向量（路径参数 s 的采样点）。
/// \param a 输出：各网格点处路径一阶导 p'(s)，作为 s̈ 的系数。
/// \param b 输出：各网格点处路径二阶导 p''(s)，作为 ṡ² 的系数。
/// \param c 输出：常数项，本约束中始终为零向量。
/// \param F 输出：不等式约束矩阵（静态，仅一份）。
/// \param g 输出：不等式约束右侧向量（静态，仅一份）。
///
/// 约束的形式为 F·v <= g，其中 v = q̈ = a·s̈ + b·ṡ² + c。
/// F 的构造使得：
///   - 前 ndofs 行取 v 的正分量，对应 q̈ <= upper。
///   - 后 ndofs 行取 v 的负分量，对应 -q̈ <= -lower 即 q̈ >= lower。
void LinearJointAcceleration::computeParams_impl(const GeometricPath& path,
                                                 const Vector& times,
                                                 Vectors& a, Vectors& b,
                                                 Vectors& c, Matrices& F,
                                                 Vectors& g, Bounds&, Bounds&) {
  // 网格点数量
  Eigen::Index N_1 = times.size();

  // 自由度数量（即关节加速度的维度）
  Eigen::Index ndofs(nbVariables());

  // ---- 构造静态 F 和 g（不随网格点变化）----
  Matrix& _F = F[0];
  // 前 ndofs 行设置为单位矩阵：提取 v 的正分量用于上界约束
  _F.topRows(ndofs).setIdentity();
  // 后 ndofs 行先清零
  _F.bottomRows(ndofs).setZero();
  // 后 ndofs 行的对角线设为 -1：提取 v 的负分量用于下界约束
  _F.bottomRows(ndofs).diagonal().setConstant(-1);
  Vector& _g = g[0];
  // 上界：q̈ <= upper
  _g.head(ndofs) = m_upper;
  // 下界取负：-q̈ <= -lower 等价于 q̈ >= lower
  _g.tail(ndofs) = -m_lower;

  // ---- 逐网格点计算 a, b, c ----
  assert(ndofs == path.dof());
  for (std::size_t i = 0; i < N_1; ++i) {
    // a[i] = p'(s_i)：路径对 s 的一阶导，即 s̈ 的系数
    a[i] = path.eval_single(times[i], 1);
    assert(a[i].size() == ndofs);
    // b[i] = p''(s_i)：路径对 s 的二阶导，即 ṡ² 的系数
    b[i] = path.eval_single(times[i], 2);
    assert(b[i].size() == ndofs);
    // 常数项为零（加速度约束没有偏置项）
    c[i].setZero();
  }
}

}  // namespace constraint
}  // namespace toppra
