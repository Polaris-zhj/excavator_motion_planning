#include <toppra/constraint/linear_joint_velocity.hpp>

#include "toppra/src/geometric_path.hpp"

namespace toppra {
namespace constraint {

/// \brief 打印关节速度约束信息到输出流。
/// 先输出类名，再调用基类打印方法，最后显示上下限向量。
std::ostream& LinearJointVelocity::print(std::ostream& os) const {
  os << "LinearJointVelocity\n";
  return LinearConstraint::print(os)
         << "    Lower velocity limit: " << m_lower.transpose()
         << "\n"
            "    Upper velocity limit: "
         << m_upper.transpose() << "\n";
}

/// \brief 校验速度上下限。
/// 检查上下限向量尺寸一致，且每个自由度的下限不大于上限。
void LinearJointVelocity::check() {
  if (m_lower.size() != m_upper.size())
    throw std::invalid_argument("Velocity limits size must match.");
  if ((m_lower.array() > m_upper.array()).any())
    throw std::invalid_argument("Bad velocity limits.");
}

/// \brief 计算规范线性约束的系数，核心是计算 xbound（ṡ² 的边界）。
///
/// 关节速度约束 q̇ = p'(s)·ṡ 需满足 lower <= q̇ <= upper，
/// 即 lower <= v·ṡ <= upper（v = p'(s) 为路径一阶导）。
///
/// 对每个关节 k，根据 v[k] 的正负号将不等式两边除以 v[k]：
///   - v[k] > 0：lower[k]/v[k] <= ṡ <= upper[k]/v[k]
///   - v[k] < 0：upper[k]/v[k] <= ṡ <= lower[k]/v[k]（不等式方向翻转）
///   - v[k] == 0：若 0 不在 [lower, upper] 内则不可行
///
/// 所有关节约束的交集给出 ṡ 的可行区间 [sdmin, sdmax]，
/// 然后 xbound[i] = [sdmin², sdmax²]（x = ṡ²）。
///
/// \param path 几何路径对象。
/// \param gridpoints 网格点向量（路径参数 s 的采样点）。
/// \param xbound 输出：每个网格点处 ṡ² 的 [下界, 上界]。
void LinearJointVelocity::computeParams_impl(const GeometricPath& path,
                                             const Vector& gridpoints, Vectors&,
                                             Vectors&, Vectors&, Matrices&,
                                             Vectors&, Bounds&,
                                             Bounds& xbound) {
  // 网格点数量
  Eigen::Index N_1 = gridpoints.size();
  // 自由度数量
  Eigen::Index ndofs(m_lower.size());
  assert(path.dof() == m_lower.size());

  Vector v, v_inv(ndofs), lb_v(ndofs);
  for (std::size_t i = 0; i < N_1; ++i) {
    // 更新该网格点处的速度限值（常数限值时为空操作）
    computeVelocityLimits(gridpoints[i]);

    // v = p'(s_i)：路径在 s_i 处对 s 的一阶导
    v = path.eval_single(gridpoints[i], 1);
    assert(ndofs == v.size());

    // v_inv = 1/v，逐元素求倒数
    v_inv.noalias() = v.cwiseInverse();

    // 初始化 ṡ 的可行区间为 [-m_maxsd, m_maxsd]
    value_type sdmin = -m_maxsd, sdmax = m_maxsd;
    for (Eigen::Index k = 0; k < ndofs; ++k) {
      if (v[k] > 0) {
        // q̇ = v[k]·ṡ 为正比关系：ṡ 上界由 upper 限制，下界由 lower 限制
        sdmax = std::min(m_upper[k] * v_inv[k], sdmax);
        sdmin = std::max(m_lower[k] * v_inv[k], sdmin);
      } else if (v[k] < 0) {
        // q̇ = v[k]·ṡ 为反比关系：不等式方向翻转
        sdmax = std::min(m_lower[k] * v_inv[k], sdmax);
        sdmin = std::max(m_upper[k] * v_inv[k], sdmin);
      } else {
        // v[k] == 0：q̇ 恒为零，若 0 不在 [lower, upper] 内则不可行
        if (m_upper[k] < 0 || m_lower[k] > 0)
          /// \todo 问题不可行，应如何通知用户？
          throw std::runtime_error("BoxConstraint is infeasible");
      }
    }
    // xbound 下界：若 sdmin > 0 则 x 不能为 0（ṡ² >= sdmin²）
    xbound[i][0] = 0;
    if (sdmin > 0) xbound[i][0] = sdmin * sdmin;
    // xbound 上界：x = ṡ² <= sdmax²
    xbound[i][1] = sdmax * sdmax;
  }
}

}  // namespace constraint
}  // namespace toppra
