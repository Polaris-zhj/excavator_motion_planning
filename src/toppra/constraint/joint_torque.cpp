#include <toppra/constraint/joint_torque.hpp>

#include <toppra/geometric_path.hpp>

namespace toppra {
namespace constraint {

/// \brief 打印关节力矩约束信息到输出流。
/// 先输出类名，再调用基类打印方法，最后显示力矩上下限。
std::ostream& JointTorque::print (std::ostream& os) const
{
  os << "JointTorque\n";
  return LinearConstraint::print(os) <<
    "    Lower torque limit: " << m_lower.transpose() << "\n"
    "    Upper torque limit: " << m_upper.transpose() << "\n";
}

/// \brief 校验力矩限值和摩擦系数的合法性。
/// 检查：上下限尺寸一致、摩擦系数尺寸匹配（或为空）、下限不大于上限。
void JointTorque::check ()
{
  if (m_lower.size() != m_upper.size())
    throw std::invalid_argument("Torque limits size must match.");
  if (m_frictionCoeffs.size() != 0 && m_lower.size() != m_frictionCoeffs.size())
    throw std::invalid_argument("Torque limits size and friction vector size must match.");
  if ((m_lower.array() > m_upper.array()).any())
    throw std::invalid_argument("Bad torque limits.");
}

/// \brief 计算规范线性约束的系数 a, b, c, F, g。
///
/// 力矩方程 τ = A(q)·q̈ + q̇ᵀ·B(q)·q̇ + C(q) + D(q̇)，
/// 在路径 q = p(s) 上展开为：τ = a(s)·s̈ + b(s)·ṡ² + c(s)。
///
/// 通过三次逆动力学调用分离各项（利用叠加原理）：
///   1. c[i] = ID(q, 0, 0)     —— 仅重力项（速度和加速度均为零）
///   2. a[i] = ID(q, 0, v) - c[i] —— 惯性项（q̈ 方向，减去重力偏置）
///   3. b[i] = ID(q, v, a) - c[i] —— 科氏+离心项（含速度和加速度，减去重力偏置）
///
/// 若启用摩擦，c[i] 还需加上 sign(v)·frictionCoeffs（干摩擦与速度方向相关）。
///
/// F 和 g 为静态矩阵/向量，结构与加速度约束相同：
///   前 ndofs 行为单位阵（上界），后 ndofs 行为负单位阵（下界取反）。
void JointTorque::computeParams_impl(const GeometricPath& path,
        const Vector& times,
        Vectors& a, Vectors& b, Vectors& c, Matrices& F, Vectors& g,
        Bounds&, Bounds&)
{
  Eigen::Index N = times.size();
  Eigen::Index ndofs (m_lower.size());

  // ---- 构造静态 F 和 g（与加速度约束结构相同）----
  // 前 ndofs 行：单位阵，对应 τ <= upper
  F[0].topRows(ndofs).setIdentity();
  // 后 ndofs 行：负单位阵，对应 -τ <= -lower 即 τ >= lower
  F[0].bottomRows(ndofs).setZero();
  F[0].bottomRows(ndofs).diagonal().setConstant(-1);
  g[0].head(ndofs) =   m_upper;
  g[0].tail(ndofs) = - m_lower;

  // 零向量，用于逆动力学调用（分离各项时令速度或加速度为零）
  Vector zero = Vector::Zero(ndofs);
  for (std::size_t i = 0; i < N; ++i) {
    // 从路径中提取该网格点处的位形、速度、加速度
    Vector cfg = path.eval_single(times[i], 0);  // q = p(s)
    Vector vel = path.eval_single(times[i], 1);  // q̇ = p'(s)·ṡ（此处取 p'(s)）
    Vector acc = path.eval_single(times[i], 2);  // p''(s)
    assert(cfg.size() == path.configSize());
    assert(vel.size() == ndofs);
    assert(acc.size() == ndofs);

    // c[i] = ID(q, 0, 0)：仅含重力项（速度、加速度均为零）
    computeInverseDynamics(cfg, zero, zero, c[i]);
    // a[i] = ID(q, 0, vel) - c[i]：惯性项，分离出 s̈ 的系数
    //   ID(q, 0, vel) = A(q)·vel + C(q)，减去 C(q) 得 A(q)·vel = a(s)
    computeInverseDynamics(cfg, zero, vel, a[i]);
    a[i] -=  c[i];
    // b[i] = ID(q, vel, acc) - c[i]：科氏+离心项，分离出 ṡ² 的系数
    //   ID(q, vel, acc) = A(q)·acc + velᵀ·B(q)·vel + C(q)
    //   减去 C(q) 得 A(q)·acc + velᵀ·B(q)·vel = b(s)
    computeInverseDynamics(cfg, vel, acc, b[i]);
    b[i] -=  c[i];

    // 若考虑干摩擦，将 sign(vel)·frictionCoeffs 加入常数项 c[i]
    // 干摩擦力矩方向与速度方向相反，大小与摩擦系数成正比
    if (m_frictionCoeffs.size() > 0)
      c[i].array() += vel.array().sign() * m_frictionCoeffs.array();
  }
}

} // namespace constraint
} // namespace toppra
