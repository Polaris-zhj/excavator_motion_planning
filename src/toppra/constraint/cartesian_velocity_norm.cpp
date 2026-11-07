#include <toppra/constraint/cartesian_velocity_norm.hpp>

#include "toppra/src/geometric_path.hpp"

namespace toppra {
namespace constraint {

/// \brief 打印笛卡尔速度范数约束信息到输出流。
/// 先输出类名，再调用基类打印方法，最后显示速度限值。
std::ostream& CartesianVelocityNorm::print(std::ostream& os) const {
  os << "CartesianVelocityNorm\n";
  return LinearConstraint::print(os)
         << "    Velocity norm limit: " << m_limit << "\n";
}

/// \brief 校验速度限值和选择矩阵。
/// 限值须非负（范数不可能为负），S 矩阵须为 6×6（6D 速度向量）。
void CartesianVelocityNorm::check() {
  if (m_limit < 0)
    throw std::invalid_argument("Velocity limit should be positive.");
  if (m_S.rows() != 6 || m_S.cols() != 6)
    throw std::invalid_argument("S matrix should be of size 6x6.");
}

/// \brief 计算规范线性约束的系数 a, b, c, F, g。
///
/// 约束形式：vᵀ·S·v·ṡ² <= limit，其中 v = J(q)·p'(s) 为坐标系 6D 速度。
/// 因此：
///   - a[i] = 0（无 s̈ 项）
///   - b[i] = vᵀ·S·v（标量，作为 ṡ² 的系数）
///   - c[i] = 0（无常数项）
///   - F 为 1×1 单位阵，g 为限值标量
///
/// 若限值为常数（constantF = true），F 和 g 仅计算一次；
/// 否则逐网格点计算（支持变限值场景）。
///
/// \param path 几何路径对象。
/// \param gridpoints 网格点向量。
/// \param a 输出：零向量（无 s̈ 项）。
/// \param b 输出：vᵀ·S·v 标量，ṡ² 的系数。
/// \param c 输出：零向量（无常数项）。
/// \param F 输出：1×1 单位阵。
/// \param g 输出：速度限值标量。
void CartesianVelocityNorm::computeParams_impl(const GeometricPath& path,
                                               const Vector& gridpoints,
                                               Vectors& a, Vectors& b,
                                               Vectors& c, Matrices& F,
                                               Vectors& g, Bounds&, Bounds&) {
  Eigen::Index N_1 = gridpoints.size();

  // 若 F 为静态（常数限值），只需设置一次
  if (constantF()) {
    F[0].setIdentity();  // 1×1 单位阵
    g[0][0] = m_limit;   // 限值标量
  }

  // 预分配工作向量
  Vector q(path.configSize()), qdot(path.dof()), v(6), S_v(6);
  for (std::size_t i = 0; i < N_1; ++i) {
    // 更新该网格点处的速度限值（常数限值时为空操作）
    computeVelocityLimit(gridpoints[i]);

    // 从路径提取位形 q 和路径一阶导 p'(s)
    q = path.eval_single(gridpoints[i], 0);
    qdot = path.eval_single(gridpoints[i], 1);
    // 计算坐标系 6D 速度 v = J(q)·qdot
    computeVelocity(q, qdot, v);
    // S_v = S·v
    S_v = m_S * v;

    // a[i] = 0：该约束不依赖 s̈
    a[i].setZero();
    // b[i] = vᵀ·(S·v) = vᵀ·S·v：ṡ² 的系数（标量）
    b[i].noalias() = v.transpose() * S_v;
    // c[i] = 0：无常数项
    c[i].setZero();

    // 若 F 为非静态（变限值），逐网格点设置
    if (!constantF()) {
      F[i].setIdentity();
      g[i][0] = m_limit;
    }
  }
}

}  // namespace constraint
}  // namespace toppra
