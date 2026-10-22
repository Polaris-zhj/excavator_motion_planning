#ifndef TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_PINOCCIO_HPP
#define TOPPRA_CONSTRAINT_CARTESIAN_VELOCITY_NORM_PINOCCIO_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <toppra/constraint/cartesian_velocity_norm.hpp>

namespace toppra {
namespace constraint {
namespace cartesianVelocityNorm {

/** \brief 基于 Pinocchio 的笛卡尔速度范数约束实现。
 *
 * 使用 pinocchio::forwardKinematics 计算前向运动学，
 * 再通过 pinocchio::getFrameVelocity 获取指定坐标系
 * 在指定参考系下的 6D 速度。
 *
 * \extends CartesianVelocityNorm
 *
 * \tparam _Model Pinocchio 模型类型（默认 pinocchio::Model）。
 * */
template<typename Model = pinocchio::Model>
class Pinocchio;

template<typename _Model>
class Pinocchio : public CartesianVelocityNorm {
  public:
    typedef _Model Model;
    typedef typename Model::Data Data;

    /// \brief 打印约束信息，前缀标注使用 Pinocchio 后端。
    std::ostream& print(std::ostream& os) const
    {
      return CartesianVelocityNorm::print(os << "Pinocchio - ");
    }

    /// \brief 计算指定坐标系的 6D 速度。
    /// \param q 关节位形。
    /// \param qdot 关节速度。
    /// \param[out] v 输出：6 维坐标系速度（3 线速度 + 3 角速度）。
    ///
    /// 先执行前向运动学（含速度），再从结果中提取目标坐标系
    /// 在 m_reference_frame 参考系下的速度。
    void computeVelocity (const Vector& q, const Vector& qdot, Vector& v)
    {
      // 前向运动学：计算所有关节的位姿和速度
      pinocchio::forwardKinematics(m_model, m_data, q, qdot);
      // 获取指定坐标系（m_frame_id）在指定参考系下的 6D 速度
      v = pinocchio::getFrameVelocity(m_model, m_data, m_frame_id, m_reference_frame).toVector();
    }

    /// \brief 构造函数（常数速度限值）。
    /// \param model Pinocchio 机器人模型。
    /// \param S 6×6 选择/加权矩阵。
    /// \param limit 速度范数上限。
    /// \param frame 目标坐标系的索引（pinocchio::FrameIndex）。
    /// \param ref_frame 参考系类型，默认为 LOCAL_WORLD_ALIGNED（世界坐标系对齐）。
    Pinocchio (const Model& model, const Matrix& S, const double& limit,
        pinocchio::FrameIndex frame,
        pinocchio::ReferenceFrame ref_frame = pinocchio::LOCAL_WORLD_ALIGNED)
      : CartesianVelocityNorm (S, limit)
      , m_model (model)
      , m_data (model)
      , m_frame_id (frame)
      , m_reference_frame (ref_frame)
    {
    }

    /// \brief 移动构造函数，转移数据结构所有权。
    /// \note m_model 为引用，不转移所有权；m_data 转移内部缓存。
    Pinocchio (Pinocchio&& other)
      : CartesianVelocityNorm(other)
      , m_model (other.m_model)
      , m_data (std::move(other.m_data))
      , m_frame_id (m_frame_id)
      , m_reference_frame (m_reference_frame)
    {}

    /// \brief 返回对内部 Pinocchio 模型的常引用。
    const Model& model() const { return m_model; }

  protected:
    /// \brief 构造函数（变速度限值）。
    /// \param model Pinocchio 机器人模型。
    /// \param frame 目标坐标系索引。
    /// \param ref_frame 参考系类型，默认为 LOCAL_WORLD_ALIGNED。
    ///
    /// 子类需在 computeVelocityLimit 中设置 m_limit 和 m_S。
    Pinocchio (const Model& model,
        pinocchio::FrameIndex frame,
        pinocchio::ReferenceFrame ref_frame = pinocchio::LOCAL_WORLD_ALIGNED)
      : CartesianVelocityNorm ()
      , m_model (model)
      , m_data (model)
      , m_frame_id (frame)
      , m_reference_frame (ref_frame)
    {}


  private:
    const Model& m_model;              ///< 对 Pinocchio 模型的引用。
    Data m_data;                       ///< Pinocchio 运行时数据，缓存中间计算结果。
    pinocchio::FrameIndex m_frame_id;  ///< 目标坐标系索引。
    pinocchio::ReferenceFrame m_reference_frame; ///< 速度参考系（LOCAL / LOCAL_WORLD_ALIGNED / WORLD）。
}; // class Pinocchio

} // namespace cartesianVelocityNorm
} // namespace constraint
} // namespace toppra

#endif
