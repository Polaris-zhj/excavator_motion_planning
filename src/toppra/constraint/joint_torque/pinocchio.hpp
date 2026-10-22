#ifndef TOPPRA_CONSTRAINT_JOINT_TORQUE_PINOCCIO_HPP
#define TOPPRA_CONSTRAINT_JOINT_TORQUE_PINOCCIO_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <toppra/constraint/joint_torque.hpp>

namespace toppra {
namespace constraint {
namespace jointTorque {

/** \brief 基于 Pinocchio 的关节力矩约束实现。
 *
 * 使用 pinocchio::rnea（递归牛顿-欧拉算法）计算逆动力学，
 * 即给定 q, q̇, q̈ 计算关节力矩 τ。
 *
 * \extends JointTorque
 *
 * \tparam _Model Pinocchio 模型类型（默认 pinocchio::Model）。
 * */
template<typename Model = pinocchio::Model>
class Pinocchio;

template<typename _Model>
class Pinocchio : public JointTorque {
  public:
    typedef _Model Model;
    typedef typename Model::Data Data;

    /// \brief 打印约束信息，前缀标注使用 Pinocchio 后端。
    std::ostream& print(std::ostream& os) const
    {
      return JointTorque::print(os << "Pinocchio - ");
    }

    /// \brief 使用 Pinocchio RNEA 计算逆动力学。
    /// \param q 关节位形。
    /// \param v 关节速度。
    /// \param a 关节加速度。
    /// \param[out] tau 输出：关节力矩。
    ///
    /// RNEA（Recursive Newton-Euler Algorithm）是计算机器人
    /// 逆动力学的经典高效算法，复杂度 O(n)。
    void computeInverseDynamics (const Vector& q, const Vector& v, const Vector& a,
        Vector& tau)
    {
      tau = pinocchio::rnea(m_model, m_data, q, v, a);
    }

    /// \brief 构造函数（引用模型）。
    /// \param model Pinocchio 机器人模型（引用，不持有所有权）。
    /// \param frictionCoeffs 各关节干摩擦系数，默认为空（不考虑摩擦）。
    ///
    /// 力矩上下限自动从 model.effortLimit 获取。
    Pinocchio (const Model& model, const Vector& frictionCoeffs = Vector())
      : JointTorque (-model.effortLimit, model.effortLimit, frictionCoeffs)
      , m_model (model)
      , m_data (model)
    {
    }

    /// \brief 移动构造函数，转移模型和数据结构所有权。
    Pinocchio (Pinocchio&& other)
      : JointTorque(other)
      , m_storage (std::move(other.m_storage))
      , m_model (other.m_model)
      , m_data (std::move(other.m_data))
    {}

    /// \brief 构造函数（从 URDF 文件加载模型）。
    /// \param urdfFilename URDF 文件路径。
    /// \param friction 各关节干摩擦系数，默认为空。
    ///
    /// 内部调用 makeModel 从 URDF 构建 Pinocchio 模型并持有所有权。
    Pinocchio (const std::string& urdfFilename, const Vector& friction = Vector())
      : Pinocchio (makeModel(urdfFilename), friction) {}

    /// \brief 返回对内部 Pinocchio 模型的常引用。
    const Model& model() const { return m_model; }

  private:
    /// \brief 从 URDF 文件构建 Pinocchio 模型。
    /// \param urdfFilename URDF 文件路径。
    /// \return 新建模型指针（调用方持有所有权）。
    static Model* makeModel (const std::string& urdfFilename)
    {
      Model* model (new Model);
      pinocchio::urdf::buildModel(urdfFilename, *model);
      return model;
    }

    /// \brief 接管模型指针的构造函数。
    /// \param model 模型指针，本对象接管其所有权。
    /// \param friction 各关节干摩擦系数。
    ///
    /// 用于从 URDF 构建的场景，模型存储在 m_storage 中。
    Pinocchio (Model* model, const Vector& friction)
      : JointTorque (-model->effortLimit, model->effortLimit, friction)
      , m_storage (model)
      , m_model (*model)
      , m_data (m_model)
    {
    }

    /// 当本对象拥有模型时（如从 URDF 加载），模型存储于此。
    std::unique_ptr<Model> m_storage;
    const Model& m_model; ///< 对 Pinocchio 模型的引用（可能指向 m_storage）。
    Data m_data;          ///< Pinocchio 运行时数据结构，缓存中间计算结果。
}; // class Pinocchio

} // namespace jointTorque
} // namespace constraint
} // namespace toppra

#endif
