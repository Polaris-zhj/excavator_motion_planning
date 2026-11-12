# excavator_kinematics

独立的 ROS 2 / C++17 运动学算法包。提供项目自有的运动学模型和求解器接口，并使用本目录内移植的 Orocos KDL 实现 FK、几何 Jacobian 和位置 IK。不依赖 MoveIt 2，也不依赖 excavator_common；本次改动不涉及其它软件包。

## 1. 设计边界

- KinematicsModel 描述静态 Link / Joint 拓扑和关节参数，不保存运行时关节状态。
- KinematicsSolver 定义 FK、批量 FK、相对位姿、Jacobian 和 IK 等通用接口。
- KinematicsSolverFactory 按 backend 创建算法实现，当前注册 kdl，后续可增加自研 Backend。
- ModelConfigParser、UrdfModelConverter、DhModelConverter 将 YAML、URDF、Modified DH 转换成统一 ModelDescription。
- backend/kdl 是 KDL Adapter；third_library/KDL 保存上游源码。KDL 类型不出现在公共 API。
- 本包不加载碰撞几何、不依赖 FCL、不负责轨迹规划或动力学。KinematicsModel 与碰撞模型保持独立。

## 2. 目录结构

```text
excavator_kinematics/
├── include/excavator_kinematics/  # 公共类型和接口
├── kinematics/                    # 统一模型与 Factory
├── model_converter/               # YAML / URDF / Modified DH 转换
├── backend/kdl/                   # KDL Adapter
├── third_library/KDL/             # Orocos KDL 1.5.4 上游源码
├── config/
└── test/
```

## 3. 坐标系与关节约定

- 模型必须有且只有一个 root_link。
- GetLinkPose(q, link) 返回 root_link 到 link 的位姿，结果在 root_link 坐标系表达。
- IkRequest.target_pose 也在 root_link 坐标系表达，不需要额外传 base_link。
- GetRelativePose(q, reference, target) 返回目标 Link 在参考 Link 坐标系下的位姿：

```text
T_reference_target = inverse(T_root_reference) * T_root_target
```

- JointModel.origin 表示 Parent Link 到 Joint 零位坐标系的固定变换。
- JointModel.axis 在 Joint 零位坐标系下表达，正方向遵循右手定则。
- revolute / continuous 的位置单位为 rad；prismatic 的位置单位为 m。
- joint_order 是调用方 JointVector 的唯一顺序，只列出独立活动关节。未列入 joint_order 的可动关节保留在拓扑中，但在当前模型中按 q = 0 锁定；固定关节也不占用向量维度。
- Jacobian 为 6×N，前三行为线速度、后三行为角速度，表达在 root_link 坐标系，参考点为目标 Link 原点。

## 4. 模型输入

### URDF

UrdfModelConverter::FromFile / FromString 读取已展开的 URDF XML，转换 Link 树、fixed / revolute / continuous / prismatic Joint、origin、axis 和限位。若输入是 Xacro，应先通过 xacro 命令展开成 URDF；本包不依赖 xacro，也不负责解析宏。不支持的关节类型会返回错误。默认情况下，URDF 中所有非固定关节按树遍历顺序构成活动关节顺序。也可以向 FromFile / FromString 传入 active_joint_order，只选择独立规划关节；其余可动关节保留在拓扑中并锁定在 q = 0，适用于当前阶段暂不建模的液压缸或被动连杆自由度。挖机模型可以显式选择 swing_joint、boom_joint、stick_joint、bucket_joint 作为活动关节，避免将液压缸/连杆关节混入规划向量。

### 自定义 YAML

ModelConfigParser 的基本格式：

```yaml
kinematics_model:
  name: arm
  root_link: base_link
  joint_order: [joint1, joint2]
  links:
    - {name: base_link}
    - {name: link1}
    - {name: link2}
  joints:
    - name: joint1
      type: revolute
      parent_link: base_link
      child_link: link1
      origin:
        xyz: [0.0, 0.0, 0.5]
        rpy: [0.0, 0.0, 0.0]
      axis: [0.0, 0.0, 1.0]
      limit: {lower: -3.14, upper: 3.14, velocity: 0.8}
    - name: joint2
      type: prismatic
      parent_link: link1
      child_link: link2
      origin:
        xyz: [1.0, 0.0, 0.0]
        rpy: [0.0, 0.0, 0.0]
      axis: [0.0, 0.0, 1.0]
```

模型初始化会校验唯一名称、根节点、连通无环拓扑、每个非根 Link 的唯一父关节、活动关节顺序、关节轴和限位参数。

### Modified DH

第一版支持 convention: modified_dh，不自动猜测 Standard DH。采用的单节变换约定为：

```text
T(i-1,i) = TransX(a_prev) * RotX(alpha_prev)
            * RotZ(theta_offset + theta_sign * q)
            * TransZ(d)
```

每行参数展开为活动旋转关节、固定 d 偏置和物理 Link 固定偏置。theta_sign 显式指定关节正方向，不从图形模型猜测。使用 MDH 参数前，必须确认参数定义和上述公式一致。

## 5. KDL Backend

- FK：KDL::ChainFkSolverPos_recursive
- Jacobian：KDL::ChainJntToJacSolver
- IK 速度求解：KDL::ChainIkSolverVel_wdls
- IK 位置求解：KDL::ChainIkSolverPos_NR_JL

KdlModelConverter 在初始化时从 root_link 遍历模型树，选择一条包含全部 active joints 的最长 root-to-leaf 主链，构建唯一的完整 KDL Chain，并建立 Link 名称到 segment 索引的映射。若活动关节分布在不同分支，或存在长度相同且都包含全部活动关节的候选主链，则返回 kUnsupported；不在选定主链上的分支 Link 也会在查询时明确返回 kUnsupported。该限制是单 Chain 后端的边界，不会把分支 Link 错误地当作主链 Link。

KDL Joint 的 axis 使用 Joint origin 旋转后的方向，并在 Parent Link 坐标系表达。Segment 接收 Parent-Link 到 Child-Link 的完整零位变换；KDL::Segment 会处理 Joint 零位运动变换的分解，避免重复应用 origin 平移。

KdlKinematicsSolver 自身持有完整 Chain、FK/Jacobian solver、关节索引/限位映射和 Link-segment 映射，不再使用独立的 KdlSolverBundle。FK 使用 JntToCart(q, frame, segmentNr)，Jacobian 使用 JntToJac(q, jacobian, segmentNr)，因此可在同一完整 Chain 上查询任意主链 Link。相对位姿由两个 root-relative FK 结果组合得到。IK 根据目标 Link 的 segment 截取前缀 Chain，并为该次请求创建 NR_JL 求解器；这样中间 Link IK 只使用到该 Link 为止的活动关节。求解成功后会再次调用完整 Chain FK，校验目标 Link 的位置和姿态误差。Backend 对 solver 调用加锁，避免并发请求同时访问非线程安全的 KDL 实例。

## 6. 使用方式

```cpp
#include <memory>
#include "excavator_kinematics/kinematics_solver_factory.hpp"
#include "excavator_kinematics/model_converter/urdf_model_converter.hpp"

using namespace excavator::planning;

ModelDescription description;
KinematicsResult parsed =
    UrdfModelConverter::FromFile(\n        "excavator.urdf", &description,\n        {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"});
if (!parsed.ok()) {
  // Handle parse error.
}

auto model = std::make_shared<KinematicsModel>();
KinematicsResult model_result = model->Initialize(description);
if (!model_result.ok()) {
  // Handle model validation error.
}

KinematicsSolverConfig config;
config.backend = "kdl";
auto solver = KinematicsSolverFactory::Instance().Create(config);
if (!solver) {
  // Backend is not registered.
}
KinematicsResult init_result = solver->Initialize(model);
if (!init_result.ok()) {
  // Handle backend initialization error.
}

JointVector q = JointVector::Zero(
    static_cast<Eigen::Index>(model->GetActiveJointNames().size()));
Pose bucket_pose;
KinematicsResult fk_result =
    solver->GetLinkPose(q, "bucket_tip", &bucket_pose);
```

## 7. 编译与测试

在 ROS 2 Humble 工作空间中：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to excavator_kinematics --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select excavator_kinematics
colcon test-result --verbose
```

测试覆盖统一模型拓扑校验、自定义 YAML / URDF / Modified DH 转换、根坐标系 FK、Jacobian 列映射、IK 和 Factory 创建。实际编译和运行结果应以 ROS 2 Humble 环境中的 colcon 测试为准。

## 8. KDL 上游来源与许可

third_library/KDL 的源码来自 orocos/orocos_kinematics_dynamics，版本标签 1.5.4，上游 commit 84b086838dea2b1132064517a61017d30945f1d3。保留上游文件版权声明和 third_library/KDL/COPYING。Orocos KDL 使用 LGPL-2.1-or-later；再分发时需遵守对应许可义务。

本包将所需 KDL 源文件编译为单独的 excavator_kdl 共享库，再由 excavator_kinematics 私有链接。这样保留 KDL 的可替换动态链接边界，便于遵循 LGPL 的再链接要求。调用方只包含本包公共头文件，不需要包含 KDL 头文件，也不需要在公共 API 中处理 KDL 类型。
