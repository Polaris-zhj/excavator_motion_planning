# Collision Detection Module

本目录是 `excavator_collision` 软件包内的碰撞检测实现层。完整架构和设计说明见
[《挖机碰撞检测方案》](../../../docs/collision_detection_design.md)。

## 目录职责

```text
collision_detection/
├── README.md
├── include/excavator_collision/internal/
│   └── robot_collision_model.hpp
├── src/
│   ├── collision_checker.cpp
│   └── robot_collision_model.cpp
└── third_library/
    └── fcl/
        ├── fcl_collision_model.hpp
        └── fcl_collision_model.cpp
```

- `RobotCollisionModel`：项目自有的机器人碰撞模型，保存连杆名称、父连杆关系、几何参数及碰撞几何体相对连杆的固定局部位姿；不包含 FCL 类型，也不计算 FK。
- `KinematicsModel`：独立的运动学接口，输入关节位置并输出指定连杆位姿。
- `FclCollisionModel`：唯一直接依赖 FCL 的适配层，负责项目几何类型转换、构造 FCL 查询对象，以及碰撞/距离查询。
- `UrdfFclCollisionChecker`：实现项目级 `CollisionChecker` 接口，协调 FK、机器人碰撞模型、场景验证和路径采样。

## 坐标变换

当前挖机 URDF 根坐标系为 `world`，URDF 和 Modified-DH 运动学后端都以 `base_link` 为 FK 参考坐标系。模型初始化时解析固定的根坐标系到参考坐标系变换：

```text
T_root_link = T_root_reference * T_reference_link
T_root_collision = T_root_link * T_link_collision
```

`T_reference_link` 来自 `KinematicsModel::GetLinkPose()`；
`T_link_collision` 来自 URDF collision 元素的 origin。
FCL 适配层不重新计算 FK。环境障碍物 pose 必须已经表达在 URDF 根坐标系中；
当前模块不查询 TF，也不执行场景 frame 转换。

## 查询接口

- `CheckState`：自碰撞 + 环境碰撞。
- `CheckSelfCollision`：仅自碰撞。
- `CheckEnvironmentCollision`：仅环境碰撞。
- `CheckPath`：按最大关节步长离散采样，检查每个采样状态。
- `CheckPathCollisionWithObstacle`：路径与指定环境障碍物的离散碰撞检查。
- `CheckTrajectory`：轨迹结构校验后，将轨迹位置采样并进行碰撞检查。
- `ComputeDistanceToObstacle`：给定关节状态下，机器人到指定障碍物的最小距离查询。

自碰撞默认跳过同一连杆几何体、直接父子连杆对，以及用户配置的允许碰撞连杆对。

`CollisionCheckResult.valid` 表示查询是否成功执行，`collision` 才表示是否发现碰撞。
调用方不得将 `valid == false` 当作无碰撞。

## 当前支持范围

机器人碰撞几何由 URDF 提供：

- Box、Sphere、Cylinder。
- URDF 引用的 ASCII STL 和 Binary STL 三角网格。
- `package://` URI 和文件系统路径。

环境障碍物由 `PlanningScene` 的 `CollisionObject` 表达，目前支持 Box、Sphere 和
Cylinder。当前没有实现原始点云、体素地图、OctoMap/OcTree、环境三角网格或动态障碍物预测。
不支持的几何输入会导致查询失败，不会静默忽略。

规划服务只接受完整场景快照（`is_diff == false`），并保存最近收到的完整场景。
若请求没有提供场景内容，则使用最近的场景快照。场景障碍物 pose 必须位于 URDF 根 frame。

路径碰撞检测使用关节空间线性插值和离散采样；这不是连续碰撞检测，无法证明采样点之间
绝对无碰撞。减小 `max_joint_step` 可以提高采样密度，但会增加计算开销。

## 替换后端

Planner 只依赖 `CollisionChecker`。当前工厂已注册的实现是 `urdf_fcl`。
后续新算法需要实现相同接口、在 `CollisionCheckerFactory` 中注册类型，再通过
`collision_backend` 参数选择。Planner 不应包含 FCL 头文件或依赖 FCL 特有类型。
