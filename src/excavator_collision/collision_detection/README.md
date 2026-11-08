# Collision Detection Module

本目录是 `excavator_collision` 软件包内的碰撞检测实现层。完整架构和设计说明见
[《挖机碰撞检测方案》](../../../docs/collision_detection_design.md)。

## 设计要点

- **不依赖 URDF**：机器人碰撞几何从 YAML 加载；运动学由外部注入的
  `KinematicsModel` 提供（规划默认使用 `DhKinematicsModel`）。
- **FCL 本地化**：FCL 0.7.0 的核心算法移植在
  `third_library/fcl/`，构建为静态库 `excavator_fcl` 并私有链接进
  `excavator_collision`。运行时不依赖系统 `libfcl`，FCL 类型也不出现在公共头文件中。
- **职责分离**：`RobotCollisionModel` 只描述几何与局部位姿；`FclCollisionModel`
  只负责 FCL 对象和查询；`FclCollisionChecker` 负责组织 FK、场景和路径采样。

## 目录职责

```text
collision_detection/
├── README.md
├── src/
│   ├── collision_checker.cpp
│   └── robot_collision_model.cpp
└── third_library/
    ├── fcl_collision_model.hpp       # 项目 FCL 适配层
    ├── fcl_collision_model.cpp
    └── fcl/                          # 移植的 FCL 0.7.0 上游源码与 CMake
        ├── include/fcl/
        ├── src/
        ├── CMakeLists.txt            # 构建静态库 excavator_fcl
        ├── LICENSE
        └── README.md
```

## 碰撞模型 YAML

```yaml
robot:
  name: excavator
  root_frame: base_link          # FK 输出与 PlanningScene 使用的坐标系

links:
  - link_name: boom_link
    parent_link_name: swing_link # 仅用于自碰撞相邻连杆过滤
    geometries:
      - name: boom_body
        type: mesh
        resource: "package://excavator_description/meshes/pjjq03.STL"
        scale: [1.0, 1.0, 1.0]
        origin:
          xyz: [0.0, 0.0, 0.0]
          rpy: [0.0, 0.0, 0.0]
```

支持的几何类型：

| type | 参数 | FCL 表示 |
| --- | --- | --- |
| `box` | `size: [x, y, z]` | `fcl::Boxd` |
| `sphere` | `radius` | `fcl::Sphered` |
| `cylinder` | `radius`, `length` | `fcl::Cylinderd` |
| `capsule` | `radius`, `length` | `fcl::Capsuled` |
| `mesh` | `resource`, 可选 `scale` | `fcl::BVHModel<fcl::OBBRSSd>` |

Mesh 支持 ASCII/Binary STL，资源路径支持 `package://`、绝对路径以及相对
YAML 文件所在目录的路径。默认挖机碰撞模型位于
`excavator_description/config/excavator_collision.yaml`。

`root_frame` 必须与注入的 `KinematicsModel` 的位姿参考系一致。当前 DH 与 URDF
运动学后端都输出 `base_link` 下的位姿，因此场景障碍物也应在 `base_link` 下表达。

## 坐标变换

```text
T_root_collision = T_root_link * T_link_collision
```

`T_root_link` 来自 `KinematicsModel::GetLinkPose()`；
`T_link_collision` 来自 YAML 的 `origin`。碰撞后端的初始化只接受非空
`KinematicsModel`，不会自行解析 URDF 或构造回退 FK。

## 查询接口

- `CheckState`：自碰撞 + 环境碰撞。
- `CheckSelfCollision`：仅自碰撞。
- `CheckEnvironmentCollision`：仅环境碰撞。
- `CheckPath`：按最大关节步长离散采样，检查每个采样状态。
- `CheckPathCollisionWithObstacle`：路径与指定环境障碍物的离散碰撞检查。
- `CheckTrajectory`：轨迹结构校验后，将轨迹位置采样并进行碰撞检查。
- `ComputeDistanceToObstacle`：给定关节状态下，机器人到指定障碍物的最小距离查询。

自碰撞默认跳过同一连杆几何体、直接父子连杆（来自 YAML 的 `parent_link_name`），
以及用户配置的允许碰撞连杆对。

`CollisionCheckResult.valid` 表示查询是否成功执行，`collision` 才表示是否发现碰撞。
调用方不得将 `valid == false` 当作无碰撞。

## 环境障碍物

环境障碍物由 `PlanningScene` 的 `CollisionObject` 表达，目前支持 Box、Sphere 和
Cylinder。当前没有实现原始点云、体素地图、OctoMap/OcTree、环境三角网格或动态障碍物预测。
不支持的几何输入会导致查询失败，不会静默忽略。

场景的障碍物 pose 必须已表达在碰撞模型 `root_frame`（当前为 `base_link`）中；
本模块不查询 TF，也不执行场景 frame 转换。

## 可视化碰撞测试

`demo.launch.py` 会启动 `excavator_visualization/collision_visual_test_node`：

1. 在 RViz `Excavator Planning` 面板中添加 Box 障碍物；
2. 拖动 `/excavator_obstacle` 中的红色障碍物盒子移动到任意位置；
3. 拖动 `/excavator_collision_test` 中的 `bucket_tip_drag` 青色小球，挖机爪通过 IK 跟随；
4. 发生碰撞时，机器人 YAML 碰撞几何与障碍物立即显示为亮红色，并附带
   `COLLISION: ...` 文本；碰撞解除后标记自动清除。

该节点以 20 Hz 调用 `FclCollisionChecker::CheckState()`，场景来自
`/excavator/scene`，关节状态来自 `/joint_states`。手动拖动命令通过
`/excavator/manual_joint_state` 发送给轨迹控制器。详细操作见
[《挖机碰撞检测方案》](../../../docs/collision_detection_design.md) 第 11.1 节。

## 替换后端

Planner 只依赖 `CollisionChecker`。当前工厂注册的实现是 `fcl`。
后续新算法需要实现相同接口、在 `CollisionCheckerFactory` 中注册类型，再通过
`collision_backend` 参数选择。Planner 不应包含 FCL 头文件或依赖 FCL 特有类型。
