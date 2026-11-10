# Collision Detection Module

本目录是 `excavator_collision` 软件包。完整架构和设计说明见
[《挖机碰撞检测方案》](../../docs/collision_detection_design.md)。

## 设计要点

- **不依赖 URDF**：机器人碰撞几何从 YAML 加载；运动学由外部注入的
  `KinematicsModel` 提供（规划默认使用 `DhKinematicsModel`）。
- **FCL 本地化**：FCL 0.7.0 的核心算法移植在
  `third_library/fcl/`，构建为静态库 `excavator_fcl` 并私有链接进
  `excavator_collision`。运行时不依赖系统 `libfcl`，FCL 类型也不出现在公共头文件中。
  本仓库版本启用了 FCL 的 OctoMap/Octree 后端，因此构建需要系统 `octomap`。
- **职责分离**：`RobotCollisionGeometryModel` 只描述几何与局部位姿；`FclCollisionModelBuilder`
  持有机器人模型和运动学模型，负责把关节位置转换为连杆碰撞信息；`FclCollisionChecker`
  负责环境对象构建、碰撞/距离查询和路径采样。

## 目录职责

```text
excavator_collision/
├── README.md
├── include/excavator_collision/
│   ├── collision_checker_interface.hpp
│   ├── fcl_collision_checker.hpp
│   ├── robot_collision_geometry_model.hpp
│   └── collision_checker_factory.hpp
├── src/
│   ├── fcl_collision_checker.cpp
│   ├── robot_collision_geometry_model.cpp
│   ├── fcl_collision_model_builder.hpp       # 项目 FCL 适配层
│   ├── fcl_collision_model_builder.cpp
│   └── collision_checker_factory.cpp
└── third_library/
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
  root_frame: base_link          # FK 输出与障碍物位姿使用的坐标系

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

- `CheckCollision`：自碰撞 + 环境碰撞。
- `CheckSelfCollision`：仅自碰撞。
- `CheckEnvironmentCollision`：仅环境碰撞。
- `CheckPathCollision(path, obstacles, max_joint_step)`：按最大关节步长离散采样，检查每个采样状态。
- `CheckPathCollision(path, obstacle, max_joint_step)`：只针对单个障碍物检查路径，不需要构造完整场景。
- `CheckTrajectory`：轨迹结构校验后，将轨迹位置采样并进行碰撞检查。
- `ComputeDistance(q, obstacle)`：给定关节状态下，机器人到单个障碍物的最小距离查询。
- `SetAllowedCollision(link_a, link_b, allowed)`：按无序对启用或撤销自碰撞豁免。

环境查询通过 `CollisionQueryOptions.obstacle_type` 选择障碍物表示：

- `kGeometry`（默认）：读取 `CollisionObject.primitive`，支持 Box/Sphere/Cylinder。
- `kPointCloud`：读取 `CollisionObject.point_cloud`，按 `point_cloud_resolution` 体素化为 Octree。

自碰撞默认跳过同一连杆几何体、直接父子连杆（来自 YAML 的 `parent_link_name`）。
规划服务、轨迹控制器和可视化测试节点默认不再配置额外允许碰撞对；原来的
`base_link:bucket_link` 等粗盒子过滤项会隐藏真实的铲斗-履带底盘碰撞，已移除。
需要结构性例外时可通过 `SetAllowedCollision()`（规划启动参数
`allowed_collision_pairs`）显式添加。

`CollisionCheckResult.valid` 表示查询是否成功执行，`collision` 才表示是否发现碰撞。
调用方不得将 `valid == false` 当作无碰撞。

## 环境障碍物

环境障碍物由 `Obstacle`（即 `CollisionObject`）列表表达：

- 几何表示：Box、Sphere、Cylinder；
- 点云表示：`sensor_msgs/PointCloud2`，按体素分辨率转换为 FCL Octree。

同一障碍物可以同时携带 `primitive` 和 `point_cloud`，具体使用哪一个由
`CollisionQueryOptions.obstacle_type` 决定。空点云、无有效点或非法分辨率都会导致
查询失败，不会静默当作无碰撞。

障碍物 pose 必须已表达在碰撞模型 `root_frame`（当前为 `base_link`）中；
本模块不查询 TF，也不执行坐标变换。点云坐标先按障碍物 `pose` 变换，再参与碰撞查询。

## 可视化碰撞测试

`demo.launch.py` 会启动 `excavator_visualization/collision_visual_test_node`：

1. 在 RViz `Excavator Planning` 面板中添加 Box 障碍物；
2. 拖动 `/excavator_obstacle` 中的红色障碍物盒子移动到任意位置；
3. 拖动 `/excavator_collision_test` 中的 `bucket_tip_drag` 青色小球，挖机爪通过 IK 跟随；
4. **点云障碍物**：选择 RViz 工具栏 `Publish Point`，在场景中逐点单击即可添加点云。
   节点会把点击点从 RViz Fixed Frame 转换到 `base_link`，按
   `point_cloud_resolution`（默认 0.05 m）体素化后做点云碰撞检测；点默认显示为橙色，
   碰撞时点云和碰撞连杆变亮红。清除命令：
   ```bash
   ros2 topic pub --once /excavator/clear_point_cloud std_msgs/msg/Empty "{}"
   ```
   Box 与点云可同时存在，节点分别按 `kGeometry` 和 `kPointCloud` 查询后合并结果。
5. 发生碰撞时，高亮**所有接触障碍物的连杆**的 YAML 碰撞几何与碰撞障碍物（自碰撞时
   高亮两个连杆），按亮红色显示并附带 `COLLISION: ...` 文本；继续添加点云点时，
   新接触的连杆会追加高亮；碰撞解除后标记自动清除。

该节点以 20 Hz 检查碰撞：几何障碍物来自 `/excavator/scene`，点云点来自
`/clicked_point`，关节状态来自 `/joint_states`。手动拖动命令通过
`/excavator/manual_joint_state` 发送给轨迹控制器。详细操作见
[《挖机碰撞检测方案》](../../docs/collision_detection_design.md) 第 11.1 节。

## 替换后端

Planner 只依赖 `CollisionChecker`。当前工厂注册的实现是 `fcl`。
后续新算法需要实现相同接口、在 `CollisionCheckerFactory` 中注册类型，再通过
`collision_backend` 参数选择。Planner 不应包含 FCL 头文件或依赖 FCL 特有类型。
