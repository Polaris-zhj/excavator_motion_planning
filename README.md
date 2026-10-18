# 挖机轨迹规划与仿真框架

本项目基于 **ROS 2 Humble、C++17**，提供挖掘机模型、关节空间轨迹规划接口、运动学计算和轨迹可视化/仿真。代码遵循 Google C++ Style，运行时不依赖 MoveIt 2。

当前第一阶段实现的是**运动学仿真**，不是液压或刚体动力学仿真。液压缸驱动、液压压力/流量、土壤接触与动力学暂不在实现范围内。

## 1. 模型说明

模型描述包位于 `src/excavator_description/`，包含 URDF/Xacro 和网格资源。当前规划使用四个主要关节：

- `swing_joint`：回转关节
- `boom_joint`：动臂关节
- `stick_joint`：斗杆关节
- `bucket_joint`：铲斗关节

Xacro 同时定义了末端参考坐标系 `bucket_tip`，可用于末端位姿查询。

模型来源及许可信息见[模型归属与许可说明](src/excavator_description/NOTICE.md)。大部分网格资源来自 [Polaris-zhj/ws_excavator](https://github.com/Polaris-zhj/ws_excavator)。由于原始 `base_link.STL` 文件无法通过当前 GitHub 文件传输方式获取，仓库中的该文件暂时是生成的简化底盘/履带模型，并非原始 CAD 网格。若本地已有上游仓库，可执行以下脚本恢复原始文件：

```bash
bash scripts/replace_base_link_mesh.sh ../ws_excavator
```

恢复后请检查网格路径、模型比例及原始资源的再分发许可。

## 2. 软件包结构

| 软件包 | 职责 |
| --- | --- |
| `excavator_description` | 挖机 URDF/Xacro 与网格模型 |
| `excavator_interfaces` | ROS 2 消息，以及 `PlanTrajectory`、`ExecuteTrajectory` Action 定义 |
| `excavator_common` | `JointPath`、`JointTrajectory`、机器人状态/配置、关节限位与轨迹校验、可替换规划接口 |
| `kinematics` | 基于 URDF 的正运动学（FK）、几何雅可比、阻尼最小二乘逆运动学（IK）及奇异性指标 |
| `excavator_planning` | 直线关节空间基线规划器、分段时间参数化、轨迹插值与 `/plan_trajectory` Action Server |
| `excavator_sim_viz` | 轨迹播放、`/joint_states` 发布、RViz 模型显示及轨迹执行 Action 桥接 |

规划算法通过接口解耦，后续可替换为 RRT、RRT*、PRM 或自定义算法，而不必改变上层调用方式。

**当前安全边界：**直线规划器只是可运行的基线，不执行环境或自碰撞检测；时间参数化也不是时间最优或 jerk 受限算法。规划成功不代表轨迹无碰撞或可用于真实设备。请在仿真环境充分验证后再考虑接入真实控制系统。

## 3. 环境要求与编译

- Ubuntu 22.04
- ROS 2 Humble
- C++17、Eigen、URDF、colcon

安装依赖并编译：

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-urdf ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-rviz2 libeigen3-dev

source /opt/ros/humble/setup.bash
mkdir -p ~/excavator_ws/src
cd ~/excavator_ws/src
git clone https://github.com/Polaris-zhj/excavator_motion_planning.git

cd ~/excavator_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

如果终端新开了会话，需要重新执行：

```bash
source /opt/ros/humble/setup.bash
source ~/excavator_ws/install/setup.bash
```

## 4. 启动模型可视化与轨迹仿真

执行：

```bash
ros2 launch excavator_sim_viz model_view.launch.py
```

该启动文件会启动模型、RViz、轨迹播放器和执行 Action 桥接。轨迹播放器以 **50 Hz** 发布 `/joint_states`，由 `robot_state_publisher` 根据同一关节状态流计算 TF。

### 发布一条示例轨迹

在另一个终端（同样 source 工作空间）执行。轨迹中的关节顺序必须与 `joint_names` 一致：

```bash
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [swing_joint, boom_joint, stick_joint, bucket_joint], points: [
  {positions: [0.0, -0.35, 0.45, -0.4], time_from_start: {sec: 0, nanosec: 0}},
  {positions: [0.4, -0.2, 0.3, -0.4], time_from_start: {sec: 3, nanosec: 0}},
  {positions: [0.0, -0.35, 0.45, -0.4], time_from_start: {sec: 6, nanosec: 0}}
]}"
```

播放器会对轨迹点之间的关节位置做线性插值。新轨迹提交后会替换当前活动轨迹；无效轨迹不会替换正在执行的轨迹。

## 5. 规划一条关节空间轨迹

在独立终端启动规划服务：

```bash
ros2 launch excavator_planning planning_server.launch.py
```

下面的请求演示从起始关节位置规划到目标关节位置：

```bash
ros2 action send_goal /plan_trajectory excavator_interfaces/action/PlanTrajectory \
"{start_state: {name: [swing_joint, boom_joint, stick_joint, bucket_joint], position: [0.0, -0.35, 0.45, -0.4]}, goal_state: {name: [swing_joint, boom_joint, stick_joint, bucket_joint], position: [0.2, -0.3, 0.4, -0.3]}, target_link: bucket_tip, allowed_planning_time: {sec: 1, nanosec: 0}, planning_scene: {is_diff: false, objects: []}}"
```

当前规划器只提供直线关节空间路径基线。请求包含障碍物时，服务会拒绝规划，因为 FCL 碰撞检测后端尚未接入。即使空场景请求成功，也**不提供碰撞安全保证**。

规划结果可发布到 `/joint_trajectory` 播放，或交给 `/execute_trajectory` Action 执行。执行 Action 默认要求显式确认（参数 `require_confirmation: true`），并会检查轨迹起点与当前关节状态是否连续；任一关节起点偏差超过约 0.05 rad 时会拒绝执行。

## 6. 运动学接口示例

`kinematics` 包可以从展开后的 URDF XML 或 URDF 文件加载模型。存在 `base_link` 时，FK 以 `base_link` 为参考系；运动学调用不需要维护可变的“当前关节状态”。

先将 Xacro 展开成 URDF：

```bash
xacro src/excavator_description/urdf/excavator.urdf.xacro > /tmp/excavator.urdf
```

C++ 示例：

```cpp
#include "kinematics/urdf_kinematics_model.hpp"

tramcar::planning::UrdfKinematicsModel kinematics;
const auto load = kinematics.InitializeFromFile("/tmp/excavator.urdf");
if (!load.ok) {
  // 根据 load.message 处理错误。
}

tramcar::planning::Pose3 pose;
const auto fk = kinematics.GetLinkPose(
    {0.0, -0.35, 0.45, -0.4}, "bucket_tip", &pose);
```

默认关节顺序为回转、动臂、斗杆、铲斗。使用前请核对实际模型的关节限位和坐标系定义。

## 7. 测试

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

GitHub Actions 工作流会尝试构建 ROS 2 工作空间、运行单元测试并验证 Xacro 文件。请以实际工作流结果为准；代码尚需在 Ubuntu 22.04 + ROS 2 Humble 环境中完成整体验证，包括 RViz 模型与 TF 检查。

## 8. 当前未完成项

- 将仓库中的简化 `base_link.STL` 替换为上游原始 CAD 网格，并确认许可。
- 接入 FCL 等碰撞检测后端，补充机器人自碰撞与环境碰撞检测。
- 对路径中间点、插值轨迹及最终轨迹执行完整的碰撞与可行性校验。
- 完善速度、加速度和 jerk 约束；当前时间参数化不是完整的时间最优算法。
- 在真实 ROS 2 环境中完成工作空间编译、单元测试、RViz 显示和端到端轨迹执行验证。
- 按后续需求完善在线轨迹更新、路径曲线/场景可视化与离线回放。

本项目当前定位为**可扩展的基础框架与仿真验证起点**，不是已完成安全认证的挖掘机运动控制系统。
