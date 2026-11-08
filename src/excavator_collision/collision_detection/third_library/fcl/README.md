# Vendored FCL

本目录是从 `/home/zhanghj/kaiyuan/fcl`（上游 FCL 0.7.0，
<https://github.com/flexible-collision-library/fcl>）移植的本地碰撞算法实现，
供 `excavator_collision` 以静态库 `excavator_fcl` 的形式链接，避免运行时依赖
系统 `libfcl`。项目自己的 FCL 适配层是上一级目录中的
`fcl_collision_model.hpp/.cpp`，与本上游源码目录分开维护。

- 保留：`include/fcl`、`src`（broadphase / narrowphase / geometry / math / common）。
- 排除：OctoMap（`geometry/octree`），本项目暂不使用占据栅格碰撞。
- 依赖：Eigen3、libccd（系统包 `libccd-dev`）。
- 许可证：见 [LICENSE](LICENSE)（BSD 3-Clause）。

`COLCON_IGNORE` 用于阻止 colcon 将本目录识别为独立 ROS 包；它只通过
`excavator_collision/CMakeLists.txt` 中的 `add_subdirectory` 参与构建。
FCL 随后被静态链接进 `libexcavator_collision.so`，对外不暴露 FCL 类型或头文件。
