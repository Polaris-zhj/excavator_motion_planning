# Orocos KDL 运动学移植清单（挖机臂 FK / IK）

> 适用场景：挖机臂轨迹规划，仅使用 KDL 的**运动学**能力（建模 + 正解 + 逆解 + 雅可比），
> 不使用笛卡尔轨迹生成、速度剖面、逆动力学。
> 基准版本：orocos_kdl 1.5.4（本仓库）。
> 路径约定：下列相对路径均以 `src/` 为根。

---

## 1. 适用范围

### 1.1 移植后要能做的事

| 能力 | 对应类 |
|---|---|
| 串联链建模（关节 + 连杆偏移） | `Chain` / `Segment` / `Joint` |
| 基座坐标系到末端的齐次变换 | `Frame` / `Rotation` / `Vector` |
| 正运动学（关节角 → 末端位姿） | `ChainFkSolverPos_recursive` |
| 雅可比（关节角 → 6×n 矩阵） | `ChainJntToJacSolver` |
| 逆速度运动学（末端速度 → 关节速度） | `ChainIkSolverVel_wdls` |
| 逆位置运动学（末端位姿 → 关节角，带限位） | `ChainIkSolverPos_NR_JL` |
| 改进 DH 建链 | `Rotation::RotX` + `Vector`（`frames` 内） |

### 1.2 明确不包含的能力

- 笛卡尔路径（直线 / 圆弧 / 组合 / 圆滑过渡）
- 速度剖面（梯形 / 半梯形 / 样条 / 矩形 / Dirac）
- 轨迹合成与时间参数化
- 关节空间轨迹插值
- 逆动力学、动力学参数、外力估计
- 碰撞检测、工作空间约束
- 分支树结构（`Tree`）

---

## 2. 移植后的目录结构

```
third_party/kdl_kin/
├── frames.hpp
├── frames.inl
├── frames.cpp
├── joint.hpp
├── joint.cpp
├── segment.hpp
├── segment.cpp
├── chain.hpp
├── chain.cpp
├── jntarray.hpp
├── jntarray.cpp
├── jacobian.hpp
├── jacobian.cpp
├── rotationalinertia.hpp
├── rotationalinertia.cpp
├── rigidbodyinertia.hpp
├── rigidbodyinertia.cpp
├── solveri.hpp
├── chainfksolver.hpp
├── chainiksolver.hpp
├── framevel.hpp
├── framevel.inl
├── framevel.cpp
├── frameacc.hpp
├── frameacc.inl
├── frameacc.cpp
├── jntarrayvel.hpp
├── jntarrayvel.cpp
├── jntarrayacc.hpp
├── jntarrayacc.cpp
├── chainfksolverpos_recursive.hpp
├── chainfksolverpos_recursive.cpp
├── chainjnttojacsolver.hpp
├── chainjnttojacsolver.cpp
├── chainiksolvervel_wdls.hpp
├── chainiksolvervel_wdls.cpp
├── chainiksolverpos_nr_jl.hpp
├── chainiksolverpos_nr_jl.cpp
└── utilities/
    ├── kdl-config.h
    ├── utility.h
    ├── utility.cxx
    ├── utility_io.h
    ├── utility_io.cxx
    ├── hash_combine.h
    ├── traits.h
    ├── rall1d.h
    ├── rall2d.h
    ├── error.h
    ├── error_stack.h
    ├── error_stack.cxx
    ├── svd_eigen_HH.hpp
    └── svd_eigen_HH.cpp
```

> 头文件搜索路径需要同时包含 `kdl_kin/` 和 `kdl_kin/utilities/` 的**上一级**，
> 因为源码内部使用 `#include "utilities/xxx.h"` 与 `#include "frames.hpp"` 两种写法。

---

## 3. 必需文件清单

### 3.1 几何与数据结构（8 组）

| 文件 | 作用 | 关键依赖 |
|---|---|---|
| `frames.hpp` / `frames.inl` / `frames.cpp` | `Frame`、`Rotation`、`Vector`、`Twist`；`Frame::DH`、`Frame::DH_Craig1989` | `utilities/utility.h`、`kdl-config.h`、`hash_combine.h` |
| `joint.hpp` / `joint.cpp` | 关节类型、`scale`、`offset` | `frames.hpp` |
| `segment.hpp` / `segment.cpp` | 关节 + `f_tip` 固定变换 | `frames.hpp`、`joint.hpp`、**`rigidbodyinertia.hpp`** |
| `chain.hpp` / `chain.cpp` | 串联链容器，`getNrOfJoints()` 等 | `segment.hpp` |
| `jntarray.hpp` / `jntarray.cpp` | 关节角数组 | `frames.hpp`、`jacobian.hpp`、**Eigen** |
| `jacobian.hpp` / `jacobian.cpp` | 6×n 雅可比 | `frames.hpp`、**Eigen** |
| `rotationalinertia.hpp` / `.cpp` | 被 `rigidbodyinertia.hpp` 引入 | `frames.hpp` |
| `rigidbodyinertia.hpp` / `.cpp` | `Segment` 构造默认参数需要 | `rotationalinertia.hpp` |

### 3.2 求解器接口（4 组）

| 文件 | 说明 |
|---|---|
| `solveri.hpp` | 统一错误码基类，头文件即可 |
| `chainfksolver.hpp` | FK 抽象接口 |
| `chainiksolver.hpp` | IK 抽象接口 |
| `framevel.hpp` / `.inl` / `.cpp` | 被上面两个接口头文件引用 |
| `frameacc.hpp` / `.inl` / `.cpp` | 同上 |
| `jntarrayvel.hpp` / `.cpp` | 同上 |
| `jntarrayacc.hpp` / `.cpp` | 同上 |

> **重要**：`chainfksolver.hpp` 为了声明速度/加速度版本的纯虚函数，强制 include 了
> `framevel.hpp`、`frameacc.hpp`、`jntarrayvel.hpp`、`jntarrayacc.hpp`。
> 即使实际只调用位置解，也必须把这 7 个文件一起带过来。

### 3.3 求解器实现（4 组）

| 文件 | 用途 | 额外依赖 |
|---|---|---|
| `chainfksolverpos_recursive.hpp` / `.cpp` | 正运动学 | — |
| `chainjnttojacsolver.hpp` / `.cpp` | 雅可比 | — |
| `chainiksolvervel_wdls.hpp` / `.cpp` | 速度逆解（加权阻尼最小二乘） | `utilities/svd_eigen_HH.*`、Eigen |
| `chainiksolverpos_nr_jl.hpp` / `.cpp` | 位置逆解（Newton-Raphson + 关节限位） | 上述 FK 与 IKVel |

### 3.4 工具与数值

| 文件 | 说明 |
|---|---|
| `utilities/kdl-config.h` | 静态配置文件（非 CMake 生成），定义 `KDL_INLINE`、`KDL_INDEX_CHECK`、`KDL_USE_EQUAL` |
| `utilities/utility.h` / `utility.cxx` | `epsilon`、`sign`、数值辅助 |
| `utilities/hash_combine.h` | `frames.hpp` 使用 |
| `utilities/traits.h` | `framevel.hpp` 使用 |
| `utilities/rall1d.h` / `rall2d.h` | `framevel` / `frameacc` 的自动微分模板，头文件即可 |
| `utilities/error.h` | 错误基类 |
| `utilities/error_stack.h` / `error_stack.cxx` | 错误栈 |
| `utilities/utility_io.h` / `utility_io.cxx` | `error_stack.h` 串联引入 |
| `utilities/svd_eigen_HH.hpp` / `.cpp` | WDLS 使用的 SVD 实现 |

### 3.5 外部依赖

| 依赖 | 必需 | 说明 |
|---|---|---|
| **Eigen3**（仅头文件） | 是 | `jntarray.hpp`、`jacobian.hpp`、`chainiksolvervel_wdls.hpp` 使用 |
| **Boost** | 否 | 只有 `Tree` 的新接口使用；本方案不移植 `Tree` |
| C++ 标准 | C++11 及以上 | `frames.hpp` 使用 `std::function`、`std::hash` |
| 其他系统库 | 无 | 纯计算，无 IO、无锁、无线程 |

---

## 4. 依赖关系图

```
                    kdl-config.h / utility.h
                            │
                         frames.hpp/.inl/.cpp
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
 rotationalinertia     joint.hpp          jntarray.hpp ──► Eigen
        │                   │                   │
 rigidbodyinertia      segment.hpp         jacobian.hpp ──► Eigen
        │                   │                   │
        └───────────────► chain.hpp ◄───────────┘
                            │
                    solveri.hpp
                            │
              chainfksolver.hpp / chainiksolver.hpp
               │                              │
        framevel / frameacc            framevel / frameacc
        jntarrayvel / jntarrayacc      jntarrayvel / jntarrayacc
                            │
        ┌───────────────┬───┴────────────┬──────────────────┐
        │               │                │                  │
 chainfksolverpos  chainjnttojac   chainiksolvervel   chainiksolverpos
   _recursive        _solver          _wdls              _nr_jl
                                         │
                                  svd_eigen_HH (Eigen)
```

---

## 5. 不移植清单

| 文件 / 目录 | 原因 |
|---|---|
| `tree*.hpp` / `tree*.cpp` | 分支树结构，挖机为单链；且依赖 Boost |
| `path_*.hpp|.cpp`、`path.hpp|.cpp` | 笛卡尔路径生成 |
| `velocityprofile_*.hpp|.cpp` | 速度剖面 |
| `trajectory_*.hpp|.cpp` | 轨迹合成 |
| `rotational_interpolation*.hpp|.cpp` | 仅 `Path` 使用 |
| `motion.hpp` | 轨迹总入口 |
| `chainidsolver_*.hpp|.cpp` | 逆动力学 |
| `chaindynparam.*` | 动力学参数 |
| `chainfdsolver_*`、`chainhdsolver_*` | 前向动力学、Vereshchagin 混合动力学 |
| `chainexternalwrenchestimator.*` | 外力估计 |
| `articulatedbodyinertia.*` | 仅 HD 求解器使用 |
| `jntspaceinertiamatrix.*` | 动力学 |
| `stiffness.hpp` | 阻抗控制 |
| `chainfksolvervel_recursive.*` | 仅需位置正解时不需要；若需要末端速度可加回 |
| `frames_io.*`、`kinfam_io.*`、`framevel_io.*`、`frameacc_io.*` | 仅用于打印/日志；调试需要时可加回 |
| `models/`、`tests/`、`examples/`、`python/` | 示例与绑定 |

---

## 6. 移植步骤

- [ ] **步骤 1**：把第 3 节列出的文件按第 2 节目录结构拷贝到 `third_party/kdl_kin/`
- [ ] **步骤 2**：确认 `.inl` 文件一并拷贝（`frames.inl`、`framevel.inl`、`frameacc.inl`）
- [ ] **步骤 3**：确认 `utilities/kdl-config.h` 存在（这是源码文件，不是构建产物）
- [ ] **步骤 4**：配置 include 路径：`third_party/kdl_kin` 与 Eigen 头文件目录
- [ ] **步骤 5**：按第 7 节 CMake 片段建立静态库
- [ ] **步骤 6**：编译空文件验证头文件自洽（只 `#include` 所有求解器头文件）
- [ ] **步骤 7**：按第 9 节做 FK/IK 验收
- [ ] **步骤 8**：替换为自己的改进 DH 表，再次验收

---

## 7. 编译配置

```cmake
cmake_minimum_required(VERSION 3.10)
project(excavator_kinematics CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

set(KDL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/kdl_kin)

add_library(kdl_kin STATIC
    ${KDL_ROOT}/frames.cpp
    ${KDL_ROOT}/joint.cpp
    ${KDL_ROOT}/segment.cpp
    ${KDL_ROOT}/chain.cpp
    ${KDL_ROOT}/jntarray.cpp
    ${KDL_ROOT}/jacobian.cpp
    ${KDL_ROOT}/rotationalinertia.cpp
    ${KDL_ROOT}/rigidbodyinertia.cpp
    ${KDL_ROOT}/framevel.cpp
    ${KDL_ROOT}/frameacc.cpp
    ${KDL_ROOT}/jntarrayvel.cpp
    ${KDL_ROOT}/jntarrayacc.cpp
    ${KDL_ROOT}/chainfksolverpos_recursive.cpp
    ${KDL_ROOT}/chainjnttojacsolver.cpp
    ${KDL_ROOT}/chainiksolvervel_wdls.cpp
    ${KDL_ROOT}/chainiksolverpos_nr_jl.cpp
    ${KDL_ROOT}/utilities/utility.cxx
    ${KDL_ROOT}/utilities/utility_io.cxx
    ${KDL_ROOT}/utilities/error_stack.cxx
    ${KDL_ROOT}/utilities/svd_eigen_HH.cpp
)

target_include_directories(kdl_kin PUBLIC
    ${KDL_ROOT}
    ${EIGEN3_INCLUDE_DIR}
)
```

若改用上游 CMake 构建完整库，则需要把 `KDL_USE_NEW_TREE_INTERFACE` 设为 `OFF`，
以避免触发 `find_package(Boost REQUIRED)`。

---

## 8. 已知坑位

| # | 坑 | 表现 | 规避 |
|---|---|---|---|
| 1 | `Segment` 强依赖 `RigidBodyInertia` | 漏 `rigidbodyinertia.cpp` 会在链接期报错 | 按清单带全，或把惯性相关符号自行桩化 |
| 2 | `chainfksolver.hpp` / `chainiksolver.hpp` 拖入速度/加速度类型 | 只做位置解也必须带 7 个 framevel/jntarrayvel 文件 | 见第 10 节“方案 B” |
| 3 | `wdls` 依赖 `svd_eigen_HH.cpp` | 编译通过、链接失败 | 必带 `.cpp`，不能只带 `.hpp` |
| 4 | `kdl-config.h` 被误认为是构建产物 | 找不到头文件 | 它是仓库内的静态文件，直接复制 |
| 5 | 遗漏 `.inl` 文件 | 大量“未定义”但签名正确的报错 | `frames.inl`、`framevel.inl`、`frameacc.inl` 必带 |
| 6 | Boost 依赖 | 无谓引入第三方库 | 不移植 `Tree`；CMake 关闭新 Tree 接口 |
| 7 | 编译标准过低 | `std::function` / `std::hash` 报错 | 至少 C++11 |
| 8 | 改进 DH 直接套 `Frame::DH_Craig1989` 当 `f_tip` | `d ≠ 0` 时几何错误且不报错 | 用 `Frame(Rotation::RotX(α), Vector(a,0,d))` |
| 9 | 单位混淆 | 角度/长度量级错误 | KDL 内部一律**弧度 + 米**；接口边界做 deg↔rad |
| 10 | 许可合规 | — | KDL 为 **LGPL-2.1**；静态链接或嵌入闭源产品需评估义务 |

---

## 9. 验收测试

### 9.1 建链正确性

1. 用 3R 平面臂表建链（`L1=3, L2=2.5, L3=1`，改进 DH，`α=0`）。
2. 取 `q = (30°, -60°, 20°)`，调用 `JntToCart`。
3. 期望：`x ≈ 5.748`、`y ≈ 0.0764`、`θz ≈ -10°`。
4. 取 `q = (0,0,0)`，期望末端在 `(L1+L2+L3, 0, 0)`（臂完全伸直）。

### 9.2 逆解正确性

1. 以同一组 `q` 作为 seed，调用 `ChainIkSolverPos_NR_JL::CartToJnt`。
2. 目标位姿用 9.1 的 FK 结果。
3. 期望返回 0，且 `q_out ≈ (30°, -60°, 20°)`。
4. 再用 `q_out` 跑一次 FK，与目标位姿逐元素对比（位置 < 1e-6 m，姿态 < 1e-6 rad）。

### 9.3 链条完整性

- 关节角全零时末端位置 = 各段 `a` 之和；
- 单关节单独 +10° 时，末端变化方向与物理方向一致（检查正方向与 `scale`）；
- 关节限位生效：给定超限目标，`q_out` 不越界。

---

## 10. 可选裁剪（方案 B）

若目标平台 Flash 紧张，可进一步去掉速度/加速度相关文件：

**做法**：自己写精简版接口，只保留位置方法。

```cpp
// 精简版：只保留位置正解
class ChainFkSolverPos {
public:
    virtual int JntToCart(const KDL::JntArray& q, KDL::Frame& p, int segmentNr = -1) = 0;
    virtual ~ChainFkSolverPos() {}
};
```

同时让 `ChainFkSolverPos_recursive`、`ChainIkSolverPos_NR_JL`
继承这套精简接口，即可去掉：

- `framevel.*`、`frameacc.*`、`jntarrayvel.*`、`jntarrayacc.*`
- `utilities/traits.h`、`rall1d.h`、`rall2d.h`
- 若同时不用 WDLS，还可去掉 `svd_eigen_HH.*`

**代价**：改动了上游源码，后续跟进上游版本需要重新适配。

| 方案 | 文件数 | 风险 |
|---|---|---|
| A. 原样移植（推荐） | 约 15 个 `.cpp` + 25 个头文件 | 零修改，行为与上游一致 |
| B. 精简接口 | 少约 8 个文件 | 需维护接口分支 |

---

## 11. 一句话总结

移植范围 = **`frames` → `joint` → `segment` → `chain` → (`jntarray` / `jacobian`) 的基础层**
+ **`solveri` / `chainfksolver` / `chainiksolver` 接口层**
+ **4 个求解器实现**（FK 位置、雅可比、IK 速度 WDLS、IK 位置 NR_JL）
+ **`utilities` 中的数值与错误处理基础**；
外部只需 Eigen3，不需要 Boost。
