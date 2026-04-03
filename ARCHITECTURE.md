# Prometheus 架构文档

## 项目概述

Prometheus 是一套基于 ROS1 (catkin) 和 PX4 的自主无人机开源软件平台，提供控制、规划、感知、通信和集群等功能模块。

## 目录结构

```
Prometheus/
├── Modules/                    # 功能模块（按域分组）
│   ├── core/                   # 核心基础
│   │   └── common/             # 公共消息定义 & 工具头文件
│   │       ├── prometheus_msgs/  # Prometheus 自定义 ROS 消息/服务
│   │       ├── quadrotor_msgs/   # 四旋翼消息定义
│   │       └── include/          # 公共头文件 (math_utils, geometry_utils, printf_utils)
│   ├── control/                # 控制模块
│   │   ├── uav_control/        # 无人机控制节点 (PID/UDE/NE 控制器)
│   │   ├── uav_control_fmt/    # FMT 固件无人机控制
│   │   └── ugv_control/        # 无人地面车辆控制
│   ├── planning/               # 规划模块
│   │   ├── motion_planning/    # 运动规划 (全局/局部/min-snap)
│   │   ├── ego_planner_swarm/  # EGO-Planner 集群避障规划
│   │   └── global_planner_ugv/ # UGV 全局路径规划
│   ├── perception/             # 感知模块
│   │   ├── FAST_LIO/           # 基于 LiDAR 的实时定位与建图
│   │   └── simulator_utils/    # 仿真感知工具
│   ├── communication/          # 通信模块 (地面站桥接)
│   ├── swarm/                  # 集群模块
│   │   ├── swarm_control/      # 集群控制 (submodule)
│   │   ├── swarm_formation/    # 集群编队 (submodule)
│   │   └── searching_pkg/      # 集群搜索与跟踪 (submodule)
│   ├── demo/                   # 示例与实验
│   │   ├── tutorial_demo/      # 教程演示
│   │   ├── future_aircraft/    # 未来飞行器实验
│   │   └── experiment/         # 实验配置与启动文件
│   └── integration/            # 外部集成
│       └── matlab_bridge/      # MATLAB/Simulink 桥接 (submodule)
├── Simulator/                  # 仿真环境
│   ├── gazebo_simulator/       # Gazebo 仿真 (模型/世界/launch)
│   ├── airsim_simulator/       # AirSim 仿真
│   ├── realsense_gazebo_plugin/
│   ├── velodyne_gazebo_plugins/
│   └── livox_laser_gazebo_plugins/
├── scripts/                    # 脚本
│   ├── build/                  # 编译脚本
│   ├── installation/           # 安装脚本
│   └── simulation/             # 仿真启动脚本
├── CMakeLists.txt              # 顶层 catkin workspace 配置
├── Makefile                    # 统一构建入口
└── .github/workflows/          # CI/CD 配置
```

## 模块依赖关系

```
                    ┌─────────────────┐
                    │   core/common   │
                    │ (prometheus_msgs│
                    │  quadrotor_msgs)│
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
              ▼              ▼              ▼
     ┌────────────┐  ┌─────────────┐  ┌──────────┐
     │  control/  │  │  planning/  │  │perception│
     │uav_control │  │motion_plan. │  │ FAST_LIO │
     │ugv_control │  │ego_planner  │  │sim_utils │
     │uav_ctrl_fmt│  │global_ugv   │  └──────────┘
     └──────┬─────┘  └──────┬──────┘
            │               │
            ▼               ▼
     ┌─────────────┐  ┌──────────┐
     │communication│  │  swarm/  │
     │(ground stn) │  │swarm_ctrl│
     └─────────────┘  │formation │
                      │searching │
                      └──────────┘
```

### 编译顺序

1. **core/common** — 必须最先编译（消息定义）
2. **Simulator/** — 仿真插件和模型
3. **control/** — 控制模块
4. **communication/** — 通信模块（依赖 core）
5. **planning/** — 规划模块（依赖 core, control）
6. **perception/** — 感知模块（依赖 core）
7. **swarm/** — 集群模块（依赖 core, control, communication）
8. **demo/** — 示例（依赖 core, control）

## 构建方式

### 使用 Makefile（推荐）

```bash
make help       # 查看所有构建目标
make all        # 构建全部模块
make control    # 仅构建控制模块
make planning   # 仅构建规划模块
make clean      # 清理构建产物
```

### 使用编译脚本

```bash
bash scripts/build/compile_all.sh       # 编译全部
bash scripts/build/compile_control.sh   # 编译控制模块
bash scripts/build/compile_planning.sh  # 编译规划模块
```

## 消息定义

所有自定义 ROS 消息和服务位于 `Modules/core/common/`:

- **prometheus_msgs**: UAVState, UAVCommand, UAVSetup, SwarmCommand, DetectionInfo 等
- **quadrotor_msgs**: PositionCommand, SO3Command 等（用于底层控制）

## 代码风格

项目使用 `.clang-format` 定义 C++ 代码风格，基于 Google 风格并适配 ROS 开发习惯：
- 缩进: 4 空格
- 行宽: 120 字符
- 指针对齐: 左对齐 (`int* ptr`)

格式化代码:
```bash
# 格式化单个文件
clang-format -i path/to/file.cpp

# 格式化整个模块
find Modules/control -name "*.cpp" -o -name "*.h" | xargs clang-format -i
```

## Git 子模块

以下模块作为 Git 子模块管理（源码托管在 Gitee）：

| 模块 | 路径 | 源仓库 |
|------|------|--------|
| swarm_control | Modules/swarm/swarm_control | gitee.com/amovlab1/swarm_control |
| swarm_formation | Modules/swarm/swarm_formation | gitee.com/amovlab1/Swarm-Formation |
| searching_pkg | Modules/swarm/searching_pkg | gitee.com/amovlab1/swarm-sch-track |
| matlab_bridge | Modules/integration/matlab_bridge | gitee.com/amovlab1/matlab_bridge |

克隆时初始化子模块:
```bash
git clone --recursive https://github.com/amov-lab/Prometheus.git
```
