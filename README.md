# UAV Simulator

基于 Unreal Engine 5 的高保真无人机仿真平台，提供完整的物理模型、飞行控制、AI 行为树和轨迹规划功能。

## 功能特性

### 物理仿真
- 六自由度动力学模型（牛顿-欧拉方程）
- RK4 数值积分器
- 陀螺效应与空气阻力模型
- 电机推力映射

### 传感器模拟
- IMU 传感器（加速度计、陀螺仪）
- GPS 传感器
- 可配置的噪声模型

### 飞行控制
- 级联 PID 控制架构（位置 → 速度 → 姿态 → 推力）
- 扩展卡尔曼滤波器（EKF）状态估计
- 积分抗饱和与重力补偿
- 运行时参数调整

### 路径规划与避障
- A* 算法（3D 网格搜索）
- RRT/RRT* 算法（快速随机探索树）
- 5 阶多项式轨迹优化
- 多航段路径规划（逐段 A* + line-of-sight 精简）
- NMPC 局部避障（非线性模型预测控制）
  - 6-状态点质量模型，投影梯度下降求解器
  - 指数势垒障碍物代价 + 温启动
  - 支持 Sphere/Box/Cylinder 障碍物及动态障碍物预测
- 基于射线检测的障碍物感知（ObstacleDetector）

### AI 行为树
- 任务节点：飞往位置、轨迹跟踪、悬停、巡逻
- 服务节点：状态更新、路径规划
- 装饰器：位置检查
- 黑板数据管理

### 调试工具
- 实时 PID 参数调整
- 飞行数据记录（CSV 导出）
- 轨迹、障碍物与 NMPC 预测轨迹可视化
- HUD 状态显示

## 项目结构

```
uav_simulator/
├── Source/uav_simulator/
│   ├── Core/           # 核心类（UAVPawn、数据类型）
│   ├── Physics/        # 动力学模型
│   ├── Sensors/        # 传感器模拟
│   ├── Control/        # 飞行控制器
│   ├── Planning/       # 路径规划、轨迹优化与 NMPC 避障
│   ├── Mission/        # 任务管理（MissionComponent）
│   ├── AI/             # AI 控制器与行为树节点
│   ├── Debug/          # 调试与可视化工具
│   ├── Tests/          # 单元测试
│   └── Utility/        # 工具函数
├── Content/
│   ├── UAV/            # UAV 蓝图与 AI 资产
│   └── Environment/    # 关卡与环境
├── Script/             # 自动化脚本
├── Config/             # 项目配置
└── Docs/               # 文档
```

## 快速开始

### 环境要求
- Unreal Engine 5.7+
- Visual Studio 2022
- Windows 10/11

### 环境配置
编辑 `Script/env.bat`，将 `UE_ROOT` 修改为你的 UE 安装路径：
```bat
set UE_ROOT=D:\mySoftware\Epic Games\UE_5.7
```
项目路径会自动从脚本位置推导，无需手动设置。

### 编译项目
```bash
Script\build.bat
```
或使用 Unreal Engine 打开 `uav_simulator.uproject`，在编辑器中编译。

### 运行仿真
```bash
Script\sim.bat          # 默认运行 60 秒
Script\sim.bat 30       # 指定运行 30 秒
```
仿真日志保存至 `Logs/uav.log`。

也可在编辑器中打开关卡 `Content/Environment/Levels/UavSimulatorMap`，放置 `BP_UAVPawn_Default` 后点击 Play。

### 配置 AI 行为
1. 在 UAV Pawn 蓝图中设置 AI Controller Class 为 `BP_UAVAIController`
2. 配置黑板键值（TargetLocation、PatrolPoints 等）
3. 行为树将自动启动并控制无人机

## 核心模块

### UAVPawn
无人机 Pawn 类，集成所有子系统：
- `UAVDynamics`: 动力学组件
- `PositionController`: 位置控制器
- `AttitudeController`: 姿态控制器
- `StateEstimator`: 状态估计器
- `PathPlanner`: 路径规划器
- `TrajectoryOptimizer`: 轨迹优化器
- `TrajectoryTracker`: 轨迹跟踪器
- `ObstacleManager`: 障碍物管理器
- `ObstacleDetector`: 障碍物感知组件
- `PlanningVisualizer`: 规划可视化组件
- `MissionComponent`: 任务管理组件

### 控制接口
```cpp
// 设置目标位置
UAVPawn->SetTargetPosition(FVector TargetLocation);

// 设置目标速度
UAVPawn->SetTargetVelocity(FVector TargetVelocity);

// 执行轨迹
UAVPawn->FollowTrajectory(FTrajectory Trajectory);
```

### 规划接口
```cpp
// 路径规划
TArray<FVector> Path = PathPlanner->PlanPath(Start, Goal);

// 轨迹优化
FTrajectory Trajectory = TrajectoryOptimizer->OptimizePath(Path);

// 轨迹跟踪
TrajectoryTracker->StartTracking(Trajectory);

// NMPC 局部避障
FNMPCAvoidanceResult Result = NMPCAvoidance->ComputeAvoidance(
    CurrentPosition, CurrentVelocity, ReferencePoints, Obstacles);
```

## 单元测试

项目使用 UE5 Automation Test Framework 进行单元测试，覆盖以下模块：

| 模块 | 测试数量 | 说明 |
|------|---------|------|
| Control.AttitudeController | 8 | 姿态控制器 PID、角度归一化、抗饱和 |
| Control.PositionController | 9 | 位置控制器、级联控制、推力限制 |
| Control.StateEstimator | 7 | EKF 状态估计、预测收敛、GPS 融合 |
| Mission.MissionComponent | 17 | 任务状态机、航点管理、循环模式 |
| Physics.UAVDynamics | 8 | 动力学模型、RK4 积分、推力计算 |
| Planning.AStar | 8 | A* 路径规划、启发式、避障 |
| Planning.MultiSegment | 4 | 多段路径规划、碰撞检测、路径精简 |
| Planning.NMPCAvoidance | 5 | NMPC 求解、障碍物代价、动态障碍物 |

### 运行测试

```bash
# 使用测试脚本
Script\test.bat

# 或在 UE5 编辑器中
# Window -> Developer Tools -> Session Frontend -> Automation
# 筛选 "UAVSimulator" 并运行
```

## 开发路线图

| 阶段 | 内容 | 状态 |
|------|------|------|
| Phase 1 | 基础框架（Actor、动力学、传感器） | 已完成 |
| Phase 2 | 飞行控制（级联控制、EKF、调试工具） | 已完成 |
| Phase 3 | AI 行为树（控制器、任务节点、服务） | 已完成 |
| Phase 4 | 轨迹规划（A*、RRT、轨迹优化、避障） | 已完成 |
| Phase 5 | 任务管理（MissionComponent、航点管理、任务状态） | 已完成 |
| Phase 6 | 单元测试（UE5 Automation Test Framework） | 已完成 |
| Phase 7 | 路径规划与实时避障（多航段 A*、NMPC 局部避障、障碍物感知） | 已完成 |
| Phase 8 | 多机协同与安全滤波（联合 NMPC、CBF-QP、编队控制） | 计划中 |
| Phase 9 | 任务分配与联合优化（MILP/MIQP/MINLP、联合轨迹优化） | 计划中 |
| Phase 10 | 环境优化（风场、天气、性能优化） | 计划中 |

### 目标架构

集中式联合最优化：任务分配（MILP/MIQP/MINLP）+ 联合轨迹优化/NMPC（带硬约束）+ 多机 CBF-QP 安全滤波

```
┌─────────────────────────────────────────────────┐
│         任务分配层 (MILP/MIQP/MINLP)            │
│  决策变量: 任务-UAV 分配矩阵、执行顺序           │
│  约束: 能力、载荷、续航、时间窗口                 │
├─────────────────────────────────────────────────┤
│      联合轨迹优化 / 多机 NMPC（带硬约束）         │
│  联合状态空间、机间安全距离、编队保持              │
├─────────────────────────────────────────────────┤
│          多机 CBF-QP 安全滤波                    │
│  Control Barrier Function 定义安全集              │
│  QP 实时滤波保证前向不变性                        │
├─────────────────────────────────────────────────┤
│          单机 NMPC 局部避障                       │
└─────────────────────────────────────────────────┘
```

## 文档

- [clangd配置方法.md](Docs/clangd配置方法.md)

## 许可证

本项目仅供学习和研究使用。
