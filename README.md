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

### 路径规划
- A* 算法（3D 网格搜索）
- RRT/RRT* 算法（快速随机探索树）
- 5 阶多项式轨迹优化
- 动态避障与重规划

### AI 行为树
- 任务节点：飞往位置、轨迹跟踪、悬停、巡逻
- 服务节点：状态更新、路径规划
- 装饰器：位置检查
- 黑板数据管理

### 调试工具
- 实时 PID 参数调整
- 飞行数据记录（CSV 导出）
- 轨迹与障碍物可视化
- HUD 状态显示

## 项目结构

```
uav_simulator/
├── Source/uav_simulator/
│   ├── Core/           # 核心类（UAVPawn、数据类型）
│   ├── Physics/        # 动力学模型
│   ├── Sensors/        # 传感器模拟
│   ├── Control/        # 飞行控制器
│   ├── Planning/       # 路径规划与轨迹优化
│   ├── AI/             # AI 控制器与行为树节点
│   ├── Debug/          # 调试与可视化工具
│   └── Utility/        # 工具函数
├── Content/
│   ├── UAV/            # UAV 蓝图与 AI 资产
│   └── Environment/    # 关卡与环境
├── Config/             # 项目配置
└── Docs/               # 文档
```

## 快速开始

### 环境要求
- Unreal Engine 5.3+
- Visual Studio 2022
- Windows 10/11

### 编译项目
1. 使用 Unreal Engine 打开 `uav_simulator.uproject`
2. 等待 Shader 编译完成
3. 在编辑器中点击 `Compile` 或使用 Visual Studio 编译

### 运行仿真
1. 打开关卡 `Content/Environment/Levels/UavSimulatorMap`
2. 在关卡中放置 `BP_UAVPawn_Default`
3. 配置目标位置或巡逻点
4. 点击 `Play` 运行仿真

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
```

## 开发路线图

| 阶段 | 内容 | 状态 |
|------|------|------|
| Phase 1 | 基础框架（Actor、动力学、传感器） | 已完成 |
| Phase 2 | 飞行控制（级联控制、EKF、调试工具） | 已完成 |
| Phase 3 | AI 行为树（控制器、任务节点、服务） | 已完成 |
| Phase 4 | 轨迹规划（A*、RRT、轨迹优化、避障） | 已完成 |
| Phase 5 | 多机协同（编队、任务分配、通信） | 计划中 |
| Phase 6 | 任务规划（任务定义、调度、监控） | 计划中 |
| Phase 7 | 环境优化（风场、天气、性能优化） | 计划中 |

## 文档

- [项目路线图](UAV_SIMULATOR_ROADMAP.md)
- [轨迹规划配置指南](Docs/Phase4_TrajectoryPlanning_Setup_Guide.md)
- [行为树使用指南](Source/uav_simulator/AI/README_BehaviorTree.md)

## 许可证

本项目仅供学习和研究使用。
