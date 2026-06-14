# UAV Simulator

基于 Unreal Engine 5 的高保真无人机仿真平台，提供完整的物理模型、飞行控制、AI 行为树和轨迹规划功能。

## 功能特性

### 物理仿真
- 六自由度动力学模型（牛顿-欧拉方程）
- RK4 数值积分器
- 电机一阶滞后动力学
- 陀螺效应与空气阻力模型（线性 + 二次）
- 炸机后生成 UE Chaos 物理残骸，使用复合碰撞体接管坠落与接触阻挡，并停止飞行期评分/规划更新

### 传感器模拟
- IMU 传感器（加速度计、陀螺仪，Box-Muller 高斯噪声 + 偏置建模）
- GPS 传感器（位置 + 速度测量，1m/0.1m/s 标准差）
- 气压计传感器（ISA 大气模型，50Hz）
- 磁力计传感器（地磁场→机体坐标系，100Hz）
- 风速计传感器（读取风场数据，20Hz）
- 可配置的噪声模型（Box-Muller 高斯噪声）

### 飞行控制
- 级联 PID 控制架构（位置 → 速度 → 姿态 → 推力）
- NMPC 直接控制层（绕过位置 PID，输出 u*[0]）
- 姿态控制器优化：
  - **前馈控制**：基于期望角加速度，减少PID反馈延迟，响应速度提升20-30%
  - **自适应控制**：MIT规则在线估计模型误差和外部扰动，稳态误差降低50-70%
- 线性化MPC（LinearMPC）：
  - 状态空间线性化，QP求解器（投影梯度下降 + 解析梯度）
  - 计算量降低50-70%，支持配置切换NMPC/LinearMPC
- 扩展卡尔曼滤波器（EKF）状态估计（6状态，IMU + GPS 融合）
- 积分抗饱和与重力补偿
- 运行时参数调整

#### 控制系统架构

三种控制模式：`Attitude`（直接姿态 PID）、`Position`（级联位置 PID）、`Trajectory`（NMPC 全程接管）。以下为 Trajectory 模式完整流水线：

```
┌─────────────────────────────────────────────────────────────────┐
│                    Trajectory 控制模式                           │
│                 (NMPC 全程接管，Tick 子步 50Hz)                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              轨迹跟踪器 (TrajectoryTracker)                      │
│  • 时间参数化轨迹回放                                             │
│  • 自适应时间尺度（跟踪误差大时自动减速/暂停）                   │
│  • 障碍物距离速度缩放                                             │
│  输出：参考轨迹点 (N+1 个位置)                                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              MPC 规划层 (可配置切换，20Hz)                       │
│  ┌──────────────────────────┐  ┌──────────────────────────┐    │
│  │     NMPC (Nonlinear)     │  │   LinearMPC (Linear)     │    │
│  │  • 6状态点质量模型       │  │  • 状态空间线性化         │    │
│  │  • PGD + 回溯线搜索      │  │  • 解析梯度 + QP 求解     │    │
│  │  • 7 候选多初值求解      │  │  • 计算量：-50~70%        │    │
│  │  • Homotopy 绕行侧记忆   │  │                          │    │
│  │  • Dykstra 投影 (加速+速度)│ │                          │    │
│  │  • 振荡/位置卡死检测      │  │                          │    │
│  └──────────────────────────┘  └──────────────────────────┘    │
│  输出：最优加速度 u*[0] = [ax, ay, az]                         │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              后处理管道                                          │
│  • EMA 平滑（新旧解 0.4/0.6 混合）                             │
│  • 低速补偿（v < 6 m/s 且控制量不足时注入最小加速度）           │
│  • 偏差保护（横向偏差 > 2m 时 PD 修正，> 0.8m 硬限制强制纠偏）  │
│  • 偏差感知速度钳位（偏差大时限制前向速度，防止飞过纠偏范围）    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              CBF-QP 安全滤波 (统一)                           │
│  ┌────────────────────────────────────────────────────────┐    │
│  │  机间 HOCBF: h = ‖pi−pj‖² − DSafe²                   │    │
│  │  静态障碍 HOCBF: 基于符号距离梯度 ∇d(p)                │    │
│  │  Active-Set QP + Warm Start + Slack 变量               │    │
│  │  速度包络制动意图 + 倾角可执行加速度域约束                │    │
│  └────────────────────────────────────────────────────────┘    │
│  模式：Active（替换加速度）│ ShadowLog（仅记录）│ Disabled      │
│  输出：安全加速度 SafeAcceleration                               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              位置控制器 (AccelerationToControl)                  │
│  • 加速度 → 推力向量 T = [ax, ay, g+az] → yaw 对齐的快速限速姿态 + 推力 │
│  • 重力补偿 + 倾斜角限制 (≤30°)                                 │
│  • Mellinger-Kumar 微分平坦度 → 前馈角加速度                    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              姿态控制器                                         │
│  PID 反馈 + 前馈力矩 (I·α·Gain) + 自适应扰动估计 (MIT 规则)    │
│  输出：电机推力 [M0, M1, M2, M3]                                │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    UAV 动力学模型                                │
│  • 六自由度牛顿-欧拉方程 + 电机一阶滞后                         │
│  • RK4 数值积分 + 陀螺效应 + 空气阻力                           │
└─────────────────────────────────────────────────────────────────┘
```

**关键特性**：
- **分层控制**：轨迹跟踪 → MPC 规划 → 后处理 → CBF-QP 安全滤波 → 位置控制 → 姿态控制 → 动力学
- **灵活配置**：NMPC/LinearMPC 运行时切换，CBF-QP 三级模式（Active/ShadowLog/Disabled），前馈/自适应独立开关
- **鲁棒性保障**：EMA 平滑 + 偏差保护 + 硬限制纠偏 + 偏差感知速度钳位 + CBF-QP 安全滤波 + 卡死逃逸


### 产品型号支持
- 农业无人机：AG-20（20L）、AG-60（60L）、AG-100（100L）
- 测绘无人机：SV-Pro（RGB 相机）、SV-LiDAR（激光雷达）
- 通过 `ModelID` UPROPERTY 一键切换型号，自动应用物理/控制参数
- 运行时动态载荷质量更新（`SetPayloadMass`）

### 路径规划与避障
- A* 算法（3D 网格搜索）
- RRT/RRT* 算法（快速随机探索树）
- 5 阶多项式轨迹优化
- 多航段路径规划（逐段 A* + line-of-sight 精简）
- NMPC 局部避障（非线性模型预测控制）
  - 6-状态点质量模型，投影梯度下降求解器
  - Frenet 坐标分解代价函数（横向/纵向误差 + L2 平方距离）
  - Smooth hinge 障碍代价（A/B 对比开关）
  - 确定性多初值 PGD（Warm/Nominal/Left/Right/Up/Down/SlowForward 7 候选 + 短 PGD 精炼）
  - Homotopy 绕行侧记忆（跨帧持久化，迟滞切换）
  - Dykstra 投影（同时满足加速度 + 速度约束）
  - 自适应安全阈值（基于实际净空分布动态调整 BrakeFallback 触发条件）
  - 净空不足制动保留横向避障分量（避免完全丢弃 NMPC 避障输出）
  - 同次优化内失败检测（线搜索失败 / 梯度收敛但净空不足）
  - 振荡/位置卡死检测与逃逸逻辑
  - 嵌套参数结构（Solver/Cost/Obstacle/Actuator/Init 分组）
  - 支持 Sphere/Box/Cylinder 障碍物及动态障碍物预测
- 基于射线检测的障碍物感知（ObstacleDetector）与命名静态障碍物启动注册

### AI 行为树
- 任务节点：飞往位置、轨迹跟踪、悬停、巡逻
- 服务节点：状态更新、路径规划
- 装饰器：位置检查
- 黑板数据管理

### 多机协同
- Agent 管理：注册、状态缓存、邻居检测
- 编队控制：线形、V形、环形、菱形编队，平滑过渡动画
- 联合 NMPC：Leader 统一求解（6N状态/3N控制，解耦动力学 + 耦合代价）
- 任务分配：MILP 求解器（Branch & Bound + Revised Simplex LP 松弛）优化任务分配
- 任务监控：实时监控，停滞/偏差/超时检测，委托触发重规划
- Agent 通信：通过 AgentManager 的理想零延迟/零丢包模型（架构预留延迟/丢包模拟）
- CBF-QP 安全滤波：
  - 统一 HOCBF 安全层（静态障碍 + 机间）
  - 可行初值 Primal Active-Set QP 求解器 + Warm Start
  - 速度包络主动制动约束
  - 倾角可执行加速度域约束
  - Slack 变量保证可行性，超大 slack 触发保守降级制动
  - Shadow mode 迁移

### 任务管理
- 航点 CRUD 操作
- 任务控制：启动、停止、暂停、恢复、重置
- 状态机：Idle → Ready → Running → Paused → Completed/Failed
- 循环模式（Once/Loop/PingPong）
- 航点悬停时长
- 丰富的生命周期事件委托
- 农业应用支持（条带间距、喷洒流量）
- 测绘应用支持（重叠率、相机触发间隔）

### 调试工具
- 实时 PID 参数调整
- 飞行数据记录（CSV 导出）
- 轨迹、障碍物与 NMPC 预测轨迹可视化
- HUD 状态显示
- 飞行稳定性评分（0-100，EMA 平滑）
- 分模块日志等级控制
- 结构化仿真指标日志（速度、姿态、偏差、NMPC/CBF 求解、安全失败汇总）

### 场景资产化系统
- 声明式仿真场景（`UScenario` DataAsset）：障碍布局、风场、机队、任务航点、验收标准、随机种子
- 组合引用式资产：`UScenario` 外壳引用 5 个可复用子资产（`UObstacleLayout`/`UWindProfile`/`UFleetSetup`/`UMissionProfile`/`UAcceptanceCriteria`）
- 场景装配器（`UScenarioLoader`）：运行时按声明内容 Spawn 机队、注册障碍、配置风场、下发任务
- 场景验收器（`UScenarioEvaluator`）：周期快照指标，对照验收标准判定 PASS/FAIL，输出 `scenario_result.json`（不依赖进程正常退出，pkill 强杀时留最近一次快照）；fail-closed——场景缺失验收标准时直接判 FAIL，避免任务失败被误报为 PASS
- 命令行驱动：`-Scenario=<资产路径>` 指定运行场景，`sim.sh` 据退出码（PASS=0/FAIL=1/缺失=2）供 CI 判定
- 风场为场景级单例（挂在 `AMultiAgentGameMode`，全关卡共享）

### 视角切换
- UMG 按钮面板：屏幕左下角显示视角切换按钮
- 全局视角：自由观察模式（默认）
- 无人机跟随视角：第三人称相机（SpringArm + CameraComponent）
- 平滑视角过渡：0.5秒 blend 切换
- 动态按钮列表：根据无人机数量自动更新

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
│   ├── MultiAgent/     # 多机协同（AgentManager、CBF-QP、编队控制）
│   ├── Environment/    # 环境模拟（风场、天气）
│   ├── Debug/          # 调试与可视化工具
│   ├── UI/             # 用户界面（视角切换 Widget）
│   ├── Utility/        # 工具函数
│   └── Tests/          # 单元测试
├── Content/
│   ├── UAV/            # UAV 蓝图与 AI 资产
│   └── Environment/    # 关卡与环境
├── Script/             # 自动化脚本
├── Docs/               # 文档
├── Config/             # 项目配置
└── Plugins/            # 插件
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
Script\sim.bat                          # 默认运行 60 秒
Script\sim.bat 30                       # 指定运行 30 秒
Script\sim.bat 30 8 "/Game/Scenarios/S" # 指定场景资产（可选第 3 参数）
```
仿真日志保存至 `Logs/uav.log`。传入场景资产时，进程退出码反映判决：`0`=PASS、`1`=FAIL、`2`=结果缺失。

也可在编辑器中打开关卡 `Content/Environment/Levels/UavSimulatorMap`，放置 `BP_UAVPawn_Default` 后点击 Play。

### 配置 AI 行为
1. 在 UAV Pawn 蓝图中设置 AI Controller Class 为 `BP_UAVAIController`
2. 配置黑板键值（TargetLocation、PatrolPoints 等）
3. 行为树将自动启动并控制无人机

## 核心模块

### UAVPawn
无人机 Pawn 类，集成所有子系统：
- `UAVDynamics`: 动力学组件
- `AttitudeController`: 姿态控制器（PID + 前馈 + 自适应）
- `PositionController`: 位置控制器
- `StateEstimator`: 状态估计器（EKF）
- `PathPlanner`: 路径规划器（A*/RRT/RRT*）
- `TrajectoryOptimizer`: 轨迹优化器（Minimum Snap）
- `TrajectoryTracker`: 轨迹跟踪器
- `ObstacleManager`: 障碍物管理器
- `ObstacleDetector`: 障碍物感知组件
- `PlanningVisualizer`: 规划可视化组件
- `MissionComponent`: 任务管理组件
- `NMPCAvoidance`: NMPC 局部避障组件

### 性能分析
内置 `SCOPE_CYCLE_COUNTER` 性能埋点，支持 UE5 `stat UAVSim` 实时查看各模块耗时。日志中每 2 秒输出 `[PERF_SUMMARY]` 帧时间指标。

已覆盖模块：UAVPawn::Tick、物理子步进、RK4积分、推力/力矩计算、NMPC求解、CBF-QP滤波、碰撞检测、传感器更新、控制器更新、轨迹跟踪、Debug绘制等。
- `LinearMPCAvoidance`: 线性 MPC 避障组件（可切换）
- `CBFQPFilter`: CBF-QP 安全滤波组件
- `WindField`: 风场组件
- `StabilityScorer`: 稳定性评分组件
- `ControlParameterTuner`: 参数调优组件

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
| Planning.AStar | 7 | A* 路径规划、启发式、避障 |
| Planning.RRT | 9 | RRT/RRT* 路径规划、采样优化 |
| Planning.MultiSegment | 5 | 多段路径规划、碰撞检测、路径精简 |
| Planning.NMPCAvoidance | 16 | NMPC 求解、障碍物代价、多初值、卡死检测 |
| Planning.NMPCBaseline | 1 | 基准回归场景集（8 个子场景） |
| Planning.ObstacleManager | 11 | 障碍物注册、碰撞检测、动态追踪 |
| Planning.TrajectoryOptimizer | 9 | Minimum Snap 轨迹优化 |
| Planning.TrajectoryTracker | 10 | 轨迹跟踪、自适应时间尺度、完成检测 |
| MultiAgent.CBFQPFilter | 19 | CBF h/hdot、HOCBF、Active-Set QP、slack、多约束 |
| MultiAgent.JointNMPCSolver | 7 | 联合 NMPC 求解、编队代价 |
| MultiAgent.MILPSolver | 8 | MILP Branch & Bound、LP 松弛 |
| MultiAgent.TaskAllocator | 6 | 任务分配、重规划 |
| MultiAgent.TaskMonitor | 6 | 任务监控、停滞/偏差/超时检测 |
| Sensors.IMU | 4 | IMU 加速度计/陀螺仪、噪声 |
| Sensors.GPS | 4 | GPS 位置/速度测量 |
| Sensors.Barometer | 4 | 气压计高度测量 |
| Sensors.Magnetometer | 4 | 磁力计地磁场测量 |
| Sensors.Anemometer | 3 | 风速计测量 |
| Sensors.ObstacleDetector | 3 | 射线检测、聚类、注册 |
| Environment.WindField | 5 | 风场模型（恒定/阵风/Dryden 湍流） |

### 运行测试

```bash
# 使用测试脚本
Script\test.bat

# 或在 UE5 编辑器中
# Window -> Developer Tools -> Session Frontend -> Automation
# 筛选 "UAVSimulator" 并运行
```

## 目标架构

集中式联合最优化：任务分配（MILP）+ 联合轨迹优化（Joint NMPC）+ 单机 CBF-QP 安全滤波

```
┌──────────────────────────────────────────────────────────┐
│             任务分配层 (MILP Solver)                      │
│  • Branch & Bound + Revised Simplex LP 松弛              │
│  • 决策变量：x[i,j] ∈ {0,1} 任务-UAV 分配矩阵           │
│  • 约束：唯一性、续航、载荷、能力匹配                     │
│  • 层级耦合：MILP → Joint NMPC → 反馈迭代 → 重新求解    │
│  • 增量重规划：支持新任务/故障 Agent                      │
├──────────────────────────────────────────────────────────┤
│             任务监控 (TaskMonitor)                        │
│  • 实时进度跟踪、停滞/偏差/超时检测                       │
│  • 委托触发重规划                                        │
├──────────────────────────────────────────────────────────┤
│      联合轨迹优化 / Joint NMPC (Leader 求解，5Hz)        │
│  • 联合状态空间 [p1,v1,...,pN,vN] (6N维)                │
│  • 联合控制 [a1,...,aN] (3N维)                          │
│  • 解耦动力学 + 耦合代价（参考跟踪 + 机间避碰 + 编队）   │
│  • PGD + 回溯线搜索 + Warm Start                         │
│  • 编队类型：Line / V-Shape / Circle / Diamond           │
├──────────────────────────────────────────────────────────┤
│          多机 CBF-QP 安全滤波 (per-agent)                │
│  ┌──────────────────────────────────────────────────┐    │
│  │  统一安全滤波器                                   │    │
│  │  • 机间 HOCBF: h = ‖pi−pj‖² − DSafe²           │    │
│  │  • 静态障碍 HOCBF: 基于 ∇d(p) 符号距离梯度      │    │
│  │  • Active-Set QP + Slack 变量 + Warm Start       │    │
│  │  模式：Active / ShadowLog / Disabled             │    │
│  └──────────────────────────────────────────────────┘    │
├──────────────────────────────────────────────────────────┤
│    单机 MPC 控制层（NMPC/LinearMPC 可切换，20Hz）        │
│  ┌────────────────────────┐  ┌────────────────────────┐ │
│  │     NMPC               │  │    LinearMPC           │ │
│  │  • 6状态非线性优化      │  │  • 状态空间线性化      │ │
│  │  • 7候选多初值 PGD      │  │  • 解析梯度 + QP       │ │
│  │  • Homotopy 记忆       │  │  • 计算量 -50~70%      │ │
│  │  • Dykstra 投影        │  │                        │ │
│  └────────────────────────┘  └────────────────────────┘ │
│  输出：u*[0] → 后处理 → 安全滤波 → 位置控制 → 姿态控制  │
├──────────────────────────────────────────────────────────┤
│    姿态控制器（PID + 前馈 + 自适应）                     │
│  • 前馈控制: I·α 减少延迟，响应速度 +20~30%             │
│  • 自适应控制: MIT 规则补偿扰动，稳态误差 -50~70%       │
├──────────────────────────────────────────────────────────┤
│    Agent 通信层 (AgentCommunicationComponent)            │
│  • 理想零延迟/零丢包模型（架构预留延迟/丢包模拟）        │
│  • 通过 AgentManager 中转                                │
└──────────────────────────────────────────────────────────┘
```

**单机控制详细流水线**（Trajectory 模式）：

```
TrajectoryTracker → NMPC/LinearMPC (20Hz)
    → EMA 平滑 + 低速补偿 + 偏差保护 + 硬限制纠偏 + 偏差感知速度钳位
    → CBF-QP 安全滤波
    → AccelerationToControl (姿态+推力)
    → 前馈角加速度 (Mellinger-Kumar)
    → AttitudeController (PID + Feedforward + Adaptive)
    → UAVDynamics (6-DOF + RK4)
```

## 开发路线图

| 阶段 | 内容 | 状态 |
|------|------|------|
| Phase 1 | 基础框架（Actor、动力学、传感器） | ✅ 已完成 |
| Phase 2 | 飞行控制（级联控制、EKF、调试工具） | ✅ 已完成 |
| Phase 3 | AI 行为树（控制器、任务节点、服务） | ✅ 已完成 |
| Phase 4 | 轨迹规划（A*、RRT、轨迹优化、避障） | ✅ 已完成 |
| Phase 5 | 任务管理（MissionComponent、航点管理、任务状态） | ✅ 已完成 |
| Phase 6 | 单元测试（UE5 Automation Test Framework） | ✅ 已完成 |
| Phase 7 | 路径规划与实时避障（多航段 A*、NMPC 局部避障、障碍物感知） | ✅ 已完成 |
| Phase 8 | NMPC 控制层升级（直接输出 u*[0]，绕过位置 PID） | ✅ 已完成 |
| Phase 9 | 产品化支持（农业 AG-20/60/100、测绘 SV-Pro/LiDAR，型号注册表） | ✅ 已完成 |
| Phase 10 | NMPC 全程接管重构（50Hz 全程控制，消除模式切换） | ✅ 已完成 |
| Phase 11 | 控制器优化与线性MPC（前馈、自适应、LinearMPC） | ✅ 已完成 |
| Phase 12 | 多机协同与安全滤波（联合 NMPC、CBF-QP、编队控制） | ✅ 已完成 |
| Phase 13 | 任务分配与联合优化（MILP 求解器、任务监控与重规划） | ✅ 已完成 |
| Phase 14 | 环境模拟与传感器扩展（风场模型、气压计、磁力计、风速计） | ✅ 已完成 |
| Phase 15 | NMPC/CBF 优化重构（Frenet 代价、多初值 PGD、Homotopy 记忆、Dykstra 投影） | ✅ 已完成 |
| Phase 16 | 安全层重构（统一 HOCBF、Active-Set QP、Shadow 迁移、控制链统一） | ✅ 已完成 |

## 文档

- [clangd配置方法.md](Docs/clangd配置方法.md)
- [PID调参指南](Docs/PID调参指南.md)
- [C++ 历史遗留问题汇总](Docs/C++%20历史遗留问题汇总.md)
- [Houdini Engine for Unreal教程](Docs/Houdini%20Engine%20for%20Unreal教程.md)
- [UE5+Houdini生成山脉教程](Docs/UE5+Houdini生成山脉（带树林、电线杆与电线）完整教程.md)

## 许可证

本项目仅供学习和研究使用。
