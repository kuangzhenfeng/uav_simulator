# UAV Simulator 开发路线图

## 项目概述
基于 Unreal Engine 5.7 的无人机模拟器，用于调试轨迹规划算法、飞行控制、多机协同和任务规划。

## 核心模块架构

### 1. 无人机物理模型 (UAV Physics)
- **四旋翼动力学模型**：实现基于牛顿-欧拉方程的六自由度动力学
- **电机模型**：螺旋桨推力、扭矩、电机响应延迟
- **传感器模拟**：IMU、GPS、气压计、磁力计、视觉传感器
- **环境干扰**：风场、气流扰动、传感器噪声

### 2. 飞行控制系统 (Flight Controller)
- **姿态控制**：PID/LQR/MPC 控制器
- **位置控制**：级联控制架构
- **控制分配**：电机输出映射
- **状态估计**：EKF/UKF 融合算法

### 3. 轨迹规划 (Trajectory Planning)
- **路径规划算法**：A*、RRT、RRT*、快速行进法
- **轨迹优化**：最小snap轨迹、B样条轨迹
- **避障规划**：动态窗口法、人工势场法
- **轨迹跟踪**：前馈+反馈控制

### 4. 多机协同 (Multi-Agent Coordination)
- **编队控制**：领航-跟随、虚拟结构、行为基
- **任务分配**：拍卖算法、匈牙利算法
- **通信模拟**：延迟、丢包、带宽限制
- **碰撞避免**：分布式避碰算法

### 5. 任务规划 (Mission Planning)
- **任务定义**：航点、区域覆盖、目标跟踪
- **任务调度**：时序约束、资源分配
- **重规划**：动态环境响应
- **任务监控**：执行状态、异常处理

### 6. 环境系统 (Environment)
- **地形生成**：高程图、障碍物
- **天气系统**：风场、光照、能见度
- **动态对象**：移动目标、其他飞行器
- **地理参考**：真实坐标系统（已启用 GeoReferencing 插件）

### 7. 可视化与调试 (Visualization & Debug)
- **实时数据显示**：状态、轨迹、传感器数据
- **3D可视化**：无人机模型、轨迹线、传感器视锥
- **性能分析**：帧率、计算耗时
- **日志记录**：飞行数据、事件记录

## 开发阶段

### Phase 1: 基础框架 ✅ (已完成)
- [x] 创建 UAV Actor 基类 (Core/UAVActor.h/cpp)
- [x] 实现基础物理模型（简化动力学）(Physics/UAVDynamics.h/cpp)
- [x] 添加基础传感器（IMU、GPS）(Sensors/IMUSensor.h/cpp, GPSSensor.h/cpp)
- [x] 实现简单的姿态控制器 (Control/AttitudeController.h/cpp)
- [x] 创建测试场景和基础UI (Debug/DebugVisualizer, UAVHUD, UAVTestGameMode)

### Phase 2: 飞行控制 ✅ (已完成)
- [x] 完善六自由度动力学模型
  - RK4 积分器提高数值精度和稳定性
  - 添加陀螺效应（完整欧拉动力学方程）
  - 改进空气阻力模型（二次阻力项）
  - 改进 Yaw 力矩模型（基于物理参数）
  - 新增文件：`Physics/UAVDynamics.h/cpp`（已更新）
- [x] 实现完整的姿态和位置控制器
  - 级联控制架构（位置→速度→姿态→推力）
  - 控制限制（最大速度、倾斜角、推力范围）
  - 积分抗饱和和重力补偿
  - 新增文件：`Control/PositionController.h/cpp`
- [x] 添加状态估计器（EKF）
  - 扩展卡尔曼滤波器实现
  - IMU 和 GPS 数据融合
  - 简化对角协方差矩阵（实时性能优化）
  - 新增文件：`Control/StateEstimator.h/cpp`
- [x] 实现控制参数调试界面
  - 运行时 PID 参数调整
  - 参数保存/加载到 INI 配置文件
  - 重置为默认值功能
  - Blueprint 可调用接口
  - 新增文件：`Debug/ControlParameterTuner.h/cpp`
- [x] 添加飞行数据记录功能
  - CSV 格式导出（位置、速度、姿态、电机推力）
  - 可配置记录频率（默认 100Hz）
  - 自动导出和数据管理
  - 新增文件：`Debug/DataLogger.h/cpp`

### Phase 3: AI行为树系统 ✅ (已完成)
- [x] 添加AI模块依赖
  - 更新 `uav_simulator.Build.cs` 添加 AIModule、GameplayTasks、NavigationSystem
- [x] 创建UAVPawn类（支持AI控制）
  - 继承自APawn，支持AIController
  - 保留UAVActor的所有功能
  - 新增文件：`Core/UAVPawn.h/cpp`
- [x] 实现UAV AI控制器
  - 行为树和黑板组件管理
  - 自动启动行为树
  - 黑板键值设置接口
  - 新增文件：`AI/UAVAIController.h/cpp`
- [x] 实现行为树任务节点
  - **BTTask_UAVFlyToLocation**: 飞往目标位置
  - **BTTask_UAVHover**: 悬停任务（支持定时和无限悬停）
  - **BTTask_UAVPatrol**: 巡逻任务（多航点循环）
  - 新增文件：`AI/Tasks/BTTask_UAVFlyToLocation.h/cpp`
  - 新增文件：`AI/Tasks/BTTask_UAVHover.h/cpp`
  - 新增文件：`AI/Tasks/BTTask_UAVPatrol.h/cpp`
- [x] 实现行为树服务节点
  - **BTService_UAVUpdateState**: 定期更新UAV状态到黑板
  - 新增文件：`AI/Services/BTService_UAVUpdateState.h/cpp`
- [x] 实现行为树装饰器
  - **BTDecorator_UAVAtLocation**: 检查是否到达目标位置
  - 新增文件：`AI/Decorators/BTDecorator_UAVAtLocation.h/cpp`
- [x] 添加使用文档
  - 详细的行为树配置指南
  - 黑板键值说明
  - 示例行为树结构
  - 新增文件：`AI/README_BehaviorTree.md`

### Phase 4: 轨迹规划 ✅ (已完成)
- [x] 实现基础路径规划算法
  - **A*算法**: 3D网格化搜索、对角线移动、障碍物膨胀处理
  - **RRT算法**: 快速随机探索树、目标偏置采样、可选RRT*优化
  - 新增文件：`Planning/PathPlanner.h/cpp`（基类）
  - 新增文件：`Planning/AStarPathPlanner.h/cpp`
  - 新增文件：`Planning/RRTPathPlanner.h/cpp`
- [x] 实现轨迹优化（最小snap）
  - 5阶多项式轨迹生成
  - 梯形速度规划时间分配
  - 速度/加速度约束
  - 新增文件：`Planning/TrajectoryOptimizer.h/cpp`
- [x] 添加轨迹跟踪控制器
  - 时间参数化轨迹跟踪
  - 前馈+反馈控制架构
  - 暂停/恢复/进度回调
  - 新增文件：`Planning/TrajectoryTracker.h/cpp`
  - 更新文件：`Control/PositionController.h/cpp`（加速度前馈支持）
- [x] 实现障碍物管理与动态避障
  - 障碍物注册、查询、碰撞检测
  - 支持球体、盒体、圆柱体障碍物
  - 动态障碍物跟踪
  - 新增文件：`Planning/ObstacleManager.h/cpp`
- [x] 可视化规划结果
  - 规划路径、优化轨迹、障碍物可视化
  - 跟踪状态实时显示
  - 新增文件：`Planning/PlanningVisualizer.h/cpp`
  - 更新文件：`Debug/DebugVisualizer.h/cpp`
- [x] 行为树集成
  - **BTTask_UAVFollowTrajectory**: 轨迹跟踪任务
  - **BTService_UAVPathPlanning**: 路径规划服务（动态避障）
  - 更新 BTTask_UAVFlyToLocation：添加路径规划选项
  - 新增文件：`AI/Tasks/BTTask_UAVFollowTrajectory.h/cpp`
  - 新增文件：`AI/Services/BTService_UAVPathPlanning.h/cpp`
- [x] UAVPawn集成
  - 添加 `EUAVControlMode` 控制模式枚举（Attitude/Position/Trajectory）
  - 集成 TrajectoryTracker、ObstacleManager、PlanningVisualizer 组件
  - 更新文件：`Core/UAVPawn.h/cpp`
  - 更新文件：`Core/UAVTypes.h`（添加轨迹相关数据结构）

### Phase 5: 任务管理 ✅ (已完成)
- [x] 创建 MissionComponent 任务管理组件
  - 统一管理航点数据、任务状态和任务配置
  - 替代分散的 WaypointsData 类
  - 新增文件：`Mission/MissionTypes.h`（任务数据结构）
  - 新增文件：`Mission/MissionComponent.h/cpp`
- [x] 任务数据结构
  - **EMissionState**: 任务状态枚举（Idle/Ready/Running/Paused/Completed/Failed）
  - **EMissionMode**: 任务模式枚举（Once/Loop/PingPong）
  - **FMissionWaypoint**: 航点数据结构（位置、悬停时间、期望速度、偏航角、标签）
  - **FMissionConfig**: 任务配置（模式、速度、加速度、阈值、循环次数等）
- [x] 航点管理接口
  - `SetWaypoints()` / `SetMissionWaypoints()`: 设置航点
  - `AddWaypoint()` / `RemoveWaypoint()` / `ClearWaypoints()`: 航点增删
  - `GetWaypointPositions()` / `GetCurrentWaypoint()`: 航点查询
- [x] 任务控制接口
  - `StartMission()` / `StopMission()`: 启动/停止任务
  - `PauseMission()` / `ResumeMission()`: 暂停/恢复任务
  - `AdvanceToNextWaypoint()` / `GoToWaypoint()`: 航点导航
- [x] 事件委托
  - `OnMissionStarted` / `OnMissionCompleted` / `OnMissionFailed`
  - `OnWaypointReached` / `OnMissionStateChanged`
  - `OnMissionPaused` / `OnMissionResumed`
- [x] UAVPawn 集成
  - 添加 MissionComponent 成员
  - 废弃旧的 Waypoints 属性（转发到 MissionComponent）
  - 更新文件：`Core/UAVPawn.h/cpp`
- [x] 行为树节点更新
  - BTService_UAVPathPlanning: 从 MissionComponent 获取航点
  - BTTask_UAVFollowTrajectory: 添加 bUseMissionComponent 选项
  - 更新文件：`AI/Services/BTService_UAVPathPlanning.h/cpp`
  - 更新文件：`AI/Tasks/BTTask_UAVFollowTrajectory.h/cpp`
- [x] 清理冗余代码
  - 删除 `Planning/WaypointsData.h/cpp`
  - 新增日志类别：`LogUAVMission`

### Phase 6: 单元测试 ✅ (已完成)
- [x] 配置测试框架
  - 更新 `uav_simulator.Build.cs` 添加测试模块依赖
  - 创建测试目录结构 `Source/uav_simulator/Tests/`
- [x] 创建测试通用工具
  - 新增文件：`Tests/UAVTestCommon.h`（测试分类常量、浮点数/向量近似相等宏、辅助函数）
- [x] Planning 模块测试（优先级 1）
  - 新增文件：`Tests/Planning/ObstacleManagerTest.cpp`（碰撞检测、距离计算、范围查询）
  - 新增文件：`Tests/Planning/AStarPathPlannerTest.cpp`（坐标转换、启发式计算、邻居生成）
  - 新增文件：`Tests/Planning/RRTPathPlannerTest.cpp`（Steer、最近节点、邻域查找）
  - 新增文件：`Tests/Planning/TrajectoryOptimizerTest.cpp`（时间分配、多项式评估、轨迹采样）
  - 新增文件：`Tests/Planning/TrajectoryTrackerTest.cpp`（状态机、进度计算、轨迹插值）
- [x] Mission 模块测试（优先级 2）
  - 新增文件：`Tests/Mission/MissionComponentTest.cpp`（航点管理、任务状态机、循环逻辑）
- [x] Core 模块测试（优先级 3）
  - 新增文件：`Tests/Core/UAVTypesTest.cpp`（FTrajectory、FObstacleInfo、枚举验证）
- [x] Control 模块测试（优先级 4）
  - 新增文件：`Tests/Control/AttitudeControllerTest.cpp`（PID 计算、输出限制）
  - 新增文件：`Tests/Control/PositionControllerTest.cpp`（级联控制、速度限制）
- [x] Physics 模块测试（优先级 5）
  - 新增文件：`Tests/Physics/UAVDynamicsTest.cpp`（RK4 积分、推力计算）
- [x] 自动化测试脚本
  - 新增文件：`Script/test.bat`（自动运行测试并统计结果）
  - 支持 UTF-8 中文输出
  - 使用 PowerShell 进行结果统计
- [x] 测试覆盖统计
  - 总计 **98 个测试用例**，全部通过
  - Control 模块：21 个测试
  - Core 模块：9 个测试
  - Mission 模块：15 个测试
  - Physics 模块：11 个测试
  - Planning 模块：42 个测试

### Phase 7: 路径规划与实时避障 ✅ (已完成)
- [x] Global Planner（全局路径规划）
  - 改写 `ProcessPresetWaypoints`：逐段 A* 避障 + 全局路径精简 + 轨迹优化
  - 新增 `PlanMultiSegmentPath()` 方法：多航段规划、拼接、全局 line-of-sight 精简
  - 新增 `CreatePathPlanner()` 方法：提取规划器创建逻辑，消除重复代码
  - 更新文件：`AI/Services/BTService_UAVPathPlanning.h/cpp`
- [x] Local Planner（局部实时避障）
  - 实现 NMPC（非线性模型预测控制）局部避障器
  - 6-状态点质量模型: `x = [px, py, pz, vx, vy, vz]`，控制向量: `u = [ax, ay, az]`
  - 代价函数: 参考跟踪 + 速度跟踪 + 控制代价 + 指数势垒障碍物代价 + 终端代价
  - 投影梯度下降求解器 + 有限差分梯度 + 回溯线搜索 + 温启动
  - 从 TrajectoryTracker 采样参考轨迹点，支持动态障碍物预测
  - 仅当 Local Planner 连续失败时触发 Global Replan
  - 新增文件：`Planning/NMPCAvoidance.h/cpp`
  - 更新文件：`AI/Services/BTService_UAVPathPlanning.h/cpp`
- [x] 感知模拟
  - 基于 UE5 射线检测（Raycast）模拟雷达/激光雷达扫描
  - 区分已知障碍物（ObstacleManager 预注册）和未知障碍物（实时感知）
  - 可配置扫描参数：扫描范围、角度分辨率、更新频率
  - 新增文件：`Sensors/ObstacleDetector.h/cpp`
  - 更新文件：`Planning/ObstacleManager.h/cpp`（支持动态添加感知障碍物）
- [x] 可视化与调试
  - 可视化 Global Path（全局规划路径）和 NMPC 预测轨迹（局部避障）
  - 可视化感知范围和检测到的障碍物
  - 更新文件：`Planning/PlanningVisualizer.h/cpp`
- [x] 单元测试
  - NMPCAvoidance 测试：前向仿真、代价计算、梯度下降、障碍物避障、温启动、Stuck检测（13 个测试）
  - PlanMultiSegmentPath 测试：多段规划、失败回退、路径精简（5 个测试）
  - ObstacleDetector 测试：射线检测、障碍物分类（5 个测试）
  - 新增文件：`Tests/Planning/NMPCAvoidanceTest.cpp`
  - 新增文件：`Tests/Planning/MultiSegmentPlanningTest.cpp`
  - 新增文件：`Tests/Sensors/ObstacleDetectorTest.cpp`

### Phase 8: NMPC 控制层升级（直接输出 u*[0]）✅ (已完成)
- [x] NMPC 从规划层升级为控制层，直接输出最优加速度 u*[0] → 姿态+推力，绕过位置 PID
  - `FNMPCAvoidanceResult.OptimalAcceleration`：存储第一步最优控制量
  - `UPositionController::AccelerationToControl()`：加速度→姿态+推力转换
  - `AUAVPawn::SetNMPCAcceleration()` / `ClearNMPCAcceleration()`：直接控制接口
  - `BTService_UAVPathPlanning`：避障时调用 `SetNMPCAcceleration()` 替代 Tracker Override

### Phase 9: 产品化支持（农业无人机 & 测绘无人机）✅ (已完成)
- [x] 产品类型与型号定义
  - 新增 `EUAVProductType` 枚举：`Agricultural`、`Mapping`
  - 新增 `EUAVModelID` 枚举：5 个型号（AG-10/AG-20/AG-40/SV-Pro/SV-LiDAR）
  - 新增 `FUAVModelSpec` 结构体：物理参数、控制参数、载荷元数据
  - 新增文件：`Core/UAVProductTypes.h`
- [x] 农业无人机产品线（3 款）
  - AG-20：12kg，20L 喷洒，喷幅 6m，MaxVel 1200cm/s
  - AG-60：22kg，60L 喷洒，喷幅 10m，MaxVel 800cm/s
  - AG-100：35kg，100L 喷洒，喷幅 14m，MaxVel 600cm/s
- [x] 测绘无人机产品线（2 款）
  - SV-Pro：3.2kg，5000万像素 RGB 相机，GSD 2cm@100m，MaxVel 2000cm/s
  - SV-LiDAR：4.5kg，Livox Mid-360 激光雷达，精度 ±2cm，MaxVel 1800cm/s
- [x] 型号注册表与参数自动应用
  - `FUAVProductManager::GetModelSpec()`：静态型号查询
  - `AUAVPawn::BeginPlay()` 自动按 `ModelID` 写入 UAVDynamics/AttitudeController/PositionController
  - `AUAVPawn::SetPayloadMass()`：运行时动态更新载荷质量（同步 Mass + HoverThrust）
  - 新增文件：`Core/UAVProductManager.h`
  - 更新文件：`Core/UAVPawn.h/cpp`、`Physics/UAVDynamics.h`
- [x] MissionComponent 扩展
  - `FMissionConfig` 新增农业参数：`StripSpacingM`（条带间距）、`SprayFlowLPerMin`（喷洒流量）
  - `FMissionConfig` 新增测绘参数：`OverlapRatio`（重叠率）、`CameraTriggerIntervalM`（触发间距）
  - 更新文件：`Mission/MissionTypes.h`

### Phase 10: 多机协同与安全滤波
- [ ] 集中式联合轨迹优化
  - 多机 NMPC：将单机 NMPC 扩展为联合状态空间，同时优化所有 UAV 的控制序列
  - 硬约束：机间最小安全距离、通信范围、动力学可行性
  - 联合代价函数：任务目标 + 编队保持 + 能耗均衡
- [ ] CBF-QP 安全滤波
  - Control Barrier Function (CBF) 定义机间安全集
  - QP (Quadratic Programming) 实时滤波：在 NMPC 输出上叠加安全约束
  - 保证前向不变性：任意时刻满足安全距离约束
- [ ] 通信模拟
  - 实现多机状态广播与邻居发现
  - 模拟通信延迟、丢包、带宽限制
  - 支持集中式（地面站）和分布式（机间）通信拓扑
- [ ] 编队控制
  - 基于联合 NMPC 的编队保持与队形变换
  - 支持预定义队形：线形、V 形、环形
  - 动态队形切换与障碍物穿越

### Phase 11: 任务分配与联合优化
- [ ] 任务分配（MILP/MIQP/MINLP）
  - Mixed-Integer 优化：将任务分配建模为 MILP/MIQP/MINLP 问题
  - 决策变量：任务-UAV 分配矩阵、任务执行顺序
  - 约束：UAV 能力、载荷、续航、时间窗口
  - 目标：最小化总任务完成时间 / 最大化覆盖率 / 能耗均衡
- [ ] 联合轨迹优化与任务分配
  - 将任务分配与轨迹优化耦合为统一优化问题
  - 分层求解：上层 MILP 分配 → 下层联合 NMPC 轨迹优化
  - 迭代反馈：轨迹代价反馈给任务分配层进行重优化
- [ ] 任务监控与重规划
  - 实时任务进度监控与异常检测
  - 动态重分配：UAV 故障、新任务插入、环境变化时触发重规划
  - 支持优先级调度和抢占式任务切换

### Phase 12: 环境与优化
- [ ] 完善环境系统（风场、天气）
- [ ] 添加复杂场景（城市、森林）
- [ ] 性能优化
- [ ] 添加更多传感器（相机、激光雷达）
- [ ] 完善文档和示例

## 技术栈

### C++ 模块
- **Core**: 核心数据结构和算法
- **Physics**: 物理模拟和动力学
- **Control**: 控制算法
- **Planning**: 规划算法
- **Sensors**: 传感器模拟
- **Communication**: 多机通信
- **Tests**: 单元测试（UE5 Automation Test Framework）

### Blueprint 集成
- UI界面
- 场景配置
- 快速原型测试

### 第三方库（可选）
- Eigen: 线性代数
- OMPL: 运动规划
- ROS2 Bridge: 与ROS2集成（可选）

## 项目结构建议

```
Source/uav_simulator/
├── Core/
│   ├── UAVActor.h/cpp              # 无人机Actor基类
│   ├── UAVPawn.h/cpp               # 无人机Pawn类（支持AI控制）
│   ├── UAVTypes.h                  # 数据类型定义
│   └── UAVMath.h/cpp               # 数学工具
├── Physics/
│   ├── UAVDynamics.h/cpp           # 动力学模型
│   ├── MotorModel.h/cpp            # 电机模型
│   └── EnvironmentForces.h/cpp     # 环境力（风等）
├── Sensors/
│   ├── SensorBase.h/cpp            # 传感器基类
│   ├── IMUSensor.h/cpp             # IMU
│   ├── GPSSensor.h/cpp             # GPS
│   └── CameraSensor.h/cpp          # 相机
├── Control/
│   ├── ControllerBase.h/cpp        # 控制器基类
│   ├── AttitudeController.h/cpp    # 姿态控制
│   ├── PositionController.h/cpp    # 位置控制
│   └── StateEstimator.h/cpp        # 状态估计
├── Planning/
│   ├── PathPlanner.h/cpp           # 路径规划基类
│   ├── AStarPathPlanner.h/cpp      # A*路径规划
│   ├── RRTPathPlanner.h/cpp        # RRT路径规划
│   ├── TrajectoryOptimizer.h/cpp   # 轨迹优化
│   ├── TrajectoryTracker.h/cpp     # 轨迹跟踪
│   ├── ObstacleManager.h/cpp       # 障碍物管理
│   ├── PlanningVisualizer.h/cpp    # 规划可视化
│   └── NMPCAvoidance.h/cpp         # NMPC 局部避障
├── Mission/
│   ├── MissionTypes.h              # 任务数据结构
│   └── MissionComponent.h/cpp      # 任务管理组件
├── MultiAgent/
│   ├── FormationController.h/cpp   # 编队控制
│   ├── TaskAllocator.h/cpp         # 任务分配
│   └── CommunicationManager.h/cpp  # 通信管理
├── Mission/
│   ├── MissionPlanner.h/cpp        # 任务规划
│   ├── MissionExecutor.h/cpp       # 任务执行
│   └── MissionMonitor.h/cpp        # 任务监控
├── AI/
│   ├── UAVAIController.h/cpp       # AI控制器
│   ├── Tasks/
│   │   ├── BTTask_UAVFlyToLocation.h/cpp    # 飞往位置任务
│   │   ├── BTTask_UAVFollowTrajectory.h/cpp # 轨迹跟踪任务
│   │   ├── BTTask_UAVHover.h/cpp            # 悬停任务
│   │   └── BTTask_UAVPatrol.h/cpp           # 巡逻任务
│   ├── Services/
│   │   ├── BTService_UAVUpdateState.h/cpp   # 状态更新服务
│   │   └── BTService_UAVPathPlanning.h/cpp  # 路径规划服务
│   ├── Decorators/
│   │   └── BTDecorator_UAVAtLocation.h/cpp  # 位置检查装饰器
│   └── README_BehaviorTree.md      # 行为树使用文档
├── Debug/
│   ├── DebugVisualizer.h/cpp       # 调试可视化
│   ├── DataLogger.h/cpp            # 数据记录
│   ├── ControlParameterTuner.h/cpp # 控制参数调试
│   ├── UAVHUD.h/cpp                # HUD显示
│   └── UAVLogConfig.h/cpp          # 日志配置
├── Tests/
│   ├── UAVTestCommon.h             # 测试通用工具
│   ├── Planning/
│   │   ├── ObstacleManagerTest.cpp
│   │   ├── AStarPathPlannerTest.cpp
│   │   ├── RRTPathPlannerTest.cpp
│   │   ├── TrajectoryOptimizerTest.cpp
│   │   └── TrajectoryTrackerTest.cpp
│   ├── Control/
│   │   ├── AttitudeControllerTest.cpp
│   │   └── PositionControllerTest.cpp
│   ├── Physics/
│   │   └── UAVDynamicsTest.cpp
│   ├── Mission/
│   │   └── MissionComponentTest.cpp
│   └── Core/
│       └── UAVTypesTest.cpp
├── Script/
│   └── test.bat                 # 自动化测试脚本
└── Utility/
    └── Debug.h/cpp                 # 调试工具（堆栈输出、性能计时等）
```

## 关键技术点

### 1. 坐标系统
- **世界坐标系**: UE5 左手坐标系 (X前, Y右, Z上)
- **机体坐标系**: FRD (前右下) 或 FLU (前左上)
- **地理坐标系**: WGS84 (使用 GeoReferencing 插件)

### 2. 时间步进
- **固定时间步**: 控制器和物理模拟使用固定时间步（如 0.002s）
- **变量时间步**: 渲染使用变量时间步
- **子步进**: 物理模拟可能需要多个子步进

### 3. 性能优化
- 使用 UE5 的多线程能力
- 异步计算密集型算法（路径规划）
- LOD 系统用于多机场景
- 数据驱动的配置系统

## 测试场景建议

1. **基础飞行测试**: 起飞、悬停、降落
2. **轨迹跟踪测试**: 圆形、8字形、复杂3D轨迹
3. **避障测试**: 静态障碍物、动态障碍物
4. **编队飞行测试**: 2-10架无人机编队
5. **任务执行测试**: 区域覆盖、目标跟踪
6. **极限测试**: 强风、传感器故障、通信中断

## 与 Claude CLI 协作建议

### 迭代开发流程
1. 使用 Claude CLI 生成模块骨架代码
2. 实现核心算法逻辑
3. 使用 Claude CLI 进行代码审查和优化
4. 添加测试和文档
5. 集成到主项目

### 提示词模板
参见 `CLAUDE_PROMPTS.md` 文件

## 参考资源

- PX4 Autopilot: https://github.com/PX4/PX4-Autopilot
- AirSim: https://github.com/microsoft/AirSim
- RotorS: https://github.com/ethz-asl/rotors_simulator
- 《Quadrotor Dynamics and Control》- Randal Beard
