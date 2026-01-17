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

### Phase 1: 基础框架 (2-3周)
- [ ] 创建 UAV Actor 基类
- [ ] 实现基础物理模型（简化动力学）
- [ ] 添加基础传感器（IMU、GPS）
- [ ] 实现简单的姿态控制器
- [ ] 创建测试场景和基础UI

### Phase 2: 飞行控制 (2-3周)
- [ ] 完善六自由度动力学模型
- [ ] 实现完整的姿态和位置控制器
- [ ] 添加状态估计器（EKF）
- [ ] 实现控制参数调试界面
- [ ] 添加飞行数据记录功能

### Phase 3: 轨迹规划 (2-3周)
- [ ] 实现基础路径规划算法（A*、RRT）
- [ ] 实现轨迹优化（最小snap）
- [ ] 添加轨迹跟踪控制器
- [ ] 实现动态避障
- [ ] 可视化规划结果

### Phase 4: 多机协同 (3-4周)
- [ ] 实现多无人机管理系统
- [ ] 添加编队控制算法
- [ ] 实现通信模拟
- [ ] 添加碰撞避免
- [ ] 实现任务分配算法

### Phase 5: 任务规划 (2-3周)
- [ ] 设计任务描述语言
- [ ] 实现任务解析器
- [ ] 添加任务调度器
- [ ] 实现任务监控系统
- [ ] 添加重规划功能

### Phase 6: 环境与优化 (2-3周)
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
│   ├── UAVActor.h/cpp              # 无人机基类
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
│   ├── PathPlanner.h/cpp           # 路径规划
│   ├── TrajectoryOptimizer.h/cpp   # 轨迹优化
│   └── ObstacleAvoidance.h/cpp     # 避障
├── MultiAgent/
│   ├── FormationController.h/cpp   # 编队控制
│   ├── TaskAllocator.h/cpp         # 任务分配
│   └── CommunicationManager.h/cpp  # 通信管理
├── Mission/
│   ├── MissionPlanner.h/cpp        # 任务规划
│   ├── MissionExecutor.h/cpp       # 任务执行
│   └── MissionMonitor.h/cpp        # 任务监控
└── Debug/
    ├── DebugVisualizer.h/cpp       # 调试可视化
    └── DataLogger.h/cpp            # 数据记录
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
