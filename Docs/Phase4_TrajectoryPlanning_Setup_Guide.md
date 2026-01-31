# Phase 4 轨迹规划系统 - UE5 编辑器配置指南

## 概述

C++代码层面已实现以下组件：
- 路径规划器 (A*, RRT)
- 轨迹优化器 (TrajectoryOptimizer)
- 轨迹跟踪器 (TrajectoryTracker)
- 障碍物管理器 (ObstacleManager)
- 行为树任务/服务 (BTTask_UAVFollowTrajectory, BTService_UAVPathPlanning)

以下是在 UE5 编辑器中完成配置的步骤。

---

## 步骤 1：创建黑板资产 (Blackboard)

1. **内容浏览器** → 右键 → **Artificial Intelligence** → **Blackboard**
2. 命名为 `BB_UAV`
3. 双击打开，添加以下键值：

| 键名 | 类型 | 说明 |
|------|------|------|
| `TargetLocation` | Vector | 目标位置 |
| `Trajectory` | Object | 轨迹数据 (UTrajectoryData) |
| `Waypoints` | Vector | 航点（简化版，单个目标点） |
| `bHasValidPath` | Bool | 是否有有效路径 |
| `bCollisionDetected` | Bool | 是否检测到碰撞 |

---

## 步骤 2：创建行为树资产 (Behavior Tree)

1. **内容浏览器** → 右键 → **Artificial Intelligence** → **Behavior Tree**
2. 命名为 `BT_UAV_Navigation`
3. 双击打开，在 **Details** 面板中设置 **Blackboard Asset** 为 `BB_UAV`

### 构建行为树结构

```
Root
└── Selector
    └── Sequence
        ├── [Service] BTService_UAVPathPlanning  ← 附加到此节点
        └── BTTask_UAVFollowTrajectory
```

### 配置 BTService_UAVPathPlanning

在 Sequence 节点上右键 → **Add Service** → **UAV Path Planning**

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| Target Location Key | TargetLocation | 目标位置黑板键 |
| Path Planning Algorithm | AStar | 规划算法 |
| Replanning Threshold | 200 | 目标移动超过此距离触发重规划 |
| Collision Check Distance | 300 | 碰撞检测距离 (cm) |
| Collision Warning Distance | 150 | 碰撞警告距离 (cm) |
| Safety Margin | 50 | 安全边距 (cm) |
| Enable Dynamic Avoidance | ✓ | 启用动态避障 |

### 配置 BTTask_UAVFollowTrajectory

添加任务节点 → **UAV Follow Trajectory**

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| Trajectory Key | Trajectory | 轨迹数据黑板键 |
| Waypoints Key | Waypoints | 航点黑板键 |
| Use Trajectory From Blackboard | false | 使用航点生成轨迹 |
| Max Velocity | 500 | 最大速度 (cm/s) |
| Max Acceleration | 200 | 最大加速度 (cm/s²) |
| Completion Threshold | 0.98 | 完成阈值 |
| Hold Position On Complete | ✓ | 完成后保持位置 |

---

## 步骤 3：配置 UAV 蓝图

打开 UAV 蓝图（如 `BP_UAV_Default` 或 `BP_TestUAV`）：

### 3.1 设置 AI 控制器

1. 选中蓝图根组件
2. **Details** 面板 → **Pawn** 分类：
   - **AI Controller Class** = `UAVAIController`
   - **Auto Possess AI** = `Placed in World or Spawned`

### 3.2 添加/配置组件

确保以下组件已添加（如果C++基类已包含则检查参数）：

**TrajectoryTracker 组件：**
- 检查跟踪参数是否正确

**ObstacleManager 组件：**
- Detection Range = 1000 (检测范围 cm)
- Update Interval = 0.1 (更新间隔 秒)

**PlanningVisualizer 组件：**
- Show Path = ✓
- Show Trajectory = ✓
- Show Obstacles = ✓
- Path Color = Green
- Trajectory Color = Blue

### 3.3 分配行为树

在蓝图 **Details** 面板或 AI Controller 中：
- **Behavior Tree** = `BT_UAV_Navigation`
- **Blackboard** = `BB_UAV`

---

## 步骤 4：关卡设置

### 4.1 放置 UAV

1. 将 UAV 蓝图拖入关卡
2. 确认 **Auto Possess AI** 已启用

### 4.2 添加障碍物（可选）

1. 放置静态网格体作为障碍物
2. 为障碍物添加碰撞体
3. 或在 ObstacleManager 中手动注册：
   ```
   在关卡蓝图中调用：
   UAV → GetObstacleManager → RegisterObstacle
   ```

### 4.3 设置目标点

**方法 A：通过蓝图设置**
```
Get AIController → Get Blackboard → Set Value as Vector
  Key Name: "TargetLocation"
  Value: 目标位置
```

**方法 B：创建目标点 Actor**
1. 创建 `BP_TargetPoint` 蓝图
2. 在 BeginPlay 中将自身位置写入 UAV 的黑板

---

## 步骤 5：调试与测试

### 5.1 启用可视化调试

在 UAV 蓝图或运行时：
```
PlanningVisualizer → bShowDebug = true
DebugVisualizer → bDrawTrajectory = true
```

### 5.2 运行时监控

在蓝图中获取状态：
```
UAV → GetTrajectoryProgress → 返回 0.0 ~ 1.0
UAV → IsTrajectoryComplete → 返回 bool
UAV → GetControlMode → 返回当前控制模式
```

### 5.3 控制台命令（如已实现）

```
ShowDebug AI          // 显示AI调试信息
ShowDebug Navigation  // 显示导航调试信息
```

---

## 步骤 6：参数调优建议

| 场景 | 调整参数 |
|------|----------|
| 轨迹不够平滑 | 降低 MaxAcceleration，增加采样间隔 |
| 响应太慢 | 提高 MaxVelocity 和 MaxAcceleration |
| 避障过于保守 | 减小 SafetyMargin |
| 频繁重规划 | 提高 ReplanningThreshold |
| 路径穿越障碍物 | 检查 ObstacleManager 注册，增大碰撞检测距离 |

---

## C++ 源文件位置

### 规划系统
```
Source/uav_simulator/Planning/PathPlanner.h/cpp          - 路径规划基类
Source/uav_simulator/Planning/AStarPathPlanner.h/cpp     - A*算法实现
Source/uav_simulator/Planning/RRTPathPlanner.h/cpp       - RRT算法实现
Source/uav_simulator/Planning/TrajectoryOptimizer.h/cpp  - 轨迹优化器
Source/uav_simulator/Planning/TrajectoryTracker.h/cpp    - 轨迹跟踪器
Source/uav_simulator/Planning/TrajectoryData.h/cpp       - 轨迹数据类
Source/uav_simulator/Planning/ObstacleManager.h/cpp      - 障碍物管理器
Source/uav_simulator/Planning/PlanningVisualizer.h/cpp   - 规划可视化
```

### 行为树组件
```
Source/uav_simulator/AI/Tasks/BTTask_UAVFollowTrajectory.h/cpp    - 跟随轨迹任务
Source/uav_simulator/AI/Tasks/BTTask_UAVFlyToLocation.h/cpp       - 飞向位置任务
Source/uav_simulator/AI/Services/BTService_UAVPathPlanning.h/cpp  - 路径规划服务
```

### 核心系统
```
Source/uav_simulator/Core/UAVPawn.h/cpp                  - UAV主类
Source/uav_simulator/Core/UAVTypes.h                     - 数据类型定义
Source/uav_simulator/Control/PositionController.h/cpp   - 位置控制器
Source/uav_simulator/Debug/DebugVisualizer.h/cpp        - 调试可视化
```

---

## 建议的资产目录结构

```
Content/
└── UAV/
    ├── AI/
    │   ├── BB_UAV.uasset              - 黑板
    │   └── BT_UAV_Navigation.uasset   - 行为树
    ├── Blueprints/
    │   ├── BP_UAV_Default.uasset      - UAV蓝图
    │   └── BP_TargetPoint.uasset      - 目标点蓝图
    └── Maps/
        └── TestLevel.umap             - 测试关卡
```

---

## 快速开始检查清单

- [ ] 创建黑板 `BB_UAV`，添加必要的键值
- [ ] 创建行为树 `BT_UAV_Navigation`
- [ ] 在行为树中添加 `BTService_UAVPathPlanning` 服务
- [ ] 在行为树中添加 `BTTask_UAVFollowTrajectory` 任务
- [ ] 配置 UAV 蓝图的 AI Controller Class
- [ ] 设置 Auto Possess AI = Placed in World or Spawned
- [ ] 分配行为树资产到 AI Controller
- [ ] 将 UAV 放入关卡
- [ ] 设置目标位置
- [ ] 运行测试
