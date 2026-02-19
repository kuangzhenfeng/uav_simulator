# 轨迹跟踪增强 — 技术说明

## 概述

本次变更围绕轨迹跟踪系统的鲁棒性进行了三项改进，解决 UAV 在复杂避障场景下轨迹跟踪失控的问题。

## 问题背景

原有实现中，`TrajectoryTracker` 以固定 `TimeScale` 推进轨迹时钟。当 UAV 因避障绕行或 NMPC stuck 而落后于轨迹时，时钟仍然匀速前进，导致：

- 期望位置与实际位置差距越来越大，控制器输出饱和
- 轨迹提前到期，触发错误的完成/重规划
- 位置控制器缺少加速度前馈，跟踪精度不足

## 变更内容

### 1. 自适应时间缩放

**文件:** `TrajectoryTracker.h` / `TrajectoryTracker.cpp`

当 UAV 落后于轨迹时，自动减速或暂停时钟推进。

**核心逻辑：**

```
误差计算 = 期望位置与实际位置的前向投影（沿轨迹方向）
  - 忽略横向偏移（避障造成的正常偏离）
  - 速度接近零时退化为欧氏距离

Error < ErrorSlowdownStart (100cm)  → 正常速度
Error ∈ [100cm, 300cm]              → 线性减速至 0
Error > ErrorPauseThreshold (300cm) → 暂停推进
```

**设计要点：**
- NMPC Override 激活时跳过自适应缩放（`!bHasDesiredStateOverride`），避免与 NMPC 控制冲突
- 使用前向投影而非欧氏距离，防止横向避障偏移误触减速

**配置参数（UPROPERTY，可在编辑器调整）：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `bEnableAdaptiveTimeScale` | true | 启用自适应缩放 |
| `ErrorSlowdownStart` | 100.0 cm | 开始减速的误差阈值 |
| `ErrorPauseThreshold` | 300.0 cm | 完全暂停的误差阈值 |

### 2. 时钟冻结（NMPC Stuck 保护）

**文件:** `TrajectoryTracker.h` / `TrajectoryTracker.cpp` / `BTService_UAVPathPlanning.cpp`

NMPC 检测到 stuck 状态时，完全冻结轨迹时钟，防止轨迹在 UAV 无法移动期间到期。

**调用链：**

```
BTService_UAVPathPlanning::CheckCollisionAndAvoid()
  → NMPC->ComputeAvoidance() 返回 Result.bStuck
  → Tracker->SetTimeFrozen(Result.bStuck)
```

**与自适应缩放的关系：**
- 自适应缩放先计算 `EffectiveTimeScale`
- `bTimeIsFrozen` 在最终更新时钟时做最后拦截
- 两者独立运作，冻结优先级更高

**生命周期管理：**
- 每帧由 `SetTimeFrozen(Result.bStuck)` 同步状态
- 障碍物消失时在无障碍物分支中显式解冻
- `Reset()`（由 `SetTrajectory()` 调用）清除冻结状态

### 3. 加速度前馈

**文件:** `UAVPawn.cpp`

轨迹跟踪模式下，将轨迹点的加速度信息传入位置控制器作为前馈。

**变更：**

```cpp
// 之前：仅位置+速度反馈
PositionController->ComputeControl(State, Pos, Vel, ...)

// 之后：增加加速度前馈
PositionController->ComputeControlWithAcceleration(State, Pos, Vel, Acc, ...)
```

**控制器内部处理：**

```
总期望加速度 = 轨迹加速度（前馈） + PID反馈加速度
推力向量 = (DesAcc.X, DesAcc.Y, Gravity + DesAcc.Z)
```

前馈使控制器能预判推力变化，减少对 PID 反馈的依赖，降低跟踪延迟。

## 修复的 Bug

**障碍物消失后时钟未解冻**

`CheckCollisionAndAvoid()` 中，当 `NearbyObstacles.Num() == 0` 提前返回时，遗漏了 `SetTimeFrozen(false)` 调用。已在 `ClearDesiredStateOverride()` 后补充。

## 系统交互图

```
BTService (每0.2s)
  ├─ Global Planner: A*/RRT → TrajectoryOptimizer → SetTrajectory()
  └─ Local Planner (每帧):
       ├─ 无障碍物 → ClearOverride + Unfreeze
       ├─ NMPC求解 → SetDesiredStateOverride / SetTimeFrozen
       └─ Stuck超限 → TriggerGlobalReplan

TrajectoryTracker (每帧)
  ├─ 自适应时间缩放（非Override时）
  ├─ 冻结检查
  ├─ 推进 TrackingTime
  └─ GetDesiredState() → Override 或 插值

UAVPawn (每帧)
  └─ GetDesiredState() → ComputeControlWithAcceleration() → AttitudeController → Motors
```

## 涉及文件

| 文件 | 变更类型 |
|------|----------|
| `Planning/TrajectoryTracker.h` | 新增自适应缩放属性、SetTimeFrozen 方法、bTimeIsFrozen 成员 |
| `Planning/TrajectoryTracker.cpp` | 实现自适应缩放逻辑、冻结检查、Reset 清理 |
| `AI/Services/BTService_UAVPathPlanning.cpp` | 调用 SetTimeFrozen、修复无障碍物分支解冻遗漏 |
| `Core/UAVPawn.cpp` | ComputeControl → ComputeControlWithAcceleration |
