---
name: ue-profiling
description: 当需要对 UE C++ 项目进行性能分析时使用——通过 SCOPE_CYCLE_COUNTER 埋点定位性能瓶颈
---

# UE 性能 Profiling 埋点

## 概述

通过 UE5 内置的 `SCOPE_CYCLE_COUNTER` 统计系统，为关键代码路径添加性能埋点，使 `stat <GroupName>` 命令可实时查看各模块耗时，精准定位性能瓶颈。

## 前置条件

- UE C++ 项目可正常编译
- 项目尚未有自定义 Stat Group（若有则复用）

## 执行流程

### 1. 扫描性能关键路径

扫描项目中所有 Tick 函数和定时器回调，识别需要埋点的位置：

```
grep 目标:
- virtual void Tick(float DeltaTime)  → 所有 Actor/Component 的 Tick
- void TickComponent(float DeltaTime, ...)  → 组件 Tick
- SetTimer / FTimerHandle 回调函数
- 任何高频调用的计算函数（物理积分、求解器、矩阵运算等）
```

按以下规则筛选需要埋点的函数：

| 函数类型 | 是否需要埋点 | 原因 |
|---------|------------|------|
| Actor/Component Tick | 是 | 每帧执行，是主要开销来源 |
| 物理子步进中的积分函数 | 是 | 高频调用，计算密集 |
| 求解器/优化器（MPC、NMPC等） | 是 | 单次调用开销大 |
| 状态估计（EKF、卡尔曼等） | 是 | 矩阵运算开销不可忽视 |
| 传感器更新 | 是 | 可能包含射线检测等耗时操作 |
| Debug 绘制函数 | 是 | DrawDebug 调用在编辑器中开销大 |
| 纯数据读写/日志 | 否 | 开销微小，不值得埋点 |
| 一次性初始化函数 | 否 | BeginPlay/构造中只执行一次 |

### 2. 声明 Stat Group 和 Cycle Stat

在项目的**主头文件**（如 `ProjectName.h`）或新建一个 `Profiling.h` 中声明：

```cpp
// 在头文件中声明统计组和各项统计
DECLARE_STATS_GROUP(TEXT("项目统计组名"), STATGROUP_GroupName, STATCAT_Advanced);

// 每个需要埋点的模块各声明一个 Cycle Stat
DECLARE_CYCLE_STAT(TEXT("模块名"), STAT_ModuleName, STATGROUP_GroupName);
```

**命名规范：**

| 元素 | 规范 | 示例 |
|------|------|------|
| Stat Group 名 | 项目名称 | `UAVSimulator` |
| Cycle Stat 名 | `模块名` 或 `类名::函数名` | `PhysicsSubStep`、`UAVPawn::Tick` |
| 枚举标识符 | `STAT_` + 驼峰 | `STAT_PhysicsSubStep` |

### 3. 添加 SCOPE_CYCLE_COUNTER 埋点

在每个需要分析的函数开头添加：

```cpp
void AMyActor::Tick(float DeltaTime)
{
    SCOPE_CYCLE_COUNTER(STAT_MyActorTick);
    // ... 原有代码
}
```

**嵌套埋点规则：** 外层函数和内层函数各自埋点，UE 会自动计算层次关系：

```cpp
void AMyActor::Tick(float DeltaTime)          // STAT_MyActorTick = 总时间
{
    SCOPE_CYCLE_COUNTER(STAT_MyActorTick);
    DoPhysics();                                // STAT_Physics = 物理部分
    DoPlanning();                               // STAT_Planning = 规划部分
    DoDebugDraw();                              // STAT_DebugDraw = 绘制部分
}
// stat 输出中会显示父子关系和各自耗时
```

**关键原则：**

- 只在函数入口放一个 `SCOPE_CYCLE_COUNTER`，不要放在循环内（除非循环本身就是热点）
- 子函数如果已经埋点，父函数的计数器会自动包含子函数时间（包含关系，不是额外开销）
- 埋点开销极小（约 10ns），可以大胆添加，但避免在极小函数中重复埋点

### 4. 编译验证

```bash
# 编译项目
Script/build.bat   # Windows
Script/build.sh    # macOS/Linux
```

编译成功后运行项目，在控制台中输入：

```
stat GroupName
```

应看到类似输出：

```
GroupName
  ModuleA          5.23 ms
  ModuleB          2.11 ms
  ModuleC          0.34 ms
  ...
```

### 5. 分析与优化

根据 `stat` 输出，按耗时排序，识别 Top 3 瓶颈模块。

| 耗时占比 | 判断 | 行动 |
|---------|------|------|
| > 50% 帧时间 | 严重瓶颈 | 必须优化 |
| 20-50% 帧时间 | 显著开销 | 应该优化 |
| < 10% 帧时间 | 可接受 | 暂不处理 |

## 常见误区

| 误区 | 正确做法 |
|------|---------|
| 在循环体内放 SCOPE_CYCLE_COUNTER | 循环外放一次，循环内的子函数单独埋点 |
| 给所有小函数都埋点 | 只给有性能影响的函数埋点，过多计数器反而干扰分析 |
| 埋点后忘记验证 | 编译后必须跑 `stat` 确认输出正确 |
| 在 Release 构建中保留所有埋点 | 用 `#if UE_BUILD_DEBUG` 或条件编译控制，按需开关 |
| 只埋 Actor Tick 不埋子组件 | 子组件的 TickComponent 也要埋，否则看不到真实分布 |
