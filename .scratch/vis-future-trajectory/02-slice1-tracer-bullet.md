# Slice 1：未来轨迹探路骨架——打通"优化轨迹"全链路

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **类型**: AFK
- **Parent**: `.scratch/vis-future-trajectory/01-future-trajectory-stream.md`

## What to build

打通**第一类**未来轨迹（`TrajectoryTracker` 正在跟踪的优化轨迹 `FTrajectory`）从 UE 到 Web 的完整端到端链路，作为后续两类的探路骨架（tracer bullet）。这是一条贯穿所有集成层的窄而完整的通路：Recorder 写盘 → server.py 解析 → app.js 渲染 → UI 开关 → 双 seam 测试。

端到端行为：仿真中一架处于轨迹跟踪态的 UAV，其优化轨迹在 Web 可视化里以半透明未来线显示，与历史轨迹在飞机本体处衔接；跟踪停止后未来线自动消失。

### 数据契约（来自 PRD 原型，编码 schema 决策）

```jsonc
// 优化轨迹：TrajectoryTracker 当前跟踪的 FTrajectory
{"type":"traj_opt","t":<simSec>,"agent":<id>,"valid":true,
 "pts":[[x,y,z], ...]}   // 降采样后的若干未来点（cm，UE 原生坐标系，消费端变换）
```

### 关键实现约束

- **C++ `TelemetryRecorder`**：帧采样（或独立降频节拍，如 5Hz）遍历机队，当 `UAVPawn->GetTrajectoryTracker()->IsTracking()` 且轨迹 `bIsValid` 时写出 `traj_opt` 行；`pts` 等距降采样到上限（默认 ≤ 32 点）。仅当跟踪态才写，停止后自然停写。沿用现有"零侵入拉取"约定。
- **server.py**：`consume()` 增加 `traj_opt` 分支，`to_web` 变换（与现有 `frame.pos` 完全一致）；为每个 agent 维护**最新快照**（覆盖式，区别于历史 `trace` 的累积式）；`snapshot()` 在 agent 对象上挂 `futureOpt` 字段（已变换坐标的点数组）。未知 type 继续安全忽略。
- **app.js**：每个 agent 新增一条 `THREE.Line`（半透明、区别于历史 trail 的样式）绘制 `futureOpt`，每帧 `rebuildScene` 用最新快照更新几何体；控制面板加一个显示开关（复选框）绑定该 mesh 的 `visible`，默认开启；`renderLegend` 补一条图例。
- **C++ 单元测试**：用 `Tests/UAVTestCommon.h` 合成 World 模式，构造挂载 `TrajectoryTracker`（含有效 `FTrajectory`）的合成 `AUAVPawn`，实例化 `UTelemetryRecorder`，驱动若干 tick，断言落盘 ndjson 含 `traj_opt` 行、`agent` id 正确、`pts` 坐标与注入的 `FTrajectory.Points.Position` 一致（cm 原生）、跟踪停止后不再写出、降采样上限生效。这是 `TelemetryRecorder` 的首个单测，建立测试 seam。
- **Python 单元测试**：为 `server.py` 的 `consume()`/`snapshot()` 新增 pytest，喂入合成 `spawn`/`frame`/`traj_opt` 行，断言 agent 对象含 `futureOpt` 且坐标已正确 `to_web` 变换、未知 type 不抛异常、新 `traj_opt` 覆盖旧快照。这是 `server.py` 的首个 Python 测试，建立前端后端解析的回归保护。

## Acceptance criteria

- [ ] `sim.*` 运行仿真后，`telemetry.ndjson` 含 `traj_opt` 行，仅出现在对应 agent 处于轨迹跟踪态的时段
- [ ] Web 可视化中处于跟踪态的 agent 显示半透明优化轨迹线，与历史轨迹在飞机本体处衔接
- [ ] 跟踪停止后该 agent 的优化轨迹线在 Web 端自动消失
- [ ] UI 控制面板有显示开关可切换优化轨迹的可见性
- [ ] 图例含"优化轨迹"条目
- [ ] C++ 单元测试通过：断言 `traj_opt` 行数据契约（id/坐标/降采样/跟踪态门控）
- [ ] Python 单元测试通过：断言 `futureOpt` 字段经 `to_web` 变换、未知 type 安全、覆盖式快照
- [ ] 编译成功，无新增 warning（遵守禁止 Verbose 日志、高频路径用 `UE_LOG_THROTTLE`）
- [ ] README.md 功能特性描述更新（如该工具在 README 中有记录）

## Blocked by

None - can start immediately
