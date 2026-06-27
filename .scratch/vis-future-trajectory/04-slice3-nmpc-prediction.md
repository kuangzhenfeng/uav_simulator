# Slice 3：补齐"NMPC 预测轨迹"与障碍代价热度可视化

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **类型**: AFK
- **Parent**: `.scratch/vis-future-trajectory/01-future-trajectory-stream.md`

## What to build

在 Slice 1 已建立的骨架上，复用三层数据通路，补齐**第三类**未来轨迹：NMPC 滚动求解出的未来 N 步预测轨迹（UE 原生 `DrawNMPCPrediction` 里橙色那条 `FNMPCAvoidanceResult.PredictedTrajectory`），并附加障碍代价热度可视化（代价越高颜色越红/点越大，对应 UE 原生色映射逻辑）。

端到端行为：实时仿真时，每个处于 NMPC 求解中的 agent 显示一条随重规划滚动更新的未来预测线；预测点按障碍代价热度着色，让研究者一眼看出规划在何处贴近障碍。

### 数据契约（复用 schema 模式，额外携带 cost）

```jsonc
// NMPC 预测：滚动重规划的未来 N 步
{"type":"traj_nmpc","t":<simSec>,"agent":<id>,
 "pts":[[x,y,z,cost], ...]}   // cost 用于热度着色
```

### 关键实现约束

- **C++ `TelemetryRecorder`**：复用降频节拍与拉取循环，新增 `traj_nmpc` 行。NMPC 预测来源为 `UAVPawn` 的 `FNMPCAvoidanceResult`（`PredictedTrajectory` 数组，每步含 `Position` + `ObstacleCost`）——通过 UAVPawn public getter 零侵入拉取；仅当存在有效预测结果时写出。NMPC 预测本身是有限步数，可全量或轻度降采样。
- **server.py**：`consume()` 增加 `traj_nmpc` 分支，`to_web` 变换坐标，保留 `cost` 原值；agent 对象挂 `futureNmpc` 最新快照（覆盖式，含 cost）。
- **app.js**：每个 agent 新增一条 NMPC 预测 `THREE.Line`，**按 cost 热度对预测点/线段着色**（复刻 `PlanningVisualizer::DrawNMPCPrediction` 的色映射：代价越高越红/点越大），独立显示开关，`renderLegend` 补图例。
- **测试扩展**：在 Slice 1 已建的双测试 seam 里追加用例覆盖 `traj_nmpc`（坐标 + cost 字段、门控、变换）。

## Acceptance criteria

- [ ] `telemetry.ndjson` 含 `traj_nmpc` 行，仅出现在对应 agent 有有效 NMPC 预测结果时
- [ ] Web 可视化显示 NMPC 预测轨迹，实时仿真下随重规划滚动更新
- [ ] 预测点按障碍代价 cost 热度着色（代价越高越红/越大），可视觉区分贴近障碍的预测段
- [ ] NMPC 预测轨迹有独立显示开关，图例含"NMPC 预测"条目
- [ ] C++ 单元测试追加用例覆盖 `traj_nmpc`（坐标 + cost 契约/门控）
- [ ] Python 单元测试追加用例覆盖 `futureNmpc`（坐标变换 + cost 保留/覆盖式快照）
- [ ] 编译成功，无新增 warning

## Blocked by

- `.scratch/vis-future-trajectory/02-slice1-tracer-bullet.md`（Slice 1：需复用其 recorder 节拍、server 分支模式、前端 line 框架与双测试 seam）
