# Slice 2：补齐"规划路径"未来轨迹

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **类型**: AFK
- **Parent**: `.scratch/vis-future-trajectory/01-future-trajectory-stream.md`

## What to build

在 Slice 1 已建立的"未来轨迹全链路 + 双测试 seam"骨架上，复用同样的三层数据通路，补齐**第二类**未来轨迹：路径规划器输出的离散规划路径（UE 原生 `PlanningVisualizer` 里绿色那条 `PersistentPath`）。

端到端行为：当 UAV 有活动规划路径时，Web 可视化以与优化轨迹不同样式的半透明线显示该离散路径，可独立开关；让研究者区分"粗规划路径"与"优化后的可执行轨迹"。

### 数据契约（复用 Slice 1 的 schema 模式）

```jsonc
// 规划路径：路径规划器输出的离散路径
{"type":"traj_plan","t":<simSec>,"agent":<id>,
 "pts":[[x,y,z], ...]}   // 降采样后的路径点（cm，UE 原生坐标系）
```

### 关键实现约束

- **C++ `TelemetryRecorder`**：复用 Slice 1 的降频节拍与拉取循环，新增 `traj_plan` 行写入。规划路径来源为 `UAVPawn` 的 `PlanningVisualizer` 持久化路径数据——通过新增或复用 UAVPawn public getter 零侵入拉取（遵循现有约定，不改仿真主循环）；仅当有有效路径（点数 ≥ 2）时写出。
- **server.py**：`consume()` 增加 `traj_plan` 分支，`to_web` 变换；agent 对象挂 `futurePlan` 最新快照（覆盖式）。
- **app.js**：每个 agent 新增一条规划路径 `THREE.Line`（样式区别于优化轨迹，如虚线或不同色相/透明度），独立显示开关，`renderLegend` 补图例。
- **测试扩展**：在 Slice 1 已建的 C++ 单测与 Python 单测里**追加用例**覆盖 `traj_plan`（同一测试文件/seam，不新建），断言数据契约、门控、变换。

## Acceptance criteria

- [ ] `telemetry.ndjson` 含 `traj_plan` 行，仅出现在对应 agent 有有效规划路径时
- [ ] Web 可视化以独立样式（区别于优化轨迹）显示规划路径，可独立开关
- [ ] 图例含"规划路径"条目
- [ ] C++ 单元测试追加用例覆盖 `traj_plan`（数据契约/门控/降采样）
- [ ] Python 单元测试追加用例覆盖 `futurePlan`（变换/覆盖式快照）
- [ ] 编译成功，无新增 warning

## Blocked by

- `.scratch/vis-future-trajectory/02-slice1-tracer-bullet.md`（Slice 1：需复用其 recorder 节拍、server 分支模式、前端 line 框架与双测试 seam）
