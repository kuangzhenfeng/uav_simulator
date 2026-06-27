# 可视化打通"未来轨迹"数据流：规划路径 / 优化轨迹 / NMPC 预测

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **来源**: 仿真可视化排查（UE 原生有三类未来轨迹，Web 端只画了历史轨迹）

## Problem Statement

研发人员在无头仿真（`-game -NullRHI`）下用 Web 可视化（`Tools/vis/`）观察机队时，只能看到各 agent 已飞过的**历史轨迹**（teal/blue 折线，来自 `telemetry.ndjson` 的 `frame.pos` 累积），**看不到任何未来将要执行的轨迹**。

而 UE 原生（PIE/编辑器）的可视化体系是"历史 + 未来"双轨的：`UPlanningVisualizer` 每帧会画出规划路径（绿）、优化轨迹（蓝，`TrajectoryTracker` 正在跟踪的 `FTrajectory`）、NMPC 预测轨迹（橙，`FNMPCAvoidanceResult.PredictedTrajectory`）。这些 `DrawDebug*` 绘制在无头 `NullRHI` 模式下根本不渲染，因此 Web 端完全缺失。

结果：研究者无法判断"控制器规划要去哪 vs 飞机实际飞到哪"，难以定位避障爬升、末段失速、横向偏差等问题的根因——这些问题往往正藏在规划与实际的差异里。Web 可视化作为"无头仿真的唯一可视化窗口"，必须把未来轨迹补齐。

## Solution

把 UE 运行时已有的三类未来轨迹数据，通过现有的 `TelemetryRecorder` → `telemetry.ndjson` 数据流，落盘成结构化数据，再由 `server.py` 解析、`app.js` 渲染。未来轨迹与历史轨迹同屏对比，并在 Web UI 上提供显示开关，让研究者按需切换三类未来轨迹的可见性。

核心思路：**不新增可视化数据通路，复用已验证可靠的 telemetry.ndjson append-only 流**（见 ADR-0001 场景数据资产化所确立的"声明式数据源 + 消费端变换"原则），仅在 Recorder 帧写入时追加未来轨迹行，消费端（Python + 前端）做坐标系变换与分层渲染。

## User Stories

1. 作为仿真研究员，我希望在 Web 可视化里同时看到每架无人机的历史轨迹和未来规划轨迹，这样我能直观判断飞机是否在按规划飞行。
2. 作为研究员，我希望看到 `TrajectoryTracker` 当前正在跟踪的优化轨迹，这样我能知道轨迹跟踪器"认为"飞机接下来要走的那条参考曲线。
3. 作为研究员，我希望看到 NMPC 滚动求解出的未来 N 步预测轨迹，这样我能观察控制器在每个时刻的真实意图和重规划行为。
4. 作为研究员，我希望看到路径规划器输出的离散规划路径，这样我能区分"粗规划路径"与"优化后的可执行轨迹"。
5. 作为研究员，我希望能用 UI 开关分别显示/隐藏"规划路径 / 优化轨迹 / NMPC 预测"三类未来轨迹，这样信息过载时我能聚焦关心的那一类。
6. 作为研究员，我希望未来轨迹用与历史轨迹明显区分的视觉样式（半透明、不同色相）绘制，这样我不会把未来轨迹误认成已飞过的轨迹。
7. 作为研究员，我希望未来轨迹的颜色与所属 agent 的主色系关联（如同历史轨迹），这样多机场景下我能分辨每条未来轨迹属于哪架机。
8. 作为研究员，我希望实时仿真时未来轨迹随控制器重规划而更新，这样我能看到 MPC 滚动重规划的动态过程。
9. 作为研究员，我希望仿真结束后回放时仍能看到未来轨迹（取对应时间点的快照），这样事后分析也能对照规划。
10. 作为研究员，我希望未来轨迹上的关键点能标注速度方向（速度箭头），这样我能评估规划的速度连续性。
11. 作为研究员，我希望 NMPC 预测点能反映障碍代价热度（如代价越高颜色越红/点越大），这样我能一眼看出规划在何处贴近障碍。
12. 作为研究员，我希望未来轨迹数据不会让 `telemetry.ndjson` 体积失控，这样长时仿真不会因日志膨胀而失败。
13. 作为研究员，我希望当某架机处于轨迹跟踪态时才出现未来轨迹、跟踪停止时未来轨迹自动消失，这样界面不会残留过期的未来曲线。
14. 作为研究员，我希望未来轨迹在 Z（高度）方向同样准确，这样能验证规划是否恒定高度避障。
15. 作为研究员，我希望三类未来轨迹在 Web UI 上有清晰的图例说明（颜色 + 名称），这样我无需查文档就能理解每条线。
16. 作为研究员，我希望未来轨迹在拖拽回放时间轴时与历史轨迹、飞机本体保持时间一致，这样回放分析不会错位。
17. 作为维护者，我希望未来轨迹数据流的解析（server.py）与渲染（app.js）有自动化测试保护，这样后续重构不会静默回归。
18. 作为维护者，我希望 `TelemetryRecorder` 的未来轨迹写入有 C++ 单元测试，这样数据契约变更能被编译期/测试期捕获。
19. 作为维护者，我希望未来轨迹的写入遵循现有节拍与降频策略，这样不破坏仿真主循环的性能与帧率。
20. 作为维护者，我希望新增的 ndjson 事件类型对旧消费端向后兼容（未知 type 安全忽略），这样增量上线不会因数据格式而崩溃。

## Implementation Decisions

### 数据通路（沿用 ADR-0001 声明式数据源原则，不新建通道）

- 复用 `TelemetryRecorder`（GameMode 持有的 `UActorComponent`，已按帧节拍追加写 `telemetry.ndjson`）。**不引入** WebSocket / 新端口 / 新文件，沿用 append-only 单文件 + SSE 增量推送。
- 新增 ndjson 事件类型承载未来轨迹。设计为"每帧可选附加"或"独立行"二选一，倾向于**独立行**（保持 `frame` 行紧凑、且未来轨迹可独立降频）。

### ndjson schema 新增（来自原型设计，编码数据契约）

新增三类事件行，每行 `type` 区分；坐标/单位保持 UE 原生（cm，左手系），消费端做 `to_web` 变换（与现有 `frame`/`spawn` 一致）。原型 schema：

```jsonc
// 优化轨迹：TrajectoryTracker 当前跟踪的 FTrajectory
{"type":"traj_opt","t":<simSec>,"agent":<id>,"valid":true,
 "pts":[[x,y,z,...], ...]}   // 降采样后的若干未来点（cm）

// 规划路径：路径规划器输出的离散路径
{"type":"traj_plan","t":<simSec>,"agent":<id>,
 "pts":[[x,y,z], ...]}

// NMPC 预测：滚动重规划的未来 N 步
{"type":"traj_nmpc","t":<simSec>,"agent":<id>,
 "pts":[[x,y,z,cost], ...]}  // cost 用于热度着色
```

- 每条未来轨迹行内联该 agent 的点序列，`pts` 为数组；NMPC 携带 `cost` 用于热度可视化。
- `valid`/是否跟踪态决定该行是否存在；跟踪停止后停止写该 agent 的对应行（消费端自然清空）。

### Recorder 端（C++）

- `TelemetryRecorder` 在帧采样（或独立降频节拍）中，遍历机队，从各 `AUAVPawn` 的 public getter 拉取未来轨迹：
  - 优化轨迹：`UAVPawn->GetTrajectoryTracker()->GetTrajectory()`（仅当 `IsTracking()` 且 `bIsValid`）。
  - 规划路径 / NMPC 预测：通过 `UAVPawn` 新增或已有的 public 取数接口获取 `PlanningVisualizer` 的持久化数据 / `FNMPCAvoidanceResult`。**优先复用 UAVPawn 已暴露的 getter**，遵循现有"零侵入拉取"约定（不改仿真主循环）。
- **降频 + 限点**：未来轨迹按低于 frame 的节拍写（如 5Hz 或每 N 帧），且每条轨迹点序列做等距降采样到上限（如 ≤ 32 点），避免 ndjson 体积失控。NMPC 预测本身已是有限步数，可全量或轻度降采样。
- 多 agent 携带 `agent=<id>` 字段，沿用现有 `frame` 的多机约定。

### server.py（Python 消费端）

- 在 `consume()` 增加 `traj_opt`/`traj_plan`/`traj_nmpc` 三个 type 分支；坐标经 `to_web` 变换（cm→m、左手系→右手系），与现有 `frame.pos` 变换完全一致。
- 为每个 agent 维护"当前帧的未来轨迹快照"（覆盖式，只保留最新一份，区别于历史 `trace` 的累积式）。
- `snapshot()` 在 agent 对象上追加字段，例如 `futureOpt`/`futurePlan`/`futureNmpc`（各自为已变换坐标的点数组）。
- 未知 type 仍安全忽略（已有行为，维持向后兼容）。

### app.js（前端渲染）

- 为每个 agent 创建三条额外的 `THREE.Line`（半透明、区别色相，或复用 agent 主色 + 不同 opacity/虚线样式），分别绘制三类未来轨迹。
- 每帧 `onNewData`/`rebuildScene` 用最新快照更新三条未来轨迹几何体（覆盖式，与历史 trail 的累积式区分）。
- UI：在控制面板新增三个显示开关（复选框），绑定三类未来轨迹 mesh 的 `visible`；默认优化轨迹开启、其余关闭，避免初始过载。
- 图例（`renderLegend`）补充三类未来轨迹条目。
- 回放（`cursorT`）时未来轨迹显示"游标时刻最近一次快照"（消费端只留最新一份，天然满足）。

### 时态与一致性

- 历史 trail（累积 `trace`）= 过去；未来轨迹（最新快照）= 当前时刻控制器意图。二者在飞机本体处衔接。
- 实时模式：未来轨迹随 MPC 重规划每节拍刷新（"一直在变"属正常行为，对应 UE 原生 `DrawDebug` 每帧重画）。
- 回放模式：未来轨迹取游标时刻的快照，与历史轨迹时间对齐。

### 性能约束

- 未来轨迹写入与渲染均需降频/限点；长时（数百秒）仿真下 `telemetry.ndjson` 增量在可接受范围（与现有 metrics 行量级相当）。
- 不修改 `sim.*` 退出码协议、不改关卡装配（与已落地的 `SimScenarioMap` 空白关卡方案正交）。

## Testing Decisions

### 测试原则

只验证**外部行为（数据契约 + 解析正确性）**，不测试内部实现细节。好测试 = 给定输入数据，断言输出/落盘/解析结果符合契约。

### C++ 单元测试（新增 TelemetryRecorder 测试）

- **Seam**：复用现有 `Tests/` 下的合成 World + helper 模式（参考 `ScenarioLoaderFleetTest.cpp` / `ScenarioE2ETest.cpp` 的 `UAVTestCommon.h` 合成 World 构造）。
- 构造一个挂载 `TrajectoryTracker`（含有效 `FTrajectory`）的合成 `AUAVPawn`，实例化 `UTelemetryRecorder`，驱动若干 tick 后读取落盘的 ndjson 文件，断言：
  - 出现 `traj_opt` 行且 `agent` id 正确；
  - `pts` 坐标与注入的 `FTrajectory.Points.Position` 一致（cm 原生，未变换）；
  - 跟踪停止（`IsTracking()==false`）后不再写出该 agent 的 `traj_opt` 行；
  - 降采样/限点上限生效（点数 ≤ 配置上限）。
- 这是 `TelemetryRecorder` 的首个单元测试，建立该组件的测试 seam。

### Python 单元测试（新增 server.py 解析测试）

- **Seam**：为 `Tools/vis/server.py` 的 `consume()`/`snapshot()` 新增 pytest 测试（项目首个 Python 测试，建立前端后端解析的回归保护）。
- 构造合成 ndjson 行（`spawn`/`frame`/`traj_opt`/`traj_plan`/`traj_nmpc`），喂入解析器，断言：
  - `snapshot()` 返回的 agent 对象含 `futureOpt`/`futurePlan`/`futureNmpc`，坐标已正确 `to_web` 变换（cm→m、坐标系翻转）；
  - 未知 type 不抛异常、不影响已知字段；
  - 新的 `traj_*` 行覆盖旧快照（只留最新一份）。
- 优先 art：参照 `server.py` 现有 `to_web`/`consume` 的数据形状写断言。

### 验收级验证

- C++ 单测 + Python 单测全绿后，跑一次完整 `sim.*` 仿真，Web 人眼核对三类未来轨迹出现且可开关切换、颜色/图例正确。

## Out of Scope

- 不修改 UE 原生 `UPlanningVisualizer` / `UDebugVisualizer` 的 `DrawDebug*` 逻辑（PIE 可视化保持现状，本 PRD 只服务无头 Web 可视化）。
- 不改变历史轨迹（`frame.pos` 累积 `trace`）的现有行为。
- 不修改 `sim.sh`/`sim.bat`、`SimScenarioMap` 空白关卡方案、场景退出码协议。
- 不引入 WebSocket / 实时双向通信；沿用 ndjson append-only + SSE 单向推送。
- 不实现未来轨迹的交互编辑或"what-if"重规划（仅只读可视化）。
- 不处理垂向恒定高度避障等控制策略本身的问题（那是独立议题，未来轨迹可视化只是帮助暴露它）。

## Further Notes

- 未来轨迹在实时仿真下"一直在变"是 MPC 滚动重规划的固有特征，与 UE 原生 `DrawDebug` 每帧重画一致，不是 bug；需在 UI/文档上向用户说明。
- 坐标系：UE 左手系（X前/Y右/Z上，cm）→ Web 右手系（Y上，m），变换复用 `server.py` 现有 `to_web`，三类未来轨迹与历史轨迹使用同一变换，保证空间对齐。
- 无头 `NullRHI` 下 `DrawDebug*` 不渲染，是必须走 `telemetry.ndjson` 结构化数据流的根本原因（与项目近期"可视化后端改用结构化数据源替代日志正则解析"的演进方向一致）。
- NMPC 预测的障碍代价热度可视化（红色越深代价越高）参照 `PlanningVisualizer::DrawNMPCPrediction` 的现有色映射逻辑，前端复刻即可。
- 多 agent 场景下，未来轨迹按 `agent` id 关联，颜色沿用各 agent 主色系（`server.py` 的 `PALETTE`），与历史 trail 区分仅靠样式（透明度/虚线）而非色相，保证多机可分辨。
