# Web 可视化全栈重建：UE 编辑器 1:1 镜像 + 任务切割

- **状态**: ready-for-agent
- **类型**: PRD
- **创建**: 2026-06-28
- **来源**: ULW 会话——轨迹 90° 夹角根因排查后用户拍板全栈重做

关联 ADR：ADR-0001（场景资产化 `UScenario` 为 canonical 任务单元，本 PRD 的"任务"与之一致）、ADR-0002（风场场景级单例）。

---

## Problem Statement

仿真工程师当前在两套割裂的可视化系统之间工作：

**1. Web 离线可视化与 UE 编辑器内可视化视觉语言不一致（🟥 P0）**

UE 编辑器内由 `PlanningVisualizer` / `DebugVisualizer` / `ObstacleDetector` 等组件用 `DrawDebug*` 原语（共 76 处调用点，跨 6 文件）绘制：航点/跟踪点/障碍 Sphere、路径/轨迹 Line、速度/推力 DirectionalArrow、搜索边界 Box、点云 Point、数值标注 String、包围圆柱 Cylinder。但 Web 离线可视化（`Tools/vis/`）用完全不同的几何和配色重画了一遍——机体用自定义 Hub+四旋翼臂，航点用 Torus 环，障碍用半透明盒——两者既不复用原语类型也不复用颜色/线宽/分层。结果是工程师在编辑器里看到的行为转到 Web 复盘时认不出来，调参闭环断裂。

**2. Web 可视化缺少 UE 在编辑器内画的全部信息（🟥 P0）**

`TelemetryRecorder`（UE）当前只把 `pos`/`vel`/`traj_opt`/`traj_plan`/`traj_nmpc`/`obstacle`/`waypoint` 写进 `Logs/telemetry.ndjson`。UE 在编辑器内额外画的**推力向量、搜索边界、跟踪误差文本标注、点云、包围圆柱、稳定性 HUD 文本**这 6 类原语根本没有被记录。Web 工程师想复盘"为何这次 NMPC 求解发散"时，编辑器里能看到的箭头和标注在 Web 里完全不存在——只能 SSH 翻日志。

**3. 仿真数据无任务边界，历史结果随重跑丢失（🟥 P0）**

`TelemetryRecorder::ResetForNewScenario` 在每次场景装配（冷启动或 `AMultiAgentGameMode::AssembleScenario` 热重载）时 **truncate** `telemetry.ndjson`。工程师每次重跑后，上一次的遥测数据和判决结果**完全消失**。没有运行记忆，没有跨 Run 对比，没有任务管理。`scenario_result.json` 虽然落盘但孤立存放，与 ndjson 无关联。

---

## Solution

全栈重建一条"UE 编辑器画什么，Web 就显示什么"的 1:1 镜像管线，并在其上叠加任务切割管理层。保持现有协议层（SSE 实时推送、HTTP 快照、回放游标）不变，**不引入前端框架**，**不改 UE 编辑器内可视化行为**。

### 1. DrawDebug 捕获：缓冲式代理（Hybrid）

新建 `UDebugDrawBuffer`（`UGameInstanceSubsystem`）作为唯一捕获入口。每个 `Add*(...)` 方法**同时**调用原生 `DrawDebug*`（保证编辑器内可视化零变化）**并**把原语推入按 AgentID + Layer 索引的帧缓冲。`TelemetryRecorder` 每 tick（20Hz）把所有 agent 的缓冲聚合成一条 ndjson `debug` 行后清空。

vs 纯轮询方案：76 个调用点大多是 fire-and-forget（duration=-1 持久或几秒瞬时），组件不留状态，轮询会漏掉瞬时原语。vs 纯 wrapper 76 处全改：调用集中在少数方法内（8 个方法覆盖 PlanningVisualizer 的 28 处），按方法迁移即可。

### 2. ndjson v2 schema（向后兼容）

`meta` 行的 `version` 升到 2；无 `version` 字段的旧文件按 v1 处理。新增 `debug` 行，每帧聚合所有原语：

```json
{"type":"debug","t":2.35,"agent":0,"prims":[
  {"t":"sphere","p":[100,-50,200],"r":25.0,"c":[255,0,0,255],"d":-1,"layer":"nmpc"},
  {"t":"line","a":[0,0,0],"b":[100,100,100],"c":[0,255,0,255],"th":3.0,"d":-1,"layer":"path"},
  {"t":"arrow","a":[0,0,0],"b":[50,0,0],"sz":20.0,"c":[0,255,255,255],"d":-1,"layer":"velocity"},
  {"t":"box","p":[0,0,0],"e":[50,50,100],"q":[0,0,0,1],"c":[255,0,0,255],"d":-1,"layer":"obstacle"},
  {"t":"point","p":[10,10,10],"sz":8.0,"c":[255,255,0,255],"d":-1,"layer":"waypoint"},
  {"t":"text","p":[10,10,10],"s":"Error: 42.1","c":[255,255,255,255],"d":0.5,"layer":"label"}
]}
```

字段语义：`p`=UE 厘米 XYZ，`r`/`th`/`sz` 单位 cm，`c`=RGBA 0-255，`d`=duration（-1 持久 / >0 瞬时，Web 按墙钟自 `t` 起过期），`layer`=18 层之一（见 Implementation Decisions 的层 taxonomy）。

### 3. 任务切割：每任务目录 + 当前任务镜像

`Logs/tasks/<时间戳>_<场景名>/telemetry.ndjson` + `result.json`（每个仿真一次任务）。`Logs/telemetry.ndjson` 保留作为"当前任务"的 live-tail 镜像——UE 写任务目录时同步镜像，Python `state.py` 不破坏 live-tail 路径。任务边界由现有信号锚定：`meta` 行（任务开始）+ `verdict final:true`（任务结束）+ `reload` 行（进程内热重载）。`scenario_result.json` 归档进同一任务目录。

### 4. Web 重做：原生原语渲染 + 分层开关

删除现有 `Tools/vis/web/` 下 10 个 JS 文件（`scene.js`/`charts.js`/`replay.js`/`control.js`/`config-tab.js`/`tune-tab.js`/`preset-tab.js`/`sim-panel.js`/`api.js`/`app.js`）。保留 `coord.js`/`logger.js`/`index.html`/`style.css` 基础设施。从零重写：Three.js scene 核心、6 种原语渲染器（Sphere/Line/Arrow/Box/Point/Text-Sprite）、分层开关浮层（18 层每层独立 toggle + 颜色色板 + 总开关）、任务选择器（3D 视图左上角浮层：时间/场景/PASS-FAIL 下拉 + 删除/导出）、SSE/回放集成、验收判决面板（保留）。

---

## User Stories

### DrawDebug 捕获与记录（UE 侧）

1. 作为仿真工程师，我希望 UE 在编辑器内画的所有调试原语自动被记录下来，这样我不用为了在 Web 上看到它们而额外配置。
2. 作为仿真工程师，我希望编辑器内的可视化行为完全不变，这样我不会因为"Web 镜像"功能而失去现有的调试体验。
3. 作为仿真工程师，我希望推力向量、速度向量、搜索边界、跟踪误差文本这些编辑器里能看的东西在 Web 复盘时也能看到，这样能远程诊断控制层问题。
4. 作为 UE 开发者，我希望新增的 `UDebugDrawBuffer` 是个独立 subsystem，通过 `Get(UWorld*)` 拿到，这样我加新调试原语时只需要调 buffer 的 Add 方法。
5. 作为 UE 开发者，我希望 `Add*` 方法既画又缓冲，这样我不用同时维护两套调用。
6. 作为 UE 开发者，我希望瞬时原语（duration > 0）和持久原语（duration = -1）都能被正确记录，这样 Web 端的过期逻辑能正确反映编辑器内的 lifetime 语义。
7. 作为 UE 开发者，我希望每个原语带 AgentID 和 Layer 标签，这样 Web 端能按机体或按层过滤。
8. 作为 UE 开发者，我希望 `TelemetryRecorder` 的 debug 帧节拍与 frame 一致（20Hz），这样原语位置与机体位置时间对齐。

### 任务切割与管理

9. 作为仿真工程师，我希望每次仿真（冷启动或热重载）自动归档成一个独立任务目录，这样历史结果不会随重跑丢失。
10. 作为仿真工程师，我希望任务目录按"时间戳_场景名"命名且可排序，这样我能快速找到某次特定运行。
11. 作为仿真工程师，我希望 `telemetry.ndjson` 和 `scenario_result.json` 归在同一个任务目录下，这样不用在两个地方找同一次运行的数据。
12. 作为仿真工程师，我希望当前的 `Logs/telemetry.ndjson` 仍然实时更新（镜像最新任务），这样现有的 live-tail 工作流不破坏。
13. 作为仿真工程师，我希望任务列表在 UE 进程重启后仍然可见，这样我能复盘昨天/上周的运行。
14. 作为仿真工程师，我希望能在 Web 端看到所有任务列表（时间、场景、PASS/FAIL），这样不必翻文件系统。
15. 作为仿真工程师，我希望能在 Web 端切换不同任务查看其 3D 可视化与判决，这样能快速回看某次失败任务。
16. 作为仿真工程师，我希望能在 Web 端删除不需要的任务记录，这样任务列表不会被过期测试数据淹没。
17. 作为仿真工程师，我希望能在 Web 端导出某次任务的 ndjson + result.json，这样能分享给同事或归档。
18. 作为 CI 工程师，我希望 batch 跑多个场景参数变体后，每次运行都是独立任务，这样能逐个回看而不混淆。

### Web 原语渲染（视觉 1:1）

19. 作为仿真工程师，我希望 Web 3D 视图里画的 Sphere/Line/Arrow/Box/Point/Text 在位置、大小、颜色、线宽上与 UE 编辑器内一致，这样我认得出这是同一组调试信息。
20. 作为仿真工程师，我希望 Web 端的坐标变换（UE 左手 cm → Web 右手 m）对所有原语一致应用，这样不会出现某层原语偏移的 bug。
21. 作为仿真工程师，我希望文本标注在 Web 端以可读的 Sprite 形式渲染（不糊、不挡视线），这样能看清"Error: 42.1"这类诊断数值。
22. 作为仿真工程师，我希望瞬时原语在 Web 端按其 duration 自动消失，这样不会满屏堆积过期标注。
23. 作为仿真工程师，我希望持久原语（duration=-1）在 Web 端持续显示直到场景切换，这样与编辑器内 lifetime 一致。

### 分层开关与配置

24. 作为仿真工程师，我希望每个调试层（path / trajectory / nmpc / obstacle / waypoint / velocity / ...）有独立的显示开关，这样信息密度大时能逐层关闭噪音。
25. 作为仿真工程师，我希望有个总开关一键全开/全关所有调试层，这样快速切换"纯机体飞行视图"和"全调试视图"。
26. 作为仿真工程师，我希望每个层的开关旁边有色板，这样一眼知道屏幕上的颜色对应哪层。
27. 作为仿真工程师，我希望分层开关的状态能跨刷新记住（localStorage），这样不用每次重开都重新勾选。
28. 作为仿真工程师，我希望仍能独立开关三类未来轨迹（优化/规划/NMPC 预测），这样与原调试层分开管理。

### 回放与实时

29. 作为仿真工程师，我希望 Web 端仍支持 SSE 实时推送（仿真跑着时画面跟着动），这样能边跑边看。
30. 作为仿真工程师，我希望 Web 端仍支持时间轴游标拖拽回放，这样跑完能任意回看某个时刻。
31. 作为仿真工程师，我希望回放时调试原语按游标时刻正确显示/过期，这样拖到 t=5s 看到的就是 UE 在 t=5s 画的东西。
32. 作为仿真工程师，我希望切换任务时游标重置、3D 视图刷新，这样不会把上一个任务的轨迹混进新任务。

### 验收与诊断（保留能力）

33. 作为仿真工程师，我希望验收判决面板（PASS/FAIL、航点到达、最小净空、横向偏差、耗时、碰撞、失败原因）保留，这样我能快速判断任务是否通过。
34. 作为仿真工程师，我希望机队指标汇总表保留，这样能看每机的速度比、低速时长、最大偏差、姿态角、卡死次数。
35. 作为仿真工程师，我希望事件流（碰撞、抵达）保留，这样能定位关键时刻。
36. 作为仿真工程师，我希望风况标签保留，这样知道当前任务的风场配置。

### 兼容与可维护

37. 作为 UE 开发者，我希望 ndjson v2 与 v1 能共存（旧文件按 v1 解析、新文件按 v2 解析），这样不会因为升级破坏既有回放数据。
38. 作为前端开发者，我希望新前端仍保持 vanilla ES Module + 零构建路线，这样不用引入 webpack/vite。
39. 作为 CI 工程师，我希望 `Script/sim.sh` 跑完后退出码仍反映任务判决（PASS=0/FAIL=1/缺失=2），这样 CI 判定逻辑不变。
40. 作为维护者，我希望所有直接调用 `DrawDebug*` 的地方在迁移后归零（除 buffer 自身的代理调用），这样能靠 grep 验证迁移完整性。

---

## Implementation Decisions

### 捕获架构：缓冲式代理（已锁定）

- 新建 `UDebugDrawBuffer`（`UGameInstanceSubsystem`）作为唯一捕获入口。
- `AddSphere/AddLine/AddArrow/AddBox/AddPoint/AddText` 方法**同时**调用原生 `DrawDebug*`（保编辑器内行为不变）**并**推入帧缓冲。
- 缓冲项结构 `FBufferedPrimitive`：类型 + 点数组 + 半径/厚度/箭头尺寸 + 四元数 + 文本 + 颜色 + duration + AgentID + Layer。
- `TelemetryRecorder` 每 tick 通过 `UDebugDrawBuffer::Get(UWorld*)` 拉取并序列化成 `debug` 行，然后清空缓冲。
- 选代理不选轮询：76 个调用点大多是 fire-and-forget，组件不留状态，轮询会漏瞬时原语。

### ndjson v2 schema（已锁定）

- `meta.version` 升到 2；无 version 字段按 v1。
- 新增 `debug` 行：`{"type":"debug","t":<sim_sec>,"agent":<id>,"prims":[<原语>,...]}`
- 6 种原语：`sphere`/`line`/`arrow`/`box`/`point`/`text`。
- 字段：`p` 位置 cm、`a`/`b` 线/箭头端点 cm、`e` 盒半轴 cm、`q` 盒四元数、`r`/`th`/`sz` cm、`c` RGBA 0-255、`d` duration 秒、`s` 文本、`layer` 层名。
- 持久/瞬时语义：`d=-1` 持久（Web 端一直显示直到场景切换）；`d>0` 瞬时（Web 端按 `now - t > d` 过期）。

### 18 层 taxonomy（已从 76 调用点归纳，已锁定）

| 层 | 来源组件 | 含义 |
|---|---|---|
| `path` | PlanningVisualizer::DrawPath | 规划路径段 |
| `trajectory` | PlanningVisualizer::DrawTrajectory | Minimum-snap 优化轨迹 |
| `tracking` | PlanningVisualizer::DrawTrackingPoint | 当前跟踪参考点 |
| `obstacle` | PlanningVisualizer::DrawObstacle | 障碍形状 |
| `search_bounds` | PlanningVisualizer::DrawSearchBounds | A* 搜索体积 |
| `waypoint` | PlanningVisualizer::DrawWaypoints | 任务航点 |
| `nmpc` | PlanningVisualizer::DrawNMPCPrediction | NMPC 预测轨迹 |
| `body_axes` | DebugVisualizer::DrawUAVState | 机体姿态轴 |
| `velocity` | DebugVisualizer::DrawUAVState | 速度向量箭头 |
| `history_trail` | DebugVisualizer::DrawTrajectoryHistory | 历史轨迹 |
| `tracking_alt` | DebugVisualizer::DrawTrackingState | 期望状态球 |
| `label` | DebugVisualizer::DrawTrackingState | 跟踪误差文本 |
| `obstacle_mgr` | ObstacleManager::DrawDebugObstacles | 静态障碍可视化 |
| `planner_path` | PathPlanner::DrawDebugPath | 一次性 A* 路径 |
| `sensor_traces` | ObstacleDetector::DrawDebugInfo | 射线检测线 |
| `sensor_points` | ObstacleDetector::DrawDebugInfo | 点云 |
| `sensor_detected` | ObstacleDetector::DrawDebugInfo | 已检测障碍 |
| `hud_stability` | StabilityScorer | 稳定性 HUD 文本 |

### 任务切割存储模型（已锁定方案 A）

- `Logs/tasks/<时间戳>_<场景名>/telemetry.ndjson` + `result.json`：每次仿真一个目录。
- `Logs/telemetry.ndjson` 保留作"当前任务"镜像，供 Python live-tail（不破坏现有协议）。
- 任务边界信号：`meta`（开始）+ `verdict final:true`（结束）+ `reload`（热重载）。
- UE 侧：`TelemetryRecorder::ResetForNewScenario` 改为"归档当前文件到任务目录再 truncate"，不再直接 truncate。
- `scenario_result.json` 由 `ScenarioEvaluatorComponent` 写入，归档逻辑需协调两者落同一任务目录。
- 任务 ID：`<UTC 时间戳 YYYYMMDD_HHMMSS>_<场景名>`，可排序、跨进程稳定。
- Python 侧：`state.py` 扩展扫描 `Logs/tasks/` 目录索引，暴露任务列表给 Web。

### Web UI 布局（已锁定方案 a）

- 分层开关 + 任务选择器放在 **3D 视图左上角浮层**（不恢复已删除的控制面板）。
- 浮层内容：任务下拉（时间/场景/PASS-FAIL）+ 删除/导出按钮 + 分层开关（18 层每层 checkbox + 色板）+ 总开关 + 三类未来轨迹开关（保留）。
- 删除的 Web 模块不恢复：时序图（charts.js）、控制面板（control/config/tune/preset/sim-panel/api）、完整回放时间轴 UI（replay.js 的复杂 UI）。
- 保留的能力：SSE 实时推送、回放游标（最小化 scrub 条）、验收判决面板、机队指标表、事件流、风况标签。

### DrawDebug 迁移顺序（按方法而非按调用点）

76 个调用点集中在少数方法内，按方法迁移：
- PlanningVisualizer（28 处，8 方法）：DrawPath / DrawTrajectory / DrawTrackingPoint / DrawObstacle / DrawSearchBounds / DrawWaypoints / DrawNMPCPrediction / DrawPersistentData
- DebugVisualizer（25 处，8 方法）：DrawUAVState / DrawTrajectoryHistory / DrawPlannedPath / DrawOptimizedTrajectory / DrawTrackingState / DrawObstacles / DrawWaypoints / DrawPersistentTrajectory
- ObstacleManager（16 处，1 方法）：DrawDebugObstacles
- PathPlanner（4 处，1 方法）：DrawDebugPath
- ObstacleDetector（10 处，3 块）：DrawDebugInfo 的 bShowDebugTraces / bShowPointCloud / bShowDetectedObstacles
- StabilityScorer（1 处）：HUD 文本

### 坐标一致性

所有新原语走与现有数据相同的 `to_web`（`web_x = Y_ue`, `web_y = Z_ue`, `web_z = -X_ue`，cm→m）。方向向量（箭头的 a→b）用同一 `to_web` 对端点各自变换，方向自动正确。

### 文本渲染保真（最高风险已识别）

`DrawDebugString` 在 Web 端用 `THREE.Sprite` + `CanvasTexture`（在 2D Canvas 上画文本后上传为纹理）。限制每帧≤3 条活动文本避免性能塌。字体用 sans-serif，DPR 缩放防糊。

---

## Testing Decisions

### 测试哲学

只测外部行为（"给 ndjson 喂 X，snapshot 产出 Y"），不测实现细节（"内部 dict 结构"）。优先复用现有接缝，新增接缝只在链路断层处补。

### 三个接缝（已与用户确认）

**接缝 1（主，pytest，杠杆最高）** — 扩展 `Tools/vis/test_ndjson_state.py` + `Tools/vis/test_coords.py`：
- 喂合成 `debug` 行（6 种原语各一例），断言 `NdjsonState.snapshot().debug` 按层分组且 `to_web` 坐标正确。
- 断言瞬时原语（`d=1.0`）在 `last_t > t+1.0` 后从 snapshot 消失，持久原语（`d=-1`）保留。
- 断言多任务 ndjson（两个 `meta` + 两个 `verdict final`）被正确切割成两个任务索引项。
- 断言 `dir_to_web`（新增的方向向量变换）与 `to_web` 在端点上等价。
- 既有先例：`test_ndjson_state.py` 已覆盖 frame/spawn/obstacle 解析；`test_coords.py` 已覆盖 `to_web` 轴映射。

**接缝 2（次，UE Automation Test）** — 扩展 `Source/uav_simulator/Tests/Telemetry/TelemetryRecorderTest.cpp`：
- 断言 TelemetryRecorder 用 `SetOutputPath` 重定向到临时文件后，跑 tick 循环产出的 ndjson 含 `version:2` meta + 合法 `debug` 行结构。
- 断言任务归档：跑两个任务后 `Logs/tasks/` 下有两个目录，每个含 `telemetry.ndjson` + `result.json`。
- 既有先例：现有 `TelemetryRecorderTest` 已用 `SetOutputPath` + 解析输出文件的模式。

**接缝 3（辅，人工 QA）** — `Script/sim.sh` + 浏览器截图 vs UE 编辑器截图：
- 跑一个标准场景（3 机 + 障碍），在 UE 编辑器 t=5s 截图；Web 端拖游标到 t=5s 截图。
- 断言 sphere/arrow/box 中心位置 ≤2cm 容差（UE 单位），颜色按层 taxonomy 匹配，原语数量匹配。
- 既有先例：项目对 JS 渲染一直用浏览器人工 QA，无 JS 测试运行器（vanilla 路线）。

### 为何不是单一接缝

JS 渲染正确性无法从 pytest 断言；UE 侧发射正确性无法从 UE 外部断言。三接缝是覆盖 C++→Python→JS 全链路且不引入 JS 测试运行器的最小集合。

### 回归保护

- 既有 UE Automation Test 全集（Control/Planning/MultiAgent/Sensors/...）必须无新增失败。
- 既有 pytest（`test_validation.py`/`test_presets.py`/`test_server.py`）必须绿。
- `Script/sim.sh` 退出码契约（PASS=0/FAIL=1/缺失=2）不变。

---

## Out of Scope

- **跨 Run 指标对比视图**：本 PRD 只做任务切割存储 + 单任务查看；多任务并排对比曲线、指标 diff 表格属 `.scratch/vis-analytical-dashboard/00-prd.md` 的范畴，不在此实现。
- **参数扫描**：批量跑参数变体并聚合热力图，属 analytical-dashboard PRD，不在此实现。
- **诊断面板**（NMPC 收敛率、CBF slack 分布、子模块耗时）：属 analytical-dashboard PRD。本 PRD 只把原语画出来，不做二级聚合统计。
- **Web 控制面板恢复**：用户已明确删除热重载配置/实时调参/预设管理面板；本 PRD 不恢复，UE 控制端 `:8770` 的接入能力不重建。
- **时序图恢复**：用户已明确删除速度/高度/净空折线图；本 PRD 不恢复。
- **多机协同专门视图**（编队拓扑图、机间通信时序）：未来需求。
- **UE 编辑器内可视化改造**：明确保持现状，`UDebugDrawBuffer` 的代理调用保证编辑器内行为零变化。
- **ndjson 压缩/分片**：任务目录内 ndjson 不做压缩，保持纯文本可 tail。
- **权限/多用户**：本地单用户场景，不做任务目录的权限管理。

---

## Further Notes

### 迁移完整性验证

迁移完成后用 `git grep "DrawDebug" Source/uav_simulator/{Planning,Debug,Sensors}/*.cpp` 应返回 0 命中（除 `DebugDrawBuffer.cpp` 内的代理调用）。这是迁移完整性的可 grep 验证标准。

### 并行执行波次

- Wave 1（并行）：UE 建 `UDebugDrawBuffer` + Web 删 10 个 JS。
- Wave 2：UE 扩 TelemetryRecorder（v2）+ 单测。
- Wave 3A（并行）：UE 迁移 76 调用点（3 组件并行）+ Python 解析 debug 行 + 任务切割层（UE 改存储 + Python 任务索引）+ Web scene 脚手架。
- Wave 3B（并行）：Web 6 种原语渲染器 + 分层开关浮层 + 任务选择器 + SSE/回放 + verdict 面板 + Python 解析测试。
- Wave 4：端到端冒烟（UE 截图 vs Web 截图 @t=5s）+ 文档。

### 与既有 PRD 的关系

本 PRD 取代 `.scratch/vis-future-trajectory/`（已完成）和 `.scratch/web-control-panel/`（已删除的模块）中与 Web 可视化基础架构相关的部分。`.scratch/vis-analytical-dashboard/00-prd.md` 在本 PRD 完成后可作为上层演进依赖本 PRD 的任务切割层。

### 最高风险

`DrawDebugString`（文本标注）的 Web 端 Sprite 渲染保真度——字体/DPI 处理不当会糊或挡视线。缓解：限制每帧≤3 条活动文本，CanvasTexture 用 DPR 缩放，sprite 尺寸按距离自适应。
