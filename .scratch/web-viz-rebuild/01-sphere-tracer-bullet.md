# Slice 1 — 端到端骨架：Sphere 原语贯通（tracer bullet）

- **状态**: ready-for-agent
- **类型**: issue（vertical slice）
- **创建**: 2026-06-28
- **Parent**: `.scratch/web-viz-rebuild/00-prd.md`

## What to build

打通"UE 编辑器内画的 Sphere → ndjson 记录 → Python 解析 → Web 3D 渲染"这一条**端到端最窄路径**，作为全栈重构的 tracer bullet。只做 Sphere 一种原语 + 只迁移一个调用点（PlanningVisualizer::DrawWaypoints 的航点 sphere），但每一层都要真实贯通且可演示。

具体行为：

1. **UE 侧**：新建 `UDebugDrawBuffer`（`UGameInstanceSubsystem`），实现 `AddSphere(World, Pos, Radius, Color, Duration, AgentID, Layer)`——**同时**调用原生 `DrawDebugSphere`（保编辑器内行为不变）**并**把原语推入帧缓冲。`TelemetryRecorder`：`meta.version` 升到 2；每 tick（20Hz）拉取缓冲、聚合成一条 ndjson `debug` 行写出、清空缓冲。把 `PlanningVisualizer::DrawWaypoints` 中的 `DrawDebugSphere` 调用改成 `UDebugDrawBuffer::AddSphere`，layer 标 `"waypoint"`。

2. **Python 侧**：`coords.py` 加 `dir_to_web`（方向向量变换，仅缩放不复位）。`state.py` 的 `NdjsonState` 加 `_feed_debug` handler，解析 `prims` 数组，对每个 sphere 的 `p`/`r`/`c`/`d`/`layer` 应用 `to_web`，按 layer 分组存入 `self.debug_prims`，瞬时原语（`d>0`）记录 `expires_at = t + d`。`snapshot()` 暴露 `debug: {layer: [prims]}`，过滤已过期项。

3. **Web 侧**：删除 `Tools/vis/web/` 下 10 个旧 JS（`scene.js`/`charts.js`/`replay.js`/`control.js`/`config-tab.js`/`tune-tab.js`/`preset-tab.js`/`sim-panel.js`/`api.js`/`app.js`）；保留 `coord.js`/`logger.js`/`index.html`/`style.css`。新建精简 `scene.js`（Three.js: WebGLRenderer/PerspectiveCamera/OrbitControls/HemisphereLight+DirectionalLight/GridHelper 地面）+ `debug-renderer.js`（仅 `renderSphere`：MeshBasicMaterial wireframe，按 layer 分组到 `App.debugGroups`）+ 新 `app.js`（最小 SSE 接收 + rebuildScene + updateAgentPose）。

## Acceptance criteria

- [ ] UE 编译通过；编辑器内运行仿真时，DrawWaypoints 画出的航点 sphere **视觉不变**（位置/颜色/半径一致）。
- [ ] `Logs/telemetry.ndjson` 出现 `{"type":"meta",...,"version":2,...}` 行。
- [ ] `Logs/telemetry.ndjson` 出现 `{"type":"debug","t":<sim_sec>,"agent":<id>,"prims":[{"t":"sphere",...,"layer":"waypoint"}]}` 行。
- [ ] pytest：喂合成 `debug` 行（含一个 sphere）给 `NdjsonState`，`snapshot().debug["waypoint"]` 含正确 `to_web` 变换后的 sphere（位置/半径从 cm→m，轴映射 `web_x=Y_ue, web_y=Z_ue, web_z=-X_ue`）。
- [ ] pytest：sphere 的瞬时变体（`d=1.0`）在 `last_t > t+1.0` 后从 snapshot 消失；持久变体（`d=-1`）保留。
- [ ] 浏览器打开 Web，跑仿真，3D 视图出现航点 sphere，位置与 UE 编辑器一致（≤2cm 容差，UE 单位）。
- [ ] 既有 UE Automation Test 全集无新增失败。
- [ ] 既有 pytest（`test_validation`/`test_presets`/`test_server`/`test_coords`/`test_ndjson_state`）保持绿。

## Blocked by

None — 可以立即开始。
