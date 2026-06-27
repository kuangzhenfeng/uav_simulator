# 前端控制面板（配置 Tab + 实时调参 + 3D 拾取）

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **来源**: PRD `.scratch/web-control-panel/00-prd.md` 切片 6

## Parent

PRD：`.scratch/web-control-panel/00-prd.md`。设计文档：`docs/design/web-control-panel.md`（§10 前端控制面板、§11 DTO）。

## What to build

在现有可视化前端（零构建原生 ES Module + Three.js）新增控制面板，承载切片 1~5 的所有后端能力，让用户在浏览器里完成"配置 → 重跑 → 观察 → 运行中调参"的完整闭环。不破坏现有 3D 场景 + 折线图 + 判决面板布局。

端到端行为：用户在配置 Tab 填写机队/障碍/航线/风场·验收/仿真参数 → 点重跑 → 看到 UE 按新配置热重载、前端自动重置回放游标并刷新场景静态数据 → 运行中切换到实时调参 Tab 拖滑块看即时响应。

面板组成（布局见设计文档 §10.1）：
- **配置 Tab**：
  - 机队：动态增删 UAV 行，每行选机型（5 个型号下拉）、初始位置 XYZ、朝向、Leader 标志；展开内联编辑该机航点序列。
  - 障碍：静态组（增删行，Type/Center/Extents/SafetyMargin）+ 动态组（MovementType/PatrolSpeed/PatrolPoints，可在 3D 视图点击拾取）。
  - 风·验收：风类型/稳态风/阵风/湍流参数；验收阈值（MinClearance/MaxLateralDev/Timeout/WaypointRadius/EnergyBudget）。
  - 仿真：slomo/时长/控制模式/MPC 类型/随机种子。
- **实时调参 Tab**：PID/NMPC/CBF/风场/slomo 滑块 + target 选择器（全队/Leader/单机）+ 调参历史记录（含 old/new，支持回滚）。
- **3D 拾取模式**：点击地面/障碍自动填入表单坐标（坐标系用 `server.py` 既有 `to_web()` 的逆变换，米右手 → 厘米左手）。
- **SSE `reload` 事件处理**：收到后重置回放游标、刷新场景静态数据（机队/障碍/航点），不显示上一场景残留轨迹。

前端始终只与可视化端口（`:8765`）通信（单一入口）。控制端不可用时（Python 探活失败），面板置灰并提示。

## Acceptance criteria

- [ ] 用户能在机队 Tab 动态增删无人机行、为每架选机型/位姿/朝向/Leader、展开内联编辑各自航点。
- [ ] 用户能在障碍 Tab 配置静态障碍（类型/位置/尺寸/安全边距）与动态障碍（运动模型/速度/巡逻点）。
- [ ] 用户能在风·验收 Tab 与仿真 Tab 配置完整场景上下文。
- [ ] 用户填写配置后点"重跑"，前端 POST DTO 经反向通道触发热重载，3D 场景按新配置刷新。
- [ ] 重跑完成后前端自动重置回放游标、刷新场景静态数据（机队/障碍/航点），无上一场景残留轨迹（US 21）。
- [ ] 用户能在运行中实时拖动滑块调 PID/NMPC/CBF/风场/slomo 并选 target，行为即时响应。
- [ ] 每次实时调参在前端有记录（old/new），可回滚到上一组（US 22）。
- [ ] 3D 拾取模式：点击地面/障碍自动把对应坐标填入当前活动表单字段（US 20）。
- [ ] 控制端不可用时面板置灰并给出明确提示，不影响只读可视化。
- [ ] 前端延续零构建原生 ES Module 路线，不引入框架/打包工具；现有 3D + 图表 + 判决面板布局不被破坏。
- [ ] 提供结构化手动验证清单（配置→重跑→观察→实时调参全链路），由人工在浏览器执行。

## Blocked by

- `01-control-channel-and-hot-reload.md`（反向控制通道）
- `02-per-agent-mission-and-fleet.md`（机队/每机航线 DTO）
- `03-obstacles-static-and-dynamic-patrol.md`（障碍 DTO）
- `04-wind-acceptance-and-sim-control.md`（风场/验收/仿真 DTO）
- `05-runtime-tuning.md`（实时调参 API）

（整合片：需切片 1~5 的全部后端 API 就绪后才有意义）
