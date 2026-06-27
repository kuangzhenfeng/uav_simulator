# 反向控制通道 + 进程内热重载内核

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **来源**: PRD `.scratch/web-control-panel/00-prd.md` 切片 1

## Parent

PRD：`.scratch/web-control-panel/00-prd.md`。设计文档：`docs/design/web-control-panel.md`（§5 热重载、§6 UE HTTP 控制端）。

## What to build

这是一条 tracer bullet（端到端贯穿切片），打通"配置 → 重跑"的最小闭环，并建立可靠的进程内热重载内核。完成本切片后，外部经 HTTP 发送一份最小 Scenario DTO（仅含机队数量）即可触发 UE 在**不重启进程**的前提下销毁旧机队、用新机队数量重新装配。

端到端行为：

1. UE 启动时（含 headless `-game -NullRHI`）拉起一个 HTTP 控制端，监听本机端口，接收 POST 命令。
2. 一个 Python 反向代理在现有可视化端口（`:8765`）上新增 POST 路由，把控制命令转发到 UE 控制端，并返回回执；读不到 UE 控制端口时标记"控制不可用"。
3. 收到 reload 命令后，UE 在 GameThread 上执行：互斥加锁 → 销毁旧机队 → 重置所有持久子组件状态 → 用新内存 `UScenario` 重新装配 → 解锁。
4. 重载完成后在 `Logs/telemetry.ndjson` 写一行 `reload` 事件，供前端/Python 感知。

本切片承载地基架构决策（已在 PRD/设计文档定稿，实施时严格对照设计文档 §5/§6）：
- UE HTTP 控制端独立于 `SoftUEBridge` 插件进程（后者 `-unattended` 会跳过），但复用其 HttpRouter 用法范式（CORS、`AsyncTask(GameThread)` 切换、`FUnattendedScriptGuard`）。控制端**显式在 headless 启动**。
- 配置**不落盘 `.uasset`**：运行时构造内存 `UScenario` 喂给现有 `ScenarioLoader` 装配链路（ADR-0001）。
- 端口规划：`:8770` 新增控制端（占用自动 +1..+9 避让），端口写入 `Saved/.uav-ctrl/port.json` 供 Python 发现。
- `UScenarioRuntimeController` 挂在 GameMode，与 `ScenarioEvaluatorComponent` 同级。
- AgentID 重置策略：重置为 0（与 telemetry `agent` 字段连续一致），前提是重载前确保旧 Agent 全部销毁。

热重载必须建立的完整 reset 骨架（设计文档 §5.2 已逐一定位，覆盖机队/验收器/遥测器/风场/任务监控/退出 Timer）。关键点：热重载前必须清除 `BTTask_ExitSimulation` 注册的延迟 `RequestExit` Timer，否则会误杀进程。

本切片只支持最小配置维度（机队数量），其余配置维度由后续切片 2/3/4 扩展。

## Acceptance criteria

- [ ] UE 在 headless（`-game -NullRHI`）模式下启动后，控制端监听成功并写入 `Saved/.uav-ctrl/port.json`；端口被占用时自动 +1..+9 避让。
- [ ] 经可视化端口（`:8765`）POST 一份最小 DTO（机队数量 N）能透传到 UE 控制端并返回成功回执。
- [ ] 收到 reload 后，UE 销毁旧机队、用新数量 N 重新装配；装配后 `ScenarioFleet` 数量 = N，且 `AgentRegistry` 中 AgentID 从 0 连续。
- [ ] 连续两次重载（数量 3 → 数量 5）后，第二次的 AgentID 仍从 0 连续，无残留旧 Agent 注册项。
- [ ] 重载后 `Logs/telemetry.ndjson` 被 truncate 重写，且首行重写静态数据；末尾出现 `reload` 事件行。
- [ ] 重载后 `ScenarioEvaluatorComponent` 的终局标志（`bFinalFlushed`）已复位，新场景任务完成时仍能写出最终判决。
- [ ] 重载后风场阵风/Dryden 相位已重置（验证方法：上一场景触发阵风后重载，新场景开局无半截阵风）。
- [ ] 重载进行中（`bReloading` 互斥期）拒绝新命令；期间验收器/遥测器不采集半装配态指标。
- [ ] 重载前清除了 `BTTask_ExitSimulation` 延迟退出 Timer，连续重载不会导致 UE 进程被误杀。
- [ ] 不发送任何控制命令时，现有只读可视化与 `sim.*` 退出码协议（0/1/2）行为完全不变（回归保护）。
- [ ] C++ 层热重载内核有 automation 单测（合成 World + 持久子组件），断言重载前后外部可观察状态（Fleet 数量、注册表清空、`NextAgentID` 复位、ndjson truncate、指标归零），对齐 `Tests/Scenario/*` 范式。
- [ ] Python 反向代理有 pytest 单测，验证端口发现、POST 转发、错误回执（mock UE 连接）。

## Blocked by

None - can start immediately
