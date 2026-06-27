# 实时调参（运行中 PID / NMPC / CBF / 风场 / slomo）

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **来源**: PRD `.scratch/web-control-panel/00-prd.md` 切片 5

## Parent

PRD：`.scratch/web-control-panel/00-prd.md`。设计文档：`docs/design/web-control-panel.md`（§7 实时调参）。

## What to build

在仿真**运行中**实时调节控制算法与环境参数，无需重跑。利用既有 `BlueprintReadWrite` 字段直接赋值（项目已有 `ControlParameterTuner` 的 `pid.set` 控制台命令先例），统一经 HTTP `/control/params` 下发。

端到端行为：仿真运行中，Web 端拖动滑块（PID 增益 / NMPC 权重与避障距离 / CBF 安全距离 / 风场强度 / slomo）并选择作用范围（全队 / Leader / 指定单机）→ POST → 下一帧生效 → 行为立即响应。

调参目标与落点：
- 姿态环 PID（Roll/Pitch/Yaw 的 Kp/Ki/Kd）→ `AttitudeController`（`BlueprintReadWrite`，薄封装）。
- 位置/速度环 PID → `PositionController`（`BlueprintReadWrite`，薄封装）。
- NMPC 全配置（权重/避障距离）→ `NMPCAvoidance->Config`（public，薄封装 `SetNMPCConfig`）。
- CBF-QP 配置 → **真实缺口**：当前 `private` 无 Setter，需改 public 或加 Setter。
- 控制模式 → 复用既有 `AUAVPawn::SetControlMode`。
- MPC 类型 → 写 `NMPCAvoidance->Config.MPCType`（懒加载已存在），薄封装。
- 风场 → 复用 runtime-safe 的 `UWindField::SetWindConfig`（切片 4 已建 slomo Setter，此处复用）。
- slomo → 复用切片 4 新增的运行时 Setter。

控制命令带 `target`：`fleet`（全体同步）/ `leader` / `{agentId}`。apply 前做值域校验（防把 NMPC/CBF 调到发散导致 NaN）。apply 成功后写一行 `param` 事件到 ndjson（agent/scope/key/old/new），供前端高亮变更、追溯响应。调参不计入判决指标、不触发 reload。

## Acceptance criteria

- [ ] 仿真运行中 POST 调整某机姿态 PID 增益后，该机下一帧控制行为按新增益响应（外部可观察：trace 中姿态响应特性变化）。
- [ ] 仿真运行中 POST 调整 NMPC 权重/避障距离后，轨迹跟踪与避障权衡立即变化。
- [ ] 仿真运行中 POST 调整 CBF-QP 安全距离/Alpha 后，安全约束严格度立即变化（CBFQPConfig 已可被外部访问，原 `private` 缺口已补）。
- [ ] 仿真运行中 POST 改变风场类型/强度后，全队立即感受到新风（外部可观察：ndjson frame 行的 wind 字段变化、各机轨迹受扰）。
- [ ] 仿真运行中 POST 改变 slomo 后，仿真时标立即变化且与 `TelemetryRecorder` 口径一致。
- [ ] `target=fleet` 时全队同步、`target=leader` 仅 Leader、`target={agentId}` 仅指定单机；非目标机不受影响。
- [ ] 越界/非法参数值被服务端拒绝（返回 400），不会下发到 UE，不会产生 NaN 毁掉仿真。
- [ ] 每次成功调参写一行 `param` 事件到 ndjson（含 old/new 值），前端可据此高亮与追溯。
- [ ] automation 单测覆盖：调参 Setter apply 后外部可读字段已变更、值域校验拒绝非法值（参照 `ControlParameterTuner` 现有控制台命令的测试范式）。

## Blocked by

- `01-control-channel-and-hot-reload.md`（反向控制通道与 GameThread 命令分发）
- （建议参考 `04-wind-acceptance-and-sim-control.md` 的 slomo Setter，避免重复实现）
