# 每机独立航线 + 完整机队配置

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **来源**: PRD `.scratch/web-control-panel/00-prd.md` 切片 2

## Parent

PRD：`.scratch/web-control-panel/00-prd.md`。设计文档：`docs/design/web-control-panel.md`（§4.1 每机独立航线）。

## What to build

扩展 Scenario 数据模型以支持"每架无人机独立航线"，并把机队配置维度补全，使 Web 端能配置任意多机联动场景。

端到端行为：Web 端配置一个多机场景（每架机型、初始位姿、朝向、是否 Leader、各自独立的航点序列）→ 触发热重载 → 每架无人机装配自己的航线并各自执行。

核心 schema 决策（来自设计文档原型，内联）：

在 Agent 描述里内联航点，装配时每个 Agent 优先用自己的内联航点，为空则回退全局 `MissionProfile`：

```
FScenarioAgentEntry {
    // 既有：UAVClass, ModelID, InitialPosition, InitialYaw, bIsLeader
    + TArray<FMissionWaypoint> Waypoints   // 空=回退全局 MissionProfile
    + EMissionMode            MissionMode  // 默认 Once
}
```

`ScenarioLoader::AssembleFleetAndMission` 由当前"只对 `Fleet[0]` 应用 `MissionProfile`"改为**逐机装配**：遍历 `FleetSetup.Agents[]`，对每个 Agent 优先用其内联 `Waypoints`（连同 `MissionMode`）下发到自己的 `UMissionComponent` 并启动任务，内联为空才回退到全局 `MissionProfile`。

DTO 扩展完整机队字段（机型枚举 / 初始位置 XYZ / 朝向 / Leader 标志 / 每机航点序列 / 模式），经 `UScenarioFactory` 构造成内存 `UScenario`。DTO 全程用 UE 厘米左手系，仅前端展示层做米/右手变换。

## Acceptance criteria

- [ ] Schema 扩展后，构造一个两机场景（A 有内联 3 航点、B 内联为空），装配后 A 的 `MissionComponent` 持有那 3 个航点、B 回退到全局 `MissionProfile` 的航点。
- [ ] 装配后两架 `MissionComponent` 均已启动（状态非 Idle），各自航点独立。
- [ ] Web 端经反向控制通道 POST 多机 + 每机航线 DTO 触发热重载后，每架无人机按各自航线飞行（外部可观察：ndjson 的 `waypoint` 行按 Agent 区分、各机 trace 走向不同）。
- [ ] 现有"全局 `MissionProfile` 装配到 Leader"的行为在 Agent 内联为空时仍然成立（向后兼容）。
- [ ] 热重载含多机异航线的场景，状态干净（无上一场景残留航点/任务状态）。
- [ ] automation 单测覆盖：多机逐机航线装配、内联回退全局、向后兼容（参照 `ScenarioLoaderFleetTest` / `ScenarioLoaderObstaclesTest` 范式）。

## Blocked by

- `01-control-channel-and-hot-reload.md`（热重载内核与反向控制通道）
