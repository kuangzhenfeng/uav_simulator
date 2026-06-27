# 障碍配置（静态 + 动态巡逻）

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **来源**: PRD `.scratch/web-control-panel/00-prd.md` 切片 3

## Parent

PRD：`.scratch/web-control-panel/00-prd.md`。设计文档：`docs/design/web-control-panel.md`（§4.2 动态障碍运动模型、§8 动态障碍驱动器）。

## What to build

扩展障碍数据模型，让 Web 端既能配置静态障碍（数量/类型/位置/尺寸/安全边距），又能配置动态障碍并让它们沿巡逻路径真正动起来（领域词汇：Dynamic Obstacle，见 CONTEXT.md）。

端到端行为：Web 端配置若干静态障碍 + 若干动态障碍（选运动模式 + 巡逻点 + 速度）→ 触发热重载 → 静态障碍就位、动态障碍沿巡逻点运动，NMPC/CBF 视其为动态约束。

核心 schema 决策（来自设计文档原型，内联）：

```
enum EObstacleMovementType { Static, LinearVelocity, PatrolLoop, PatrolPingPong }

FScenarioObstacleEntry {
    // 既有：Type, Center, Extents, Rotation, SafetyMargin, bIsDynamic, Velocity
    + EObstacleMovementType MovementType = Static
    + TArray<FVector> PatrolPoints       // 巡逻航点（PatrolLoop/PingPong）；[0] 缺省=Center
    + float              PatrolSpeed = 300  // cm/s
}
```

新增 `ADynamicObstacleActor`：对 `PatrolLoop`/`PatrolPingPong` 沿 `PatrolPoints` 按 `PatrolSpeed` 推进位置，`LinearVelocity` 匀速 + 出界处理。复用既有 `UObstacleManager::UpdateDynamicObstacles`（每帧从 LinkedActor 同步位置反算速度），NMPC/CBF 自然视为动态约束。

`ScenarioLoader::AssembleObstacles` 对 `MovementType != Static` 的条目额外 Spawn `ADynamicObstacleActor` 并关联到 `ObstacleManager`。热重载时由 `UScenarioRuntimeController` 先 Stop 再 Destroy 全部动态障碍 Actor（补充到切片 1 的 reset 骨架）。

DTO 扩展障碍字段（静态组：类型/中心/尺寸/安全边距；动态组：类型/中心/尺寸 + 运动模型/速度/巡逻点），经 `UScenarioFactory` 构造。

## Acceptance criteria

- [ ] Web 端配置静态障碍（不同 Type：Sphere/Box/Cylinder、不同 Center/Extents/SafetyMargin）触发热重载后，障碍正确就位并注册到 `ObstacleManager`。
- [ ] Web 端配置 `PatrolLoop` 动态障碍（2 个巡逻点 + 速度）触发热重载后，`ADynamicObstacleActor` 沿巡逻点循环运动，`ObstacleManager` 每帧同步其位置并反算出非零速度。
- [ ] `PatrolPingPong` 动态障碍在到达巡逻点末端后反向运动（往返）。
- [ ] 运动中的动态障碍被 NMPC/CBF 视为动态约束（外部可观察：无人机对运动障碍有避让响应、clearance 随障碍接近而下降）。
- [ ] 热重载含动态障碍的场景后，旧动态障碍 Actor 全部销毁、无残留（验证：重载后关卡中 `ADynamicObstacleActor` 数量与新配置一致）。
- [ ] automation 单测覆盖：动态障碍运动模型装配、PatrolPoints 缺省回退 Center、`ADynamicObstacleActor` 沿巡逻点推进后的外部位置/速度（参照 `ObstacleManagerTest` / `ScenarioLoaderObstaclesTest` 范式）。

## Blocked by

- `01-control-channel-and-hot-reload.md`（热重载内核与反向控制通道；本切片需向 reset 骨架补充动态障碍 Actor 的 Stop/Destroy）
