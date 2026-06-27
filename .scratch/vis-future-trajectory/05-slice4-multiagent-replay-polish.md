# Slice 4：多机/降频健壮性 + 回放一致性 + 验收打磨

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **类型**: AFK
- **Parent**: `.scratch/vis-future-trajectory/01-future-trajectory-stream.md`

## What to build

在 Slice 1/2/3 三类未来轨迹全链路打通后，做横切的健壮性、一致性收尾与验收打磨，把"能跑"提升到"工业级可靠 + 文档同步"。

端到端行为：多机场景下每架机的三类未来轨迹各自按 agent id 正确关联、颜色沿用各自主色系；长时仿真下 `telemetry.ndjson` 体积可控；回放拖拽时间轴时未来轨迹与历史轨迹/飞机本体时间对齐；README 与 CLAUDE 涉及处文档同步。

### 关键实现约束

- **多机关联**：三类未来轨迹均按 `agent` id 关联到对应 agent 对象；多机场景下未来轨迹颜色沿用 `server.py` 的 `PALETTE`（各 agent 主色），与历史 trail 区分仅靠样式（透明度/虚线）而非色相，保证多机可分辨。验证至少 2 架机同时有未来轨迹的场景。
- **降频/限点健壮性**：复核三类未来轨迹的写入节拍与每轨迹点上限，确保长时（数百秒）仿真下 `telemetry.ndjson` 增量在可接受范围（量级参照现有 metrics 行）。高频路径用 `UE_LOG_THROTTLE`，禁止 Verbose。
- **回放一致性**：回放（`cursorT`）时未来轨迹显示"游标时刻最近一次快照"（消费端只留最新一份天然满足）；验证拖拽时间轴时未来轨迹与历史轨迹、飞机本体保持时间一致，不错位。
- **开关默认值**：默认优化轨迹开启、规划路径与 NMPC 预测关闭，避免初始信息过载（符合"最少惊讶"）。
- **向后兼容**：未知 ndjson type 安全忽略（已有行为，复核不回归）。
- **验收级验证**：C++ 单测 + Python 单测全绿后，跑一次完整 `sim.*` 仿真，Web 人眼核对三类未来轨迹出现、可独立开关切换、颜色/图例正确、多机正确关联。
- **文档**：按 CLAUDE.md 要求更新 README.md 功能特性描述（最小化修改、保持简洁）。

## Acceptance criteria

- [ ] 多机（≥2）场景下，各 agent 的三类未来轨迹按 id 正确关联、颜色随各自主色系，互不错乱
- [ ] 长时仿真下 `telemetry.ndjson` 体积在可接受范围（含未来轨迹增量）
- [ ] 回放拖拽时间轴时未来轨迹与历史轨迹/飞机本体时间一致，无错位
- [ ] 三类未来轨迹默认开关合理（优化轨迹开，其余关）
- [ ] 未知 ndjson type 安全忽略，不抛异常
- [ ] C++ 单元测试与 Python 单元测试全绿（含三类未来轨迹用例）
- [ ] 跑一次完整仿真，Web 人眼核对三类未来轨迹显示/开关/图例/多机关联正确
- [ ] README.md 功能特性描述已更新（最小化修改）
- [ ] 编译成功，无新增 warning

## Blocked by

- `.scratch/vis-future-trajectory/02-slice1-tracer-bullet.md`（Slice 1）
- `.scratch/vis-future-trajectory/03-slice2-planned-path.md`（Slice 2）
- `.scratch/vis-future-trajectory/04-slice3-nmpc-prediction.md`（Slice 3）
