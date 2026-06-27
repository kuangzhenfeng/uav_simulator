# 风场 + 验收 + 仿真控制配置

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **来源**: PRD `.scratch/web-control-panel/00-prd.md` 切片 4

## Parent

PRD：`.scratch/web-control-panel/00-prd.md`。设计文档：`docs/design/web-control-panel.md`（§4.3、§7.1、§11 DTO）。

## What to build

把"场景上下文"的其余配置维度补齐，使 Web 端能配置完整场景：风场、验收阈值、仿真控制参数。完成本切片后，一个完整 Scenario 的所有子资产维度都可在 Web 端配置并经热重载生效。

端到端行为：Web 端配置风场（类型/稳态风/阵风/湍流）、验收阈值（净空/横向偏差/超时/航点半径/能耗）、仿真控制（slomo/时长/控制模式/MPC 类型/随机种子）→ 触发热重载 → 全部生效。

三个配置族的处理：

- **风场**：`FWindConfig` 已 runtime-safe（`UWindField::SetWindConfig`，见 ADR-0002 场景级单例）。本切片只需把 DTO 风场字段经 `UScenarioFactory` 构造成 `UWindProfile`，装配阶段下发到 GameMode 持有的 WindField 即可。运行中实时改风场的能力属于切片 5。
- **验收阈值**：DTO 字段构造 `UAcceptanceCriteria`，`ScenarioEvaluator` 据此判 PASS/FAIL（fail-closed 语义不变）。
- **仿真控制参数**：DTO 字段构造。其中 **slomo 需新增运行时 Setter**（当前无 C++ 运行时 slomo 入口，仅靠 `-ExecCmds=slomo`）—— 新增走 `WorldSettings->SetTimeDilation`，与 `TelemetryRecorder` 读时标的口径一致。控制模式 / MPC 类型作为装配参数（场景级一次性应用）；运行中切换属切片 5。

DTO 三个族字段经 `UScenarioFactory` 统一构造为内存 `UScenario` 的对应子资产。

## Acceptance criteria

- [ ] Web 端配置不同风场类型（None/Constant/Gust/Turbulent）及参数触发热重载后，WindField 配置已更新（外部可观察：`TelemetryRecorder` 的 `wind_config` 行反映新配置、各机感受到对应风）。
- [ ] Web 端配置不同验收阈值触发热重载后，`ScenarioEvaluator` 用新阈值判 PASS/FAIL（外部可观察：同样指标在不同阈值下产出不同 verdict）。
- [ ] Web 端配置 slomo 触发热重载后，仿真时标按新值运行（外部可观察：`TelemetryRecorder` meta 行的 `slomo` 字段更新、仿真墙钟与仿真时间比值变化）。
- [ ] slomo 的运行时 Setter 与 `TelemetryRecorder` 读取时标的口径一致（同一处 `WorldSettings->GetEffectiveTimeDilation`）。
- [ ] Web 端配置控制模式 / MPC 类型 / 随机种子触发热重载后，机队按新控制模式与 MPC 类型装配、随机种子驱动可复现的派生子种子。
- [ ] 热重载含新风场/验收/仿真参数的场景后状态干净（验收器阈值/累积指标归零、风场相位重置复用切片 1 的 reset）。
- [ ] automation 单测覆盖：DTO→风场/验收/仿真控制子资产转换、slomo Setter 生效（参照 `WindFieldTest` / `ScenarioEvaluatorTest` / `ScenarioE2EWindAppliedTest` 范式）。

## Blocked by

- `01-control-channel-and-hot-reload.md`（热重载内核与反向控制通道；本切片复用其 reset 骨架，并新增 slomo 运行时 Setter）
