# 风场提升为场景级单例

我们决定把 `UWindField` 从 `AUAVPawn` 的子组件提升为 `AMultiAgentGameMode` 持有的场景级单例组件。`AUAVPawn::BeginPlay` 移除自建 WindField 的逻辑，改为从 GameMode 获取引用并转发给 `UAnemometerSensor`。ScenarioLoader 装配时设置 GameMode 风场的 `Config = UWindProfile.Config`。

## 为何提升

风场在物理上是**环境**，不是机载设备——所有 UAV 共享同一片风场。原架构中每个 UAVPawn 各自 `CreateDefaultSubobject<UWindField>`，多机场景下存在 N 个独立 WindField 实例：配置需逐机同步、Dryden 湍流状态各自演化（违反"同一片风"的物理事实）、`AnemometerSensor` 测的是"自己机上的风"而非"环境风"。这是架构债。

提升为 GameMode 单例后：风场作为环境存在，全关卡共享一个实例，物理正确，且场景化配置（`UWindProfile`）有唯一落点。

## 考虑过的替代

- **独立 `AEnvironmentActor` 持有 WindField**：职责更"纯"（环境归环境），但需新增一个 Actor 类、关卡里手动放置。鉴于 WindField 是当前唯一的环境组件，GameMode 单例已足够，避免过早抽象。若未来环境组件增多（降水、地效等），可再重构为 `AEnvironmentActor`。
- **不重构，只同步配置**：改动最小但保留架构债，违背"按主流工业级做法"原则，否决。

## 接线变更

`AUAVPawn` 现有逻辑（`CreateDefaultSubobject<UWindField>` + `BeginPlay` 中 `AnemometerSensor->SetWindField(WindFieldComponent)`）改为：

- 移除 `WindFieldComponent` 子组件创建；
- `BeginPlay` 中 `GetAuthGameMode<AMultiAgentGameMode>()->GetWindField()` 获取场景级引用；
- 转发给 `AnemometerSensor`；
- 物理积分中引用 `WindFieldComponent` 处改为 GameMode 引用（需缓存）。

## Consequences

- 单机场景的 GameMode 也必须持有 WindField（即便不用多机），否则单机 UAVPawn 取不到引用。因此**所有场景关卡统一使用 `AMultiAgentGameMode`**（项目当前 `DefaultEngine.ini` 已设为全局 GameMode）。
- 向后兼容：原有 UAVPawn 上暴露 `WindFieldComponent` 的接口需移除或转发到 GameMode，调用方需同步更新。

## Status

Accepted. 2026-06-14.
