# 场景系统资产化（Scenario-as-DataAsset）

我们决定把"仿真场景"抽象成 UE `UDataAsset`（组合引用式资产 `UScenario`），由 ScenarioLoader 在运行时装配，取代散落在 C++ 默认值 / 蓝图 / 关卡里的硬编码配置。命令行 `-Scenario=<资产路径>` 指定运行哪个场景，未指定时用 GameMode 的默认引用。

## 为何不用其他形态

- **JSON/INI 外部配置**（AirSim/PX4 原生做法）：失去 UE 编辑器 Details 面板可视化编辑、失去蓝图引用能力，且要自写反序列化。我们的场景需要编辑器内可视化调参，故放弃。
- **纯 C++ 内存结构**（沿用 `FNMPCBaselineScenario`）：适合单元测试基准，但不支持编辑器可视化、不支持跨场景复用子配置（同一套风场档案套不同布局）。运行时端到端场景不适合。
- **单一扁平资产**（所有子结构塞进一个 UScenario）：简单但子配置无法复用。

组合引用式资产（外壳引用多个可复用子资产）兼顾了可视化编辑、子配置复用、可 diff 进 git。对齐 NVIDIA Isaac 的 world + sub-config 组合模式。

## 为何统一为单一 UScenario 概念

项目原有 `FNMPCBaselineScenario`（NMPC 单元测试基准）与新建的运行时 `UScenario` 两者都叫"Scenario"，造成术语歧义。决策：**`UScenario` 是 canonical 术语**；`FNMPCBaselineScenario` 标注为历史遗留、不再扩展，新场景一律走资产。单元测试可经由 `FScenarioBuilder` 构造资产后加载，与运行时走同一装配路径。

## 双入口

同一份 `UScenario` 资产支持两种构造入口，产出等价：

1. **编辑器 Details 面板**：手动拖拽子资产引用、调参、另存为 `.uasset`。适合人工探索性调场景。
2. **`FScenarioBuilder` C++ API**：命令式构造 `UScenario` 并 `SaveAsset` 落盘。适合批量程序化生成场景库（CI、回归矩阵）。

## 装配入口

ScenarioLoader 挂在 `AMultiAgentGameMode` 上（关卡单例 + 现有 RegisterAgent 中枢）。`GameMode::BeginPlay` 解析命令行 `-Scenario=`（优先），否则用 `DefaultScenario` 资产引用。装配顺序：风场 → 障碍 → 机队 Spawn → 任务下发 → 验收器启动。

## Status

Accepted. 2026-06-14.
