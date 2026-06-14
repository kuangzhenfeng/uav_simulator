# UAV Simulator 领域语言

本文件是项目术语表（glossary），只定义本上下文特有的概念，不收录通用编程术语。每个术语给出唯一规范词，并列出应避免的同义词。

## 语言

### 场景核心

**Scenario（场景）**:
一份与渲染解耦的仿真场景数据资产（`UScenario`），声明式描述一次完整仿真的全部动态内容：障碍布局、风场、机队、任务航点、验收指标、随机种子。运行时由 ScenarioLoader 装配成"世界"。一个 Scenario 资产 = 一次可复现的仿真。
_Avoid_: FNMPCBaselineScenario（已废弃，见下）、关卡（Level）

**Scenario Asset（场景资产）**:
`UScenario`（`UDataAsset`）的实例，是组合引用式资产外壳——本身只持有对若干子资产（ObstacleLayout / WindProfile / FleetSetup / MissionProfile / AcceptanceCriteria）的引用 + 一个 RandomSeed。
_Avoid_: 场景配置、scenario config

**FNMPCBaselineScenario**:
历史遗留的纯 C++ 内存结构，仅服务于 NMPC 单元测试基准。**已不再扩展**，新场景一律走 `UScenario` 资产。术语上不得简称为"Scenario"。

**ScenarioLoader（场景装配器）**:
读取 `UScenario` 资产、按声明内容创建"世界"的组件：Spawn UAV 机队、注册障碍物、配置风场、下发任务航点、分发随机种子。是 Scenario 从"数据"变成"运行中世界"的唯一入口。挂在 MultiAgentGameMode 上。

**ScenarioEvaluator（场景验收器）**:
运行时组件，周期快照仿真指标（航点到达、最小净空、最大横向偏差、能耗），对照 `UAcceptanceCriteria` 判定 PASS/FAIL，结果写入 `Logs/scenario_result.json` 供 sim.sh / CI 读取。**不依赖进程正常退出**——靠周期快照兜底 pkill 强杀。

### 场景的组成（子资产）

**ObstacleLayout（障碍布局）**:
`UObstacleLayout` 子资产，描述障碍物的逻辑几何（类型 + 中心 + 尺寸 + 旋转 + 安全边距）及可选的运动轨迹。Loader 装配时 Spawn 可视化 Mesh 并注册到 `UObstacleManager`。静态地形（山脉/树林）不归此管，归关卡。
_Avoid_: 障碍列表、obstacle list

**WindProfile（风场档案）**:
`UWindProfile` 子资产，封装 `FWindConfig`（恒定风/阵风/Dryden 湍流参数）。Loader 装配到 **GameMode 持有的场景级 WindField**，全关卡共享。
_Avoid_: wind config、风场配置

**FleetSetup（机队配置）**:
`UFleetSetup` 子资产，含 `Agents[]` 显式数组（每个 Agent：UAV 蓝图子类 + 型号 + 初始位姿 + 是否 Leader）+ 可选编队参数。单机场景数组只有一个元素。
_Avoid_: 机群模板、fleet template

**MissionProfile（任务档案）**:
`UMissionProfile` 子资产，封装航点序列（`FMissionWaypoint[]`）+ 任务模式（Once/Loop/PingPong）。Loader 装配到机队的 `UMissionComponent` 并启动任务。
_Avoid_: 航点列表、waypoint list

**AcceptanceCriteria（验收标准）**:
`UAcceptanceCriteria` 子资产，定义"场景成功"的硬指标阈值：航点全部到达、最小净空、最大横向偏差、超时与能耗预算。ScenarioEvaluator 据此判定 PASS/FAIL。
_Avoid_: 测试标准、test criteria

### 边界概念

**Level（关卡 / `.umap`）**:
UE 地图文件，承载**静态美术**（地形、山脉、树林、灯光）。关卡是"舞台布景"，不含动态场景内容。ScenarioLoader 在空白关卡上 Spawn 全部动态内容。场景与关卡是正交的：同一张空白关卡可承载任意 Scenario。
_Avoid_: 用"场景"指代 Level（在本项目中"场景"专指 Scenario）

**ScenarioBuilder（场景构造器）**:
`FScenarioBuilder`——C++ 构造 API，用代码命令式地组装出 `UScenario` 资产并落盘。与"在编辑器 Details 面板手搓资产"是同一资产的两种入口，等价。

**Dynamic Obstacle（动态障碍）**:
带运动轨迹的障碍物（匀速直线 或 航点循环往返）。Loader Spawn `ADynamicObstacleActor`，按轨迹驱动 Mesh 并同步 `UObstacleManager` 中的 Velocity。

## 上下文关系图

```
UScenario (资产外壳)
├── 引用 UObstacleLayout   ──► ObstacleManager + Spawn Mesh
├── 引用 UWindProfile      ──► GameMode 场景级 WindField
├── 引用 UFleetSetup        ──► Spawn AUAVPawn 机队
├── 引用 UMissionProfile    ──► 机队 MissionComponent
├── 引用 UAcceptanceCriteria ──► ScenarioEvaluator
└── RandomSeed             ──► master FRandomStream → 派生子种子
```

**装配链路**: 命令行 `-Scenario=<资产路径>` → GameMode.BeginPlay 解析 → ScenarioLoader.Load() → 装配各子系统 → ScenarioEvaluator 周期快照 → sim.sh 读 JSON 判定退出码。
