# Web 控制面板设计方案（Web Control Panel Design）

> 状态：草案 / 待评审
> 日期：2026-06-27
> 关联：ADR-0001（场景资产化）、ADR-0002（风场场景级单例）

## 1. 背景与目标

当前可视化系统是**纯只读**的：

- `Tools/vis/server.py`（Python，`:8765`）只 `tail` 解析 `Logs/telemetry.ndjson` + `scenario_result.json`，通过 `GET /api/data` 与 `GET /api/stream`(SSE) 单向推给前端。
- 前端 `Tools/vis/web/app.js`（原生 ES Module + Three.js，零构建零依赖）只做 3D 回放、折线图、判决面板，**没有任何输入控件、POST、控制能力**。
- UE 侧 `Source/` 完全没有 HTTP/Socket/网络代码，`Tools/vis/server.py` 只有 `do_GET`。

目标（用户明确诉求 + 推导出的必要补充）：

| 能力 | 说明 |
|---|---|
| **触发重跑** | Web 端一键重跑，**进程内热重载**（不重启 UE，秒级响应） |
| **障碍物配置** | 静态障碍数量/位置/尺寸；**动态障碍沿巡逻路径运动** |
| **多机机队配置** | 无人机数量、每架机型/初始位姿 |
| **每机独立航线** | 每架无人机独立航点序列（当前 schema 缺口，需补） |
| 仿真控制参数 | 时标 slomo、控制模式、MPC 类型、时长 |
| 实时调参 | 仿真运行中即时调 PID/NMPC/CBF-QP/风场，无需重跑 |
| 批量 A/B 对比 | 多套配置连续跑并比对（架构预留，二期落地） |

设计原则（遵循 CLAUDE.md）：主流工业级做法、headless 可用、sh/bat 一致、sh/bat 与现有 `sim.*` 对齐、最小化 README 改动、日志英文注释中文、不主动提交。

## 2. 关键现状约束（影响设计的硬事实）

1. **配置单一数据源 = `UScenario` DataAsset**（ADR-0001），经 `UScenarioLoader` 装配。`AMultiAgentGameMode::LoadAndAssembleScenario()` 解析 `-Scenario=` 命令行，回退 `DefaultScenario`。**无 DataTable**。
2. **航线只装配到 `Fleet[0]`** —— `ScenarioLoader::AssembleFleetAndMission` 把 `MissionProfile` 仅下发给 Lead。这是"每机航线"的核心缺口。
3. **动态障碍 schema 预留了 `Velocity` 字段但未启用**（注释"二期"）。`UObstacleManager::UpdateDynamicObstacles` 已有"每帧从 LinkedActor 同步位置反算速度"的运行时逻辑，只差"让 LinkedActor 动起来"的驱动器。
4. **进程内重跑能力完全缺失** —— 无场景级 Reset 编排者；`ScenarioEvaluatorComponent`(bFinalFlushed/Accumulated)、`TelemetryRecorder`(FileHandle/bStaticWritten)、`WindField`(阵风相位) 等持久组件无 reset 接口，跨场景会污染。
5. **CBF-QP 配置 `private` 无 Setter**（`UAVPawn.h:421`），外部不可访问；**slomo 无 C++ 运行时 Setter**（仅靠 `-ExecCmds=slomo`）。其余 PID/NMPC 字段大多 `BlueprintReadWrite`，运行时可直接赋值（已有 `UControlParameterTuner` 的 `pid.set` 先例）。
6. **`uav_simulator.Build.cs` 无任何网络模块依赖**；但插件 `SoftUEBridge` 已含 `HTTP/HTTPServer/Sockets/Networking/Json/JsonUtilities` 的完整依赖模板，且其 `FBridgeServer` 是成熟的 HttpRouter 用法范例（CORS、GameThread 切换、RAII guard）。
7. **headless(`-game -NullRHI`)下热重载可行**：飞行物理 Tick 不依赖渲染线程，`DrawDebug*`/HUD Canvas 在无 RHI 下自然 no-op。但 `SoftUEBridge` 在 `-unattended` 会跳过启动 —— 故不能直接复用其进程，需新起独立服务端。

## 3. 总体架构

新增一条**反向控制通道**，与现有正向遥测通道解耦：

```
┌──────────────┐   控制命令(POST)    ┌─────────────┐   HTTP 转发     ┌──────────────────┐
│  浏览器前端   │ ──────────────────▶ │ Python 8765  │ ──────────────▶ │  UE HTTP 控制端   │
│ (app.js+面板) │ ◀──────────────────│ (server.py)  │ ◀──────────────│  (UHttpControl)  │
│              │    SSE 遥测(实时)    │  反向代理+   │   状态回执      │   :8770           │
└──────────────┘                     │  遥测聚合     │                 │   GameThread      │
        ▲                            └──────┬───────┘                 └────────┬──────────┘
        │                                   │ tail                              │
        │                                   ▼                                   │ 读写
        │                          Logs/telemetry.ndjson  ◀────────────────────│
        │                          Logs/scenario_result.json                    │
        │                                                                       │
        │                              ┌────────────────────────────────────────┘
        │                              ▼
        │                   ┌─────────────────────┐    装配/重置    ┌──────────────┐
        └──────────────────│ UScenarioRuntime     │ ─────────────▶ │ UScenarioLoader│
                            │ Controller(GameMode) │                │ UScenario(内存)│
                            │  - ReloadScenario    │                └──────────────┘
                            │  - SetSlomo/Wind/... │
                            └─────────────────────┘
```

**职责分离**：

- **前端**：人机交互、表单、实时调参滑块、3D 回放。只与 `:8765` 通信（**单一入口，避免跨端口 CORS**）。
- **Python(:8765)**：既保留原遥测职责（tail ndjson + SSE），又新增**控制命令反向代理**：校验 → 转发到 UE `:8770` → 回执；并聚合 UE 状态注入遥测快照。
- **UE HTTP 控制端(:8770)**：独立于 SoftUEBridge，headless 启动，把 HTTP 命令切到 GameThread 执行，调用 `UScenarioRuntimeController`。
- **UScenarioRuntimeController**：进程内热重载编排者 + 实时调参入口，挂在 GameMode 上，持有 `ActiveScenario`/`ScenarioFleet`/各子组件句柄。

**端口规划**：`:8765`（可视化/前端入口，已有）、`:8080`（SoftUEBridge 编辑器，避开）、`:8770`（新增 UE 控制端）。

## 4. 数据模型扩展（Schema，`ScenarioTypes.h`）

web 配置**不落盘成 `.uasset`**：运行时 `NewObject<UScenario>` 构造内存对象喂给现有 `UScenarioLoader`，避免资产序列化/烹饪/版本库污染。因此扩展以"内联值"为主，减少对独立子资产软引用的依赖。

### 4.1 每机独立航线（核心缺口补齐）

在 `FScenarioAgentEntry` 内联航点，装配时"每机优先用自己航点，缺失回退全局 MissionProfile"：

```cpp
USTRUCT(BlueprintType)
struct FScenarioAgentEntry {
    // 既有字段...
    UPROPERTY(...) TSubclassOf<AUAVPawn> UAVClass;
    UPROPERTY(...) EUAVModelID ModelID;
    UPROPERTY(...) FVector InitialPosition;
    UPROPERTY(...) float InitialYaw;
    UPROPERTY(...) bool bIsLeader;
    // ▼ 新增：每机内联航线（空数组表示回退全局 MissionProfile）
    UPROPERTY(...) TArray<FMissionWaypoint> Waypoints;
    UPROPERTY(...) EMissionMode MissionMode = EMissionMode::Once;
};
```

`ScenarioLoader::AssembleFleetAndMission` 改为按索引逐机装配：`Entry.Waypoints` 非空则用之，否则回退 `Scenario->MissionProfile`。

### 4.2 动态障碍运动模型（巡逻路径）

扩展 `FScenarioObstacleEntry`：

```cpp
UENUM(BlueprintType)
enum class EObstacleMovementType : uint8 {
    Static,          // 静态
    LinearVelocity,  // 恒速直线（复用现有 Velocity 字段）
    PatrolLoop,      // 沿 PatrolPoints 循环
    PatrolPingPong   // 沿 PatrolPoints 往返
};

USTRUCT(BlueprintType)
struct FScenarioObstacleEntry {
    // 既有字段...
    UPROPERTY(...) EObstacleType Type;
    UPROPERTY(...) FVector Center;
    UPROPERTY(...) FVector Extents;
    UPROPERTY(...) bool bIsDynamic;       // 保留，由 MovementType!=Static 隐含
    UPROPERTY(...) FVector Velocity;      // LinearVelocity 模式用
    // ▼ 新增
    UPROPERTY(...) EObstacleMovementType MovementType = EObstacleMovementType::Static;
    UPROPERTY(...) TArray<FVector> PatrolPoints;  // 世界坐标航点（PatrolLoop/PingPong）
    UPROPERTY(...) float PatrolSpeed = 300.f;     // cm/s
};
```

> `PatrolPoints[0]` 缺省即 `Center`，web 端可只给相对偏移、由后端补绝对坐标。

### 4.3 运行时 Scenario 构造辅助

新增 `UScenarioFactory`（静态工具，`Scenario/` 目录）：提供 `BuildScenarioFromDto(const FScenarioDto&)` —— 把 web 传来的 JSON DTO（见 §11）构造为内存 `UScenario` + 子 `UObstacleLayout`/`UFleetSetup`/`UWindProfile`/`UMissionProfile`，供 `UScenarioLoader` 消费。与 `UAVModelSpec` 联动填充默认机型参数。

## 5. 进程内热重载（核心模块）

### 5.1 新增编排者 `UScenarioRuntimeController`

挂在 `AMultiAgentGameMode` 上，与 `ScenarioEvaluatorComponent` 同级（`UPROPERTY(Transient) TObjectPtr`，`LoadAndAssembleScenario` 里 `NewObject+RegisterComponent`）。职责：

- 持有当前 `ActiveScenario`、`ScenarioLoaderInstance`、`ScenarioFleet` 句柄引用（从 GameMode 透传）。
- 提供 `ReloadScenario(UScenario*)`：销毁旧场景 → 重置持久组件 → 重新装配。
- 提供实时调参入口（§7）。

### 5.2 必须新增的 reset 接口（探索已逐一定位）

| # | 新增接口 | 文件 | 作用 |
|---|---|---|---|
| 1 | `AUAVPawn::EndPlay(EEndPlayReason)` override | `Core/UAVPawn.h/.cpp` | 销毁时 `UnregisterAgent` + 清残骸 Actor + 释放 WindField 弱引用（当前全仓仅 `ControlParameterTuner` 实现了 EndPlay，Pawn 没有） |
| 2 | `AUAVPawn::ResetMetrics()` | `Core/UAVPawn.h` | Metrics*（MaxVelocity/CrossTrackDev/Roll/Pitch/InstabTime/Stuck/ForceComplete/LastSimEvent/SimEventSeq）归零 |
| 3 | `AMultiAgentGameMode::ResetForScenarioReload()` | `MultiAgent/AgentManager.h/.cpp` | 总调度：销毁机队 → 清注册表/缓存 → 重置子组件 → 重装配 |
| 4 | `UScenarioEvaluatorComponent::Reset()` | `Scenario/ScenarioEvaluator.h/.cpp` | 清 `Accumulated`(MinClearance=FLT_MAX/MaxLat=0)、`ElapsedTime`、`bFinalFlushed`、解绑旧 Mission 委托 |
| 5 | `UTelemetryRecorder::ResetForNewScenario()` | `Telemetry/TelemetryRecorder.h/.cpp` | 关旧 FileHandle → truncate 重开 → 清 `bStaticWritten`/`FrameAccum`/`MetricsAccum`/`LastEventSeq`（直接抽自 BeginPlay :65-68） |
| 6 | `UWindField::ResetDynamicState()` | `Environment/WindField.h/.cpp` | 清 `bGustActive`/`GustTimer`/`CurrentGustVelocity`/`TargetGustVelocity`/`DrydenStateX`（`SetWindConfig` 不会重置这些阵风相位） |
| 7 | 清 `BTTask_ExitSimulation` 延迟退出 Timer | `AgentManager.cpp` ResetFor 内 | 该 Task 用 `SetTimer` 延迟 `RequestExit(false)`，热重载前必须 `ClearAllTimersForObject`，否则误杀进程 |
| 8 | `TaskAllocator/TaskMonitor` Reset 或重建 | `MultiAgent/Task*.h` | 二者随 GameMode 持久、不随 Pawn 销毁，必须显式清状态（先确认其 Reset 接口，无则 NewObject 重建） |

### 5.3 重置调用顺序（`ResetForScenarioReload`）

```
ResetForScenarioReload()
 ├─ Clear ExitSimulation Timer           (防 RequestExit 误杀)
 ├─ StopTeardown 动态障碍驱动器            (§10)
 ├─ for Pawn in ScenarioFleet:
 │     Pawn->Destroy()                    (触发新增 EndPlay: 反注册+残骸+WindField)
 ├─ AgentRegistry / StateCache / JointNMPCResultCache / ScenarioFleet.Empty()
 ├─ NextAgentID = 0                       (语义对齐进程重启，与 telemetry agent 字段一致)
 ├─ JointNMPCSolveAccumulator = StateCacheAccumulator = 0
 ├─ CachedFormationType = None            (强制重算)
 ├─ ScenarioEvaluatorComponent->Reset()
 ├─ TelemetryRecorder->ResetForNewScenario()
 ├─ WindField->ResetDynamicState()
 ├─ TaskMonitor/Allocator 重建或 Reset
 ├─ WindField->SetWindConfig(NewCfg)      (在 Spawn 前，保证积分前风场就绪)
 └─ LoadAndAssembleScenario()             (NewObject 新 Loader + AssembleWind/Fleet/Mission/Obstacles + 重建 Evaluator)
```

> `LoadAndAssembleScenario` 当前每次都 `NewObject` 新建 Loader/Evaluator，需先把旧 Evaluator `DestroyComponent`/Unregister，避免重复持有。

### 5.4 线程安全

- HTTP 控制端在**网络线程**收到命令 → `AsyncTask(ENamedThreads::GameThread, ...)` 切到 GameThread 执行 reload（仿 `SoftUEBridge/BridgeServer.cpp:179`）。
- reload 期间设置 `bReloading` 互斥标志，期间拒绝新命令并暂停 evaluator/recorder 的 Tick（避免在半装配状态采指标）。
- reload 完成后清标志，写一行 `reload` 事件到 ndjson，前端据此切换回放游标。

## 6. UE HTTP 控制端（新增模块 `UHttpControlComponent`）

### 6.1 依赖与构建

- `uav_simulator.Build.cs` 的 `PrivateDependencyModuleNames` 增加：`"HTTP"`, `"HTTPServer"`, `"Sockets"`, `"Networking"`, `"Json"`, `"JsonUtilities"`（模板照搬 `Plugins/SoftUEBridge/Source/SoftUEBridge/SoftUEBridge.Build.cs:21-49`）。
- 新增 `Network/HttpControlComponent.h/.cpp`，`UActorComponent`，挂在 GameMode（BeginPlay 末尾 `NewObject+RegisterComponent`，仿 `TelemetryRecorder`）。

### 6.2 实现要点（照搬 SoftUEBridge 成熟范式）

- 用 `FHttpServerModule::Get().GetHttpRouter(Port)` + `BindRoute(FHttpPath, EHttpServerRequestVerbs::VERB_POST, handler)`，端口默认 `8770`，可环境变量 `UAV_CTRL_PORT` 覆盖，占用时 +1..+9（仿 `SoftUEBridgeSubsystem`）。
- 端口写入 `Saved/.uav-ctrl/port.json` 供 Python 自动发现（仿 `.soft-ue-bridge/instance.json`）。
- **关键差异：headless 必须启动**。SoftUEBridge 在 `FApp::IsUnattended()` 跳过；本组件**显式不跳过**（命令面板正是为 `-game -NullRHI` 设计）。
- CORS：`Access-Control-Allow-Origin: *`（与 Python 反代二选一，倾向 Python 反代统一入口，则此端无需 CORS）。
- 处理器用 RAII `FUnattendedScriptGuard` 抑制模态对话框（仿 `BridgeServer.cpp:305`），所有副作用包在 `AsyncTask(GameThread, ...)` 内。

### 6.3 路由设计

| Method | Path | 作用 |
|---|---|---|
| POST | `/control/reload` | 接收 `FScenarioDto` → `UScenarioFactory` 建内存 Scenario → `RuntimeController->ReloadScenario` |
| POST | `/control/slomo` | `{scale}` → `SetTimeDilation` |
| POST | `/control/wind` | `{config}` → `WindField->SetWindConfig` |
| POST | `/control/params` | 实时调参（见 §7），按 `scope`/`target` 分发到 fleet 或单机 |
| POST | `/control/exit` | 主动结束仿真（写终局判决后 RequestExit） |
| GET | `/control/status` | `{state: idle/running/reloading, fleet[], elapsed, scenario}` 给 Python 聚合 |

## 7. 实时调参（无需重跑）

利用既有 `BlueprintReadWrite` 字段直接赋值（已有 `ControlParameterTuner.cpp:197` 的 `pid.set` 先例），无需大改底层。统一经 `/control/params` 下发。

### 7.1 调参目标与落点

| 参数族 | 落点 | 现状 | 需补接口 |
|---|---|---|---|
| 姿态 PID(Kp/Ki/Kd × Roll/Pitch/Yaw) | `AttitudeController->Roll/Pitch/YawPID` | `BlueprintReadWrite` | 薄封装 `SetAttitudePID` |
| 位置/速度环 PID | `PositionController->Kp/Ki/Kd_Position/Velocity` | `BlueprintReadWrite` | 薄封装 |
| NMPC 全配置 | `NMPCAvoidance->Config`(public) | 可直接赋值 | 封装 `SetNMPCConfig` |
| CBF-QP 配置 | `UAVPawn->CBFQPConfig` | **private 无 Setter(GAP)** | **必须改 public 或加 Setter** (`UAVPawn.h:421`) |
| 控制模式 | `UAVPawn->SetControlMode` | 已可用 | 仅接 Web 链路（逐机或全队） |
| MPC 类型 | `NMPCAvoidance->Config.MPCType` | 懒加载已存在(`UAVPawn.cpp:619`) | 薄封装 `SetMPCType` |
| 风场 | `WindField->SetWindConfig` | 已 runtime-safe | 经 `/control/wind` |
| slomo | `WorldSettings->SetTimeDilation` | **无 Setter(GAP)** | **新增 `SetSlomo`** |

### 7.2 应用粒度

调参命令带 `target`：`"fleet"`（全体同步）、`"leader"`、`{agentId}`。`RuntimeController` 遍历 `ScenarioFleet` 过滤后逐机 apply。CBFQPConfig 改 public 后，apply 即 `Pawn->CBFQPConfig = NewCfg; Pawn->CBFQPFilter->ApplyConfig()`。

### 7.3 反馈

apply 成功后写一行 `param` 事件到 ndjson（agent/scope/key/old/new），前端面板高亮变化的参数。调参不计入判决指标，不触发 reload。

## 8. 动态障碍驱动器（新增 `UDynamicObstacleDriver`）

让 `FScenarioObstacleEntry.MovementType != Static` 的障碍真正动起来，复用既有 `ObstacleManager::UpdateDynamicObstacles` 的"每帧同步 LinkedActor"逻辑。

- 装配阶段：`ScenarioLoader::AssembleObstacles` 对 `PatrolLoop/PingPong` 障碍额外 `SpawnActor` 一个轻量 `ADynamicObstacleActor`（带 `USplineComponent` 或自维护航点数组 + 速度插值），`PatrolPoints` 喂入，初始位置=`PatrolPoints[0]`。`LinearVelocity` 模式则 `ADynamicObstacleActor` 自带匀速 + 出界销毁/循环。
- `ADynamicObstacleActor::Tick`：按 `PatrolSpeed` 沿 `PatrolPoints` 推进位置（PingPong 反向）；位置更新写入根组件。
- 关联：与静态障碍一致走 `LinkedActor` 机制，`ObstacleManager` 每帧 `UpdateDynamicObstacles` 自然拿到新位置并反算速度，NMPC/CBF 视其为动态约束。
- 热重载时由 `ResetForScenarioReload` 先 Stop 再 Destroy 所有 `ADynamicObstacleActor`（持有在 `TArray` 便于清理）。

## 9. Python 后端扩展（`Tools/vis/server.py`）

新增**控制反向代理**层，保持前端单一入口 `:8765`，避免跨端口 CORS。

- 启动时读 `Saved/.uav-ctrl/port.json` 获知 UE 控制端端口；读不到则标记"控制不可用"，面板置灰。
- 新增路由（在现有 `do_GET` 基础上加 `do_POST`）：

| Method | Path | 作用 |
|---|---|---|
| GET | `/api/control/status` | 代理 `/control/status` + 探活 |
| POST | `/api/control/reload` | 透传 `ScenarioDto` → UE `/control/reload`，返回 reload 令牌 |
| POST | `/api/control/*` | 透传 slomo/wind/params |
| GET | `/api/presets` / POST `/api/presets` | 预设存取（落 `Tools/vis/presets/*.json`，前端可存档/加载/导出） |

- 前端 SSE 现有 `done` 事件外，新增 `reload` 事件：UE reload 完成写 ndjson 的 `reload` 行 → Python feed → SSE 推 → 前端重置游标、刷新场景静态数据（障碍/航点/机队）。
- 控制命令加**基本校验**（无人机数 1..N、障碍数、坐标范围、速度正数），非法返回 400，防 UE 崩溃。

## 10. 前端控制面板（`Tools/vis/web/`）

延续**零构建原生 ES Module** 路线，新增左侧/底部控制区，不破坏现有 3D+图表布局。

### 10.1 布局

```
┌─────────────────────────────────────────────────────┐
│ Topbar  [Run ▶ Rerun ⟳]  status  verdict           │
├──────────┬──────────────────────────┬───────────────┤
│ 左侧     │   3D Scene               │  右侧         │
│ 场景配置 │   (可点击拾取坐标→填表)  │  实时面板     │
│ Tab:     │                          │  Tab:         │
│ · 机队   │                          │  · 指标/事件  │
│ · 障碍   │                          │  · 实时调参   │
│ · 航线   │                          │  · 预设库     │
│ · 风/验收│                          │               │
│ · 仿真   │                          │               │
├──────────┴──────────────────────────┴───────────────┤
│ Timeline ▶  ──●──────  t=12.3s  wind=...           │
└─────────────────────────────────────────────────────┘
```

### 10.2 配置 Tab（构建 `ScenarioDto`）

- **机队 Tab**：动态增删 UAV 行（+/-），每行选 `ModelID`(下拉 5 个型号)、初始位置(X/Y/Z)、朝向、`bIsLeader`；展开"航线"内联编辑该机航点序列（复用现有航点 UI 逻辑）。空航点 = 跟随全局/编队。
- **障碍 Tab**：分"静态"/"动态"两组。静态：增删行，选 Type(Sphere/Box/Cylinder)、Center、Extents、SafetyMargin。动态：选 MovementType(Linear/PatrolLoop/PatrolPingPong)、PatrolSpeed、PatrolPoints 列表（可在 3D 视图点击拾取）。
- **风/验收 Tab**：风类型(None/Constant/Gust/Turbulent)、SteadyWindVelocity、Gust/Turbulence 参数；验收阈值(MinClearance/MaxLateralDev/Timeout/WaypointRadius/EnergyBudget)。
- **仿真 Tab**：slomo(0.1~N)、时长(duration)、控制模式、MPC 类型、RandomSeed。

### 10.3 实时调参 Tab

- 运行中可编辑滑块：姿态/位置/速度 PID、NMPC 权重与避障距离、CBF-QP DSafe/Alpha、风场参数、slomo。
- `target` 选择器（全队/Leader/单机）。提交即 POST `/api/control/params`，无需 rerun。
- 历史记录条：每次调参记一行，支持回滚到上一组。

### 10.4 预设库 / A-B 对比（二期预留）

- 预设存取走 `/api/presets`。A/B 对比：保存两套配置，依次 rerun，并排展示判决/指标；架构在 DTO 层天然支持，二期补前端对比视图。

### 10.5 交互拾取

3D 视图支持"拾取模式"：点击地面/障碍填入表单坐标（坐标系用 server.py 既有 `to_web()` 逆变换，m 右手 → cm 左手）。这是降低配置门槛的关键。

## 11. DTO 协议（`ScenarioDto`，JSON）

前端 ↔ Python ↔ UE 的统一数据契约。结构与扩展后的 `UScenario` 一一对应，新增 `UDTO` 结构体 + `FJsonObjectConverter`（`JsonUtilities`）做自动序列化（项目当前手写 ndjson，DTO 走自动转换器更工业级）。

```jsonc
{
  "name": "WebScenario",
  "randomSeed": 42,
  "sim": { "slomo": 8, "durationSec": 60, "controlMode": "Trajectory", "mpcType": "Nonlinear" },
  "wind": { "type": "Gust", "steady": [0,100,0], "gustAmplitude": 200, /*...*/ },
  "acceptance": { "minClearanceCm": 200, "maxLateralDevCm": 500, "timeoutSec": 120, /*...*/ },
  "fleet": [
    { "model": "Agri_AG20", "initPos": [0,0,500], "yaw": 0, "isLeader": true,
      "waypoints": [ {"pos":[0,1000,500],"speed":300}, /*...*/ ], "mode": "Once" },
    { "model": "Agri_AG20", "initPos": [500,0,500], "yaw": 0, "isLeader": false,
      "waypoints": [] }
  ],
  "obstacles": {
    "static": [
      { "type": "Box", "center":[0,2000,250], "extents":[500,500,250], "safetyMargin":50 }
    ],
    "dynamic": [
      { "type": "Cylinder", "center":[1000,2000,250], "extents":[200,200,250],
        "movement": "PatrolLoop", "patrolSpeed": 300,
        "patrolPoints": [[1000,2000,250],[1000,4000,250]] }
    ]
  }
}
```

坐标系约定：DTO 全程用 **UE cm 左手系**（与 ScenarioLoader 内部一致），仅在**前端展示层**做 m/右手变换，避免变换污染数据链。

## 12. 实施阶段划分

每阶段独立可验证、可提交，遵循 CLAUDE.md 工作流（改 C++ → 编译 → 仿真 → 看日志）。

**P1 — Schema + 每机航线（数据层）**
- 扩展 `ScenarioTypes.h`（`FScenarioAgentEntry.Waypoints`、`EObstacleMovementType`、`FScenarioObstacleEntry` 新字段）。
- 改 `ScenarioLoader::AssembleFleetAndMission` 逐机装配航线。
- 验证：改一个测试 Scenario 资产配多机异航线，`test.bat` 跑单测，仿真看 ndjson `waypoint` 行每机独立。

**P2 — 热重载内核（进程内 rerun）**
- 新增 §5 全部 reset 接口 + `UScenarioRuntimeController::ReloadScenario`。
- 先用控制台命令 `scenario.reload`（仿 `ControlParameterTuner` 的 `IConsoleCommand`）驱动，验证闭环。
- 验证：连跑两次不同配置，检查 ndjson truncate 重写、判决 `bFinalFlushed` 不卡死、AgentID 从 0 起、阵风相位重置。

**P3 — UE HTTP 控制端**
- 加 `Build.cs` 网络依赖，新增 `UHttpControlComponent` + 路由 + DTO 转换 + `UScenarioFactory`。
- 用 curl 验证各 POST 端点在 headless 下生效。
- 验证：`soft-ue-cli` 或 curl 发 reload/slomo/wind/params，看 UE 日志 + 行为。

**P4 — Python 反向代理**
- `server.py` 加 `do_POST` 透传 + `/api/control/status` + 探活。
- 验证：经 `:8765` 转发到 `:8770` 链路通，错误回执正确。

**P5 — 前端控制面板**
- 新增配置/调参/预设 UI，3D 拾取，SSE `reload` 事件处理。
- 验证：浏览器端完整跑"配置 → rerun → 看结果 → 实时调参"闭环。

**P6 — 动态障碍驱动器**
- `ADynamicObstacleActor` + 装配集成 + 热重载清理。
- 验证：PatrolLoop/PingPong 障碍按预期运动，NMPC 视为动态约束。

**P7（二期）— 预设库持久化 + A/B 对比视图**

## 13. 验证方案（端到端）

1. **编译**：`Script/build.bat`（Win）/ `Script/build.sh`（*nix）。
2. **单元测试**：`Script/test.bat`，新增 DTO<->Scenario 转换、每机航线装配、reset 接口单测。
3. **headless 仿真**：`Script/sim.bat` 启动 → UE 控制端 `:8770` 就绪（看 `Saved/.uav-ctrl/port.json`）→ curl 发 `/control/reload` 带不同 DTO → 观察 `Logs/telemetry.ndjson`（reload 行 + 新机队/障碍/航点）与 `Logs/scenario_result.json` 刷新。
4. **浏览器闭环**：`Tools/vis/vis.bat` 起 Python → 开 `http://127.0.0.1:8765` → 配置机队/障碍/航线/风 → 点 Rerun → 看实时 3D + 指标 → 运行中拖滑块调 PID/slomo → 看 ndjson `param` 事件与行为响应。
5. **回归**：不传任何控制命令时，现有只读可视化与 `sim.*` 退出码协议（0/1/2）行为不变。
6. **一致性**：确认 `Tools/vis/vis.bat` 与 `vis.sh` 行为一致；新增任何脚本双端同步。

## 14. 关键风险与对策

| 风险 | 对策 |
|---|---|
| 热重载遗漏某子系统导致状态污染 | §5.2 清单 + `ResetForScenarioReload` 单测覆盖；reload 互斥标志防半装配态采指标 |
| `BTTask_ExitSimulation` 延迟 Timer 误杀进程 | reload 前强制 `ClearAllTimersForObject`（§5.2 #7） |
| headless 下 HTTP 端点不启动 | 显式绕过 SoftUEBridge 的 unattended guard（§6.2） |
| 内存 Scenario 不落盘，UE 崩溃丢配置 | 前端预设存 Python 侧 `presets/*.json`；可选 `SoftUEBridge` 落盘调试 |
| 实时调参把 NMPC/CBF 调到发散导致 NaN | apply 前做值域校验；保留 `scenario.reload` 一键恢复 |
| 端口冲突 | 8770 占用自动 +1..+9，写 port.json 供发现；避开 8080(SoftUEBridge) |

## 15. 涉及文件清单（关键路径）

**UE C++（新增/修改）**
- `Source/uav_simulator/uav_simulator.Build.cs`（+网络依赖）
- `Source/uav_simulator/Scenario/ScenarioTypes.h`（schema 扩展）
- `Source/uav_simulator/Scenario/ScenarioLoader.{h,cpp}`（逐机航线 + 动态障碍装配）
- `Source/uav_simulator/Scenario/ScenarioFactory.{h,cpp}`（新增，DTO→Scenario）
- `Source/uav_simulator/Scenario/ScenarioEvaluator.{h,cpp}`（+`Reset()`）
- `Source/uav_simulator/Scenario/ScenarioRuntimeController.{h,cpp}`（新增，热重载编排）
- `Source/uav_simulator/Network/HttpControlComponent.{h,cpp}`（新增，HTTP 服务端）
- `Source/uav_simulator/Network/ScenarioDto.h`（新增，DTO）
- `Source/uav_simulator/MultiAgent/AgentManager.{h,cpp}`（挂组件 + `ResetForScenarioReload` + `SetSlomo`）
- `Source/uav_simulator/Core/UAVPawn.{h,cpp}`（EndPlay + ResetMetrics + CBFQPConfig public + 调参 Setter）
- `Source/uav_simulator/Telemetry/TelemetryRecorder.{h,cpp}`（+`ResetForNewScenario`）
- `Source/uav_simulator/Environment/WindField.{h,cpp}`（+`ResetDynamicState`）
- `Source/uav_simulator/Planning/DynamicObstacleActor.{h,cpp}`（新增，§8）
- `Source/uav_simulator/Planning/NMPCAvoidance.{h,cpp}` / `MultiAgent/MultiAgentTypes.h`（调参 Setter）

**Python / 前端**
- `Tools/vis/server.py`（+`do_POST` 反代 + status 探活）
- `Tools/vis/web/app.js` / `index.html` / `style.css`（控制面板）
- `Tools/vis/presets/`（预设目录）
- `Tools/vis/vis.{bat,sh}`（如需端口发现参数，双端一致）

**文档**
- `README.md`（最小化新增"Web 控制面板"功能特性）
- `CONTEXT.md` / `docs/adr/`（新增 ADR：进程内热重载、Web 控制通道）

