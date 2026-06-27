# Web 控制面板（触发重跑 / 场景配置 / 实时调参）

- **状态**: ready-for-agent
- **类型**: PRD
- **创建**: 2026-06-27
- **来源**: 用户需求 + `docs/design/web-control-panel.md` 设计文档

关联设计文档：`docs/design/web-control-panel.md`（含完整架构图、reset 接口清单、阶段划分）。
关联 ADR：ADR-0001（场景资产化）、ADR-0002（风场场景级单例）。

---

## Problem Statement

当前无人机仿真项目只有一条**单向只读**的可视化链路：UE 写 `Logs/telemetry.ndjson` → Python(`:8765`) tail 解析 → SSE 推给前端。用户在浏览器里只能回放、看指标、看判决，**无法对仿真做任何干预**。

具体痛点：

1. 想换个配置（多加几架无人机、多放几个障碍、改某架的航线）跑一次，必须退出 UE → 改 Scenario 资产或命令行 → 重跑 `sim.bat`，整个流程数十秒起步，且要把人从浏览器拉回到编辑器/脚本，割裂。
2. 想反复试一组参数，没有一键重跑入口，每次都要重启进程。
3. 仿真跑起来后想微调 PID / NMPC / 风场看实时反应，完全做不到——只能停、改、重跑。
4. 动态障碍物（注释写着"二期启用"）一直没真正动起来，无法验证避障对运动目标的响应。
5. 多架无人机当前共享同一套航线（只装配到 Leader），无法配置"每架独立航线"的多机联动场景。

用户希望：在浏览器里完成"配置场景（机队/障碍/每机航线/风场/验收/仿真参数）→ 一键重跑 → 实时观察 → 运行中调参"的完整闭环，且重跑是秒级的（进程内热重载，不重启 UE）。

## Solution

在现有单向遥测通道之外，新增一条**反向控制通道**，并把场景配置从"资产/命令行"扩展为"Web 端实时可编辑"。整体由四部分组成：

1. **进程内热重载内核**：在 GameMode 上新增 `UScenarioRuntimeController`，编排"销毁旧机队 → 重置持久组件（验收器/遥测器/风场/任务监控）→ 用新配置重新装配"。重跑秒级完成，不重启 UE。为此需补齐一批当前缺失的 reset 接口（探索已逐一定位）。
2. **UE HTTP 控制端**（`:8770`，headless 必启）：把 Web 命令切到 GameThread 执行，复用 `SoftUEBridge` 插件已有的 HttpRouter 成熟范式，但显式绕过它的 `-unattended` 跳过逻辑。
3. **Python 反向代理**：`Tools/vis/server.py` 新增 `do_POST`，把控制命令从 `:8765` 转发到 `:8770`，保持前端单一入口（避免跨端口 CORS），并做基本校验防 UE 崩溃。
4. **前端控制面板**：延续零构建原生 ES Module 路线，新增配置 Tab（机队/障碍/航线/风场·验收/仿真）、实时调参 Tab、预设库，以及 3D 坐标拾取。

配置**不落盘成 `.uasset`**：运行时 `UScenarioFactory` 把 Web 传来的 DTO 构造为内存 `UScenario`，喂给现有 `UScenarioLoader`，避免资产序列化/烹饪/版本库污染，复用 ADR-0001 已确立的装配链路。

关键 schema 扩展：每架无人机内联独立航点（解决"每机航线"缺口），动态障碍新增运动模型枚举（静态/恒速直线/巡逻循环/巡逻往返）。

## User Stories

1. 作为仿真工程师，我想在浏览器里一键重跑仿真，这样我能在数秒内迭代验证参数，而不是等数十秒重启 UE。
2. 作为仿真工程师，我想在重跑时不丢失 UE 进程，这样可视化后端、日志句柄、场景上下文都保持连续，避免反复冷启动。
3. 作为仿真工程师，我想在 Web 端配置无人机数量（多机联动），这样能验证编队/协同行为而无需改资产。
4. 作为仿真工程师，我想为每架无人机单独配置航线（航点序列），这样能构造"每机不同任务"的多机联动场景。
5. 作为仿真工程师，我想为每架无人机选择机型（型号）、设置初始位置和朝向，这样能模拟异构机队。
6. 作为仿真工程师，我想在 Web 端配置静态障碍物的数量、类型（球/盒/柱）、位置、尺寸和安全边距，这样能快速布置不同障碍布局。
7. 作为仿真工程师，我想配置动态障碍物并让它们沿巡逻路径运动，这样能验证避障算法对运动目标的响应。
8. 作为仿真工程师，我想为动态障碍选择运动模式（恒速直线 / 循环巡逻 / 往返巡逻）和速度，这样能覆盖不同的动态威胁场景。
9. 作为仿真工程师，我想在 Web 端配置风场（类型/稳态风/阵风/湍流参数），这样能验证不同气象条件下的飞行稳定性。
10. 作为仿真工程师，我想配置验收阈值（最小净空/最大横向偏差/超时/航点半径/能耗预算），这样能针对不同场景调整 PASS/FAIL 判据。
11. 作为仿真工程师，我想配置仿真控制参数（时标 slomo、仿真时长、控制模式、MPC 类型、随机种子），这样能控制仿真规模和复现性。
12. 作为仿真工程师，我想在仿真运行中实时调节 PID 增益（姿态/位置/速度环），这样能在线观察控制响应、快速收敛调参而无需重跑。
13. 作为仿真工程师，我想在运行中实时调节 NMPC 权重与避障距离参数，这样能在线权衡轨迹跟踪精度与避障激进程度。
14. 作为仿真工程师，我想在运行中实时调节 CBF-QP 安全滤波参数（安全距离/Alpha），这样能在线调整安全约束严格度。
15. 作为仿真工程师，我想在运行中实时改变风场强度/类型，这样能模拟突发阵风对在飞无人机的影响。
16. 作为仿真工程师，我想在运行中实时改变时标 slomo，这样能加速长仿真或放慢观察瞬时行为。
17. 作为仿真工程师，我想选择调参作用范围（全队 / Leader / 指定单机），这样既能统一调也能逐机精调。
18. 作为仿真工程师，我想把当前一套配置存为命名预设，这样下次能一键加载而不必重新填写。
19. 作为仿真工程师，我想导出/导入配置（JSON），这样能在不同机器或同事间分享场景配置。
20. 作为仿真工程师，我想在 3D 视图里点击拾取坐标自动填入表单，这样配置障碍/航点位置时不必手动算坐标。
21. 作为仿真工程师，我想在重跑完成后前端自动重置回放游标并刷新场景静态数据（机队/障碍/航点），这样不会看到上一场景的残留轨迹。
22. 作为仿真工程师，我想看到每次实时调参的记录（旧值/新值），这样能追溯参数变更与行为响应的对应关系。
23. 作为仿真工程师，我想在不发送任何控制命令时，现有只读可视化与 `sim.*` 退出码协议（0/1/2）行为完全不变，这样不破坏既有 CI 和工作流。
24. 作为仿真工程师，我想控制端在 headless（`-game -NullRHI`）模式下正常工作，这样命令面板在自动化仿真场景下可用。
25. 作为仿真工程师，我想热重载是互斥的（重载进行中拒绝新命令），这样不会出现半装配状态下的指标采集错误。
26. 作为仿真工程师，我想调参前对参数做值域校验，这样不会把 NMPC/CBF 调到发散导致 NaN 毁掉仿真。
27. 作为仿真工程师，我想能保存两套配置连续重跑并对比结果，这样能做参数 A/B 对比实验。（二期）
28. 作为仿真工程师，我想控制命令的端口能自动避让冲突（占用则 +1），这样多实例并行时不会端口打架。

## Implementation Decisions

### 架构与通道

- 新增**反向控制通道**：浏览器 → Python(`:8765`) → UE HTTP 控制端(`:8770`) → GameThread → `UScenarioRuntimeController`。前端始终只与 `:8765` 通信（单一入口，避免跨端口 CORS）。
- UE HTTP 控制端独立于 `SoftUEBridge` 插件（不复用其进程，因后者在 `-unattended` 跳过启动），但复用其 HttpRouter 用法范式（CORS、`AsyncTask(GameThread)` 切换、`FUnattendedScriptGuard`）。控制端**显式在 headless 启动**。
- 端口规划：`:8765`（可视化/前端入口，已有）、`:8080`（SoftUEBridge 编辑器，避开）、`:8770`（新增控制端，占用自动 +1..+9，端口写入 `Saved/.uav-ctrl/port.json` 供 Python 发现）。
- 配置**不落盘 `.uasset`**：`UScenarioFactory` 把 Web DTO 构造为内存 `UScenario`，复用现有 `UScenarioLoader` 装配链路（ADR-0001）。预设以 JSON 落在 Python 侧 `Tools/vis/presets/`。

### Schema 扩展（Scenario 数据模型）

- **每机独立航线**：在 `FScenarioAgentEntry` 内联 `TArray<FMissionWaypoint> Waypoints` + `EMissionMode MissionMode`。`ScenarioLoader::AssembleFleetAndMission` 改为**逐机装配**：每个 Agent 优先用自己的内联航点，为空则回退全局 `MissionProfile`。修复当前"只装配到 `Fleet[0]`"的缺口。
- **动态障碍运动模型**：新增枚举 `EObstacleMovementType { Static, LinearVelocity, PatrolLoop, PatrolPingPong }`，挂到 `FScenarioObstacleEntry`，并新增 `TArray<FVector> PatrolPoints` 与 `float PatrolSpeed`。沿用领域词汇 Dynamic Obstacle（见 CONTEXT.md）。
- DTO 与扩展后的 `UScenario` 一一对应，用 `FJsonObjectConverter`（JsonUtilities 模块）自动序列化，DTO 全程用 UE 厘米左手系，仅前端展示层做米/右手变换。

### 进程内热重载（核心）

- 新增 `UScenarioRuntimeController`（挂在 GameMode，与 `ScenarioEvaluatorComponent` 同级），编排 `ReloadScenario(UScenario*)`：销毁旧机队 → 重置持久组件 → 重新装配。
- 必须新增的 reset 接口（当前均缺失，探索已定位）：
  - `AUAVPawn::EndPlay` override（销毁时反注册 Agent、清残骸 Actor、释放 WindField 弱引用；当前全仓仅 `ControlParameterTuner` 实现了 EndPlay）。
  - `AUAVPawn::ResetMetrics`（累计质量指标归零，防跨场景污染验收/遥测）。
  - `AMultiAgentGameMode::ResetForScenarioReload`（总调度入口）。
  - `UScenarioEvaluatorComponent::Reset`（清 `Accumulated`/`ElapsedTime`/`bFinalFlushed`，解绑旧 Mission 委托）。
  - `UTelemetryRecorder::ResetForNewScenario`（关旧文件句柄 → truncate 重开 → 清 `bStaticWritten`/累加器/`LastEventSeq`）。
  - `UWindField::ResetDynamicState`（清阵风/Dryden 相位状态，`SetWindConfig` 不会重置这些）。
  - 热重载前强制清除 `BTTask_ExitSimulation` 注册的延迟 `RequestExit` Timer（否则误杀进程）。
  - `TaskAllocator`/`TaskMonitor` 状态清理或重建（随 GameMode 持久，不随 Pawn 销毁）。
- 重置调用顺序：清退出 Timer → Stop/Destroy 动态障碍驱动器 → 销毁机队（触发 EndPlay）→ 清注册表/缓存/`NextAgentID=0` → 重置各持久组件 → `WindField->SetWindConfig` → `LoadAndAssembleScenario`（新建 Loader/Evaluator，旧的 DestroyComponent）。
- AgentID 重置策略：**重置为 0**（语义对齐进程重启，与 telemetry 的 `agent` 字段连续一致），前提是重载前确保所有旧 Agent 已销毁。
- 线程安全：控制命令经 `AsyncTask(GameThread)` 执行；重载期间 `bReloading` 互斥标志暂停 evaluator/recorder 的 Tick；完成后写 `reload` 事件到 ndjson。

### 实时调参（无需重跑）

- 利用既有 `BlueprintReadWrite` 字段直接赋值（项目已有 `ControlParameterTuner` 的 `pid.set` 控制台命令先例）。统一经 HTTP `/control/params` 下发。
- 真实缺口需补：`CBFQPConfig` 当前 `private` 无 Setter（需改 public 或加 Setter）；slomo 无 C++ 运行时 Setter（新增 `SetSlomo` 走 `WorldSettings->SetTimeDilation`，与 `TelemetryRecorder` 读时标的口径一致）。
- 调参作用范围 `target`：`fleet` / `leader` / 指定 agentId。apply 前做值域校验。
- 风场实时调参复用已有 runtime-safe 的 `UWindField::SetWindConfig`（无需底层改造）。

### 动态障碍驱动

- 新增 `ADynamicObstacleActor`，`PatrolLoop`/`PatrolPingPong` 沿 `PatrolPoints` 按 `PatrolSpeed` 推进，`LinearVelocity` 匀速+出界处理。
- 复用既有 `UObstacleManager::UpdateDynamicObstacles`（每帧从 LinkedActor 同步位置反算速度），NMPC/CBF 自然视为动态约束。
- 热重载时由 `ResetForScenarioReload` 先 Stop 再 Destroy 全部动态障碍 Actor。

### Python 反向代理

- `server.py` 新增 `do_POST`，路由透传到 `:8770`，加 DTO 基本校验（机队数/障碍数/坐标范围/速度正数），非法返回 400。
- 启动时读 `Saved/.uav-ctrl/port.json` 发现 UE 控制端口；读不到则标记"控制不可用"，前端面板置灰。
- SSE 新增 `reload` 事件：UE 重载完成写 ndjson → Python feed → 推前端 → 前端重置游标、刷新静态数据。

### 前端控制面板

- 延续零构建原生 ES Module + Three.js，新增左侧/底部控制区，不破坏现有 3D+图表布局。
- 配置 Tab：机队（动态增删，每行机型/位姿/朝向/Leader + 展开内联航线）、障碍（静态/动态分组，动态含运动模型与巡逻点）、风·验收、仿真（slomo/时长/控制模式/MPC 类型/种子）。
- 实时调参 Tab：PID/NMPC/CBF/风场/slomo 滑块 + target 选择器 + 调参历史回滚。
- 3D 拾取模式：点击地面/障碍填表单坐标。
- 预设库 / A-B 对比：架构在 DTO 层天然支持，对比视图二期落地。

### 一致性约束

- `Tools/vis/vis.bat` 与 `vis.sh` 行为一致（CLAUDE.md 强制 sh/bat 等价）。
- 控制端独立模块，默认禁用时不影响现有只读可视化与 `sim.*` 退出码协议（0/1/2）。

## Testing Decisions

测试哲学：**只测外部可观察行为，不测实现细节**。优先复用既有 seam，不引入新基础设施。

### C++ 层 —— 纯 automation 单测（主 seam）

- 全部走 `IMPLEMENT_SIMPLE_AUTOMATION_TEST`，合成 World + 持久子组件 mock，验证装配/重置后的外部可观察状态。完全对齐项目现有 `Tests/Scenario/*` 范式（如 `ScenarioLoaderFleetTest`、`ScenarioE2ETest`、`ScenarioEvaluatorTest`）。
- **什么是好测试**：构造内存 `UScenario` → 调装配/reset → 断言 Fleet 数量、机型、位姿、航点、注册表清空、文件 truncate、指标归零、AgentID 复位等外部状态。**不**断言私有成员、内部调用顺序。
- 覆盖模块：
  - schema 装配：每机独立航线逐机下发、动态障碍运动类型装配（参照 `ScenarioLoaderFleetTest` / `ScenarioLoaderObstaclesTest`）。
  - 热重载内核：`ReloadScenario` 前后状态断言（注册表/缓存/`NextAgentID`/`bFinalFlushed`/ndjson truncate/阵风相位），参照 `ScenarioE2ETest` 的合成 World helper。
  - `UScenarioFactory`：DTO ↔ Scenario 往返转换的正确性。
  - 动态障碍驱动：`ADynamicObstacleActor` 沿巡逻点推进后的外部位置/速度。
  - 实时调参 Setter：apply 后外部可读字段（PID 增益/NMPC Config/CBF Config）已变更。
- 跑法：`Script/test.bat` / `test.sh`，纳入既有 `UAVSimulator.*` 测试命名空间。

### Python 反向代理 —— 少量 pytest 单测

- 新增 pytest 测试 DTO 校验、端口发现、`do_POST` 转发与错误回执。mock 掉真实 UE 连接，断言转发 payload 与状态码。
- 这层新增测试基础设施，但 Python 侧尚无既有测试，pytest 是主流轻量选择。

### 前端 —— 手动验证清单

- 沿用前端零测试现状，提供结构化手动验证清单（配置→rerun→看结果→实时调参→预设 全链路），不引入 Playwright 等依赖（保持零构建路线）。

### headless 集成验证（非自动化 seam，PRD 外的人工验收）

- 用 curl 经 `:8770`（及经 `:8765` 转发）发 reload/slomo/wind/params，观察 `Logs/telemetry.ndjson` 与 `Logs/scenario_result.json` 的刷新行为。这是 PRD 验收环节，不作为持续测试 seam。

## Out of Scope

- **预设库持久化到 UE 资产**：预设只落 Python 侧 JSON，不生成/烘焙 `.uasset`（避免版本库与烹饪复杂度）。
- **A/B 对比前端视图**：DTO 层与重跑架构天然支持，但对比可视化 UI 标为二期。
- **认证/多用户/权限**：本机单用户工具，控制端仅监听 `127.0.0.1`，不做鉴权。
- **历史调参回放到具体数值的自动化断言**：调参历史仅做前端展示与一键回滚，不做回归测试基线。
- **WebSocket 双向通道**：用 HTTP POST + SSE（`reload`/`param`/`done` 事件）足够，不引入 WebSocket。
- **替换现有遥测协议为增量协议**：保持 SSE 整快照推送，重载后由前端刷新，不改 ndjson 行类型契约（仅新增 `reload`/`param` 事件类型）。
- **非 headless（编辑器内）的控制面板**：控制端在编辑器也能用，但 PRD 聚焦 headless 仿真场景。
- **多机间实时通信协议升级**：实时调参只做参数下发，不改多机协同通信组件。

## Further Notes

- 设计文档 `docs/design/web-control-panel.md` 是本 PRD 的详细附件，含完整架构图、reset 接口清单（逐文件:行号定位）、阶段划分（P1~P7）、风险对策。
- 关联 ADR-0001（场景资产化，内存 Scenario 复用其装配链路）、ADR-0002（风场场景级单例，热重载复用其 WindField 单例并补 reset）。
- 新增内容应新增 ADR：进程内热重载机制、Web 控制通道与端口规划。
- 实施建议按 P1（schema+每机航线）→ P2（热重载内核）→ P3（UE HTTP 控制端）→ P4（Python 反代）→ P5（前端面板）→ P6（动态障碍驱动器）→ P7（预设/A-B）推进，每阶段独立可验证、可提交，遵循 CLAUDE.md 的"改 C++ → 编译 → 仿真 → 看日志"工作流。
- README.md 需最小化新增"Web 控制面板"功能特性描述（CLAUDE.md 要求文档与代码一致）。
