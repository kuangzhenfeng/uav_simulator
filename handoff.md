# UAV Simulator 交接

更新时间：2026-10-10。

## 用户目标与当前状态

开发农业无人机全局作业效率优化：药量影响质量和耗电，提前就近喷洒减重应纳入航线、装载、充电、共享补给与任务分配的联合决策。UE 5.8 已安装。用户最后要求“尽快收敛”，随后要求交接文档放在根目录 `handoff.md`。

**尚未完成农业全场景验收。** 当前源码编译通过、单元测试 287/287 通过；最新仿真只是定位首航次过早返供的短程诊断，不能报告全局效率优化已完成。下一会话优先解决下述唯一根因链，避免继续扩展优化范围。

所有改动未提交，禁止执行 `git commit`。不要回滚既有工作。最新验证的 UE、wrapper、monitor、watchdog 及其子进程均已退出，源码已经解冻，可以继续编辑。没有自行启动仍存活的 UE 编辑器。

## 必读已有资料

- `AGENTS.md`：项目规则。大量日志分析也需上下文隔离。中文注释、英文日志，禁止 Verbose；功能修改最小更新 README。
- `README.md`：当前功能描述。
- `Docs/AgricultureEfficiency.md`：已实现的效率模型、限制和完整验收要求。不要在本交接重复实现细节。
- `Source/uav_simulator/MultiAgent/AgriculturePlanning.cpp`：能耗积分、航次预测、服务与剩余作业成本。
- `Source/uav_simulator/MultiAgent/AgricultureCoordinator.cpp`：执行状态机、机场时间线及预约。

不要调低安全阈值、放宽截止时间、修改场景或物理参数来换取通过；当前算法仍是滚动启发式，不能声称所有未来任务的数学全局最优。

## 最新证据与下一步

最新完整日志目录：

`.scratch/agriculture-efficiency/validation/Agriculture/run-return-commit-diagnostic-20261010_172833/final_root_logs/`

同级 `RUN_STATUS` 和停止快照记录运行过程。最新 build 4/4 actions 通过，test 287/287 通过。仿真正常 POST exit 于农业时间 433.38 秒；0/6 plots，最大横向偏差 277.08 cm，无碰撞；verdict `final=false`，不是 3600 秒终局。

### 已确认的问题

初始服务方案：A0/A1 在机场 0/1 装载 11.8 L、电量目标 .949，预测可喷 10.8 L、返场 134.54 秒；A2/A3 在机场 2/3 装载 11.8 L、电量目标 .986，预测可喷 10.8 L、返场 139.89 秒。

首次 Transit 时预约仍是各自原站，并非起飞前已改站。进入 Spraying 后约农业时间 111 秒，规划器判原返場承诺不可行，改为 A3 3→1、A2 2→0、A1 1→3、A0 0→2。实际先飞到这些机场，服务计划随后又转回原站。A2/A3 首轮只喷约 .1468 L/.00617 L，A0/A1 各 5.4 L，然后触发返供。

`PlanSupplyAssignments()` 原先每 .5 秒清空预约，自由按后续完成时间选站。已加修复：先检验 `ServiceAirportTargetID` 对应的本航次预算站，只有承诺不可行才搜索其他站；失败改站更新承诺并输出 `SortieReturnReplanned`。**这个修复尚未消除问题**，因为原站仍被拒绝。

最新改站事件的拒绝预算：

| Agent | 改站 | ForecastFeasible | Completed | Applied L | Travel s | Wait s | Required | Battery | SlotFeasible | ServiceLiquid |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 3 | 3→1 | 1 | 0 | 10.794 | 139.89 | 0 | .72288 | .77892 | 0 | -1 |
| 2 | 2→0 | 1 | 0 | 10.653 | 139.89 | 0 | .72080 | .77722 | 0 | -1 |
| 1 | 1→3 | 1 | 1 | 5.831 | 134.54 | 270.33 | 1.13107 | .70013 | 1 | -1 |
| 0 | 0→2 | 1 | 1 | 5.692 | 134.54 | 269.59 | 1.12808 | .69877 | 1 | -1 |

A2/A3 原站虚拟库存 Water=1983.40、Concentrate=99.77、Waste=13。其预算电量足够、Wait=0，但下一航次 `PlanService()` 返回 -1，导致 SlotFeasible=0。A0/A1 随后遭遇约 270 秒等待；“由 A2/A3 跨站预约引发连锁冲突”仍需精确时隙证据，不能直接当作已证实根因。

### 最优先待复现的线索（未证实、未修改）

`ForecastSortie()` 使用 `Length-1e-4` cm 判断条带完成；`AdvanceForecastProgress()` 使用 `Length-1e-3` cm，且把进度累加到 float `StripCoveredCm`。可能出现预测 `bCompleted=false`，但推进后的复制分区 `NextPoint` 已超过条带数组；后续 `PlanService()` 因没有剩余条带返回 -1。这能解释 A2/A3 的预算电量足够却被拒绝。

下一步先在既有农业测试中构造部分喷洒后剩余量接近整段完成的案例，比较 Forecast 的 bCompleted、AppliedLitres 与 Advance 后的 NextPoint、RemainingWorkSeconds。必要时在改站诊断补充 Remaining.NextPoint/StripPoints.Num()/Demand，**取得确定复现后才修改**。若成立，应统一完成判据/预测状态，避免给已经完成的预测分区请求补给；不要通过放宽实际覆盖质量阈值掩盖残段。

检查涉及函数：`ForecastSortie`、`AdvanceForecastProgress`、`PlanService`、`PlanSupplyAssignments` 的 worker 候选循环，以及 `BookSortie`、`EstimateSectionCost` 中类似的预测完成与服务调用。现有最新日志已包含改站时预算；不要先进行其他未经证实的效率改动。

## 已有改动与历史验证索引

具体源码差异用 `git diff` 查看。涉及 Core/UAVPawn、MultiAgent 的 Agriculture/AgentManager/TaskAllocator/CBFQP、Planning 的 CoveragePlanner/TrajectoryOptimizer 及相应 Tests。新增 `AgriculturePlanning.cpp`、`Docs/AgricultureEfficiency.md`。脚本和资产未修改。

已实现载荷减重积分、装载与目标电量联合搜索、剩余入口择优、机队最晚完成时间匹配、时隙与库存预约、源站地面充电等待/转移、返场预算、安全执行加速度契约等；实现与模型边界详见既有文档和差异。

本次会话主要证据目录（均在 `.scratch/agriculture-efficiency/validation/Agriculture/`）：

- `run-20261010_145529`：曾完整运行至 3600.81，FAIL，9/24 sections、0/6 plots；不是当前源码版本。
- `run-contract-fix-20261010_153753`：最终执行加速度契约修复后横向偏差降至约 272 cm，仍有供给失败。
- `run-supply-wait-fix-20261010_160115`：已修等待排除本机，但地面转移等待冻结和重复计算严重。
- `run-queue-snapshot-20261010_164540`：队列快照复用后晚期规划由约 2–3.6 秒降至 50–102 ms；A2 农时 3080 因 No supply airport supports remaining work 失败，8/24，未完整终局。
- `run-ground-replan-20261010_171519`：地面等待滚动复核、健康停靠无方案退回任务、实际泊位分配校验、按服务资源区分安全降落；287 tests 通过，诊断停止，首轮跨站早返仍在。
- `run-return-commit-fix-20261010_172236`：返場承诺优先仍被判不可行；287 tests 通过，诊断停止。
- 最新 `run-return-commit-diagnostic-20261010_172833`：增加拒绝预算，证据如上。

## 验证交接规则

Windows 使用 `Script/build.bat`、`Script/test.bat`、`Script/sim.bat`。每次 C++ 修改后必须 build→test→sim→日志检查；源码在验证期间冻结，失败才释放修复，防止混合 DLL。

- 正确地图：`/Game/Environment/Levels/UavCooperationMap.UavCooperationMap`。
- 农业资产：`/Game/Scenarios/Cooperation/Agriculture.Agriculture`。
- 已采用原 sim 参数 1800 real watchdog / 8×。农业时间看 telemetry 的 `agriculture.t` 与 result `metrics.elapsedSec`，不要拿 outer world t 替代。
- 完整判据：农业 3600 秒并有 NDJSON `type=verdict, final=true`，结合真实全部覆盖和返场终态。中途 result FAIL/WaypointsNotReached 是 interim，不能误判终局。
- HTTP 8770：POST `/control/status`、`/control/exit`。诊断停机要正常 POST exit 并归档 Logs；任何本次进程清理用 PID+StartTime，禁止 image-wide taskkill。
- 原 sim.bat watchdog sidecar 会 image-wide kill；验证代理曾精确停止本次 sidecar/ping/conhost，用独立 own-PID watchdog 替代，没有改原脚本。后续代理须保持这项安全约束。
- 正式验收若 Agriculture PASS，再串行 SharedSupply、PriorityAgriculture、AgricultureRecovery（恢复场景参数此前要求 3）；从现有验证脚本/记录获取完整资产路径，不猜路径。
- 全日志可达数百 MB。不要把大量日志灌入主上下文。

新会话需重新派代理，不能假定旧代理仍可接收任务。

## Suggested skills

- `diagnosing-bugs`：下一会话读取其 SKILL.md，按证据→复现→最小根治循环定位 Forecast/Advance/PlanService 状态不一致。本会话已使用过。
- `handoff`：本次用户显式调用，已读取 `C:\Users\PC\.agents\skills\handoff\SKILL.md`。用户明确要求当前目录，覆盖技能默认临时目录的保存位置要求。

无需为此次收敛扩展 UI、架构重构或全新物理模型，也无需启动 UE 编辑器。若后续确需编辑 UE 内容，必须使用官方 Unreal MCP。
