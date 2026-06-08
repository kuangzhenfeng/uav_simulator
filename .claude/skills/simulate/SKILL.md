---
name: simulate
description: 用于 UAV Simulator 的构建/测试/仿真闭环、Logs/uav.log 指标分析，以及 NMPC、CBF-QP、安全、速度、姿态、Yaw、偏差等行为回归或修复验证。
---

# Simulate Skill

## 核心原则

**批量诊断，单元验证。** 诊断阶段必须同时识别多个候选根因、共同上游和顺序依赖；实验阶段每轮只验证一个可归因的干预单元。

不要把“单一根因”理解成“只能调查一个问题”。正确做法是：先建立因果图，再选择最高信息增益的一个干预单元做实验。

## 快速规则

1. 先建立可比较基线，再修改代码。
2. 先画因果链/依赖 DAG，再选择实验；禁止凭直觉直接调参。
3. 一轮只验证一个**干预单元**，不混改 NMPC、CBF、姿态/Yaw 后择优。
4. 安全正确性优先于性能优化；安全失败直接否决候选，不能用速度或跟踪改善抵消。
5. 使用同一场景、种子、时长和 `slomo` 比较基线与候选。
6. 不自动降低指标；预算耗尽时报告最佳已验证版本、证据和下一最高价值实验。
7. 不把 `Mission completed` 单独视为成功；出现 `Overtime timeout` 或 `force completing` 即失败。
8. 不撤销用户已有修改；失败实验只精确撤销本轮由自己产生的改动。

## 干预单元

干预单元是一组服务于同一个可证伪因果假设的最小修改。它可以包含必要配套修改，但必须能归因。

**允许作为同一干预单元：**

- 修复 CBF 约束方向，同时更新对应结构化诊断日志。
- 修改代价形式，同时按新量纲重新标定直接相关权重。
- 调整 yaw 参考生成，同时补充证明该参考是否跳变的日志。

**禁止作为同一干预单元：**

- 同时改 CBF slack、NMPC 初值、Yaw PID，然后用一次仿真择优。
- 为了速度同时降低安全阈值和提高最大加速度。
- 候选未验证前启动下一轮候选。

每个干预单元必须记录：假设、证据、预计改善指标、不应退化指标、回滚条件、验证命令。

## 硬门槛

以下任一情况判定失败，不得降级：

- `[Crash]`、`Obstacle penetration` 或机间碰撞。
- NaN、Inf、断言、崩溃或未处理的求解失败。
- `Overtime timeout`、`force completing`。
- Roll 或 Pitch 超过 60 度，或持续姿态失稳。
- 单次 Yaw 跳变超过 45 度，或连续 5 秒 Yaw 振荡超过 30 度。
- 修改安全层后没有报告约束残差、slack 或求解状态。
- CBF 输出后正常路径继续修改线加速度。
- 横向偏差超过 15 米，或连续 20 秒偏差超过 10 米。

## 默认性能目标

按每个 Agent 的 `MaxVelocity` 归一化：

- 稳定巡航速度不低于 50%。
- 避障后 8 秒内恢复到 40%。
- 低于 30% 的单次持续时间不超过 15 秒。
- 默认横向偏差不超过 5 米。
- 不连续 20 秒负进度、原地徘徊或绕圈。
- Yaw 跳变超过 30° 的帧占比不超过 15%。
- 确定性回归场景任务成功率 100%。
- 报告 NMPC 求解时间 P50、P95、P99，不只报告均值。

## 参数与预算

| 参数 | 含义 | 默认 |
|---|---|---|
| `focus` | 分析域：`nmpc`/`safety`/`flight`/`all` | `all` |
| `duration` | 仿真实时时长(s) | 80 |
| `slomo` | UE 时间倍率 | 8 |
| `max_iterations` | 最大候选实验轮数 | 6 |
| `scenario` | 仿真场景 | 当前默认 |

用户明确指定目标、场景、时间或预算时，以用户要求为准。

### 预算受限模式

当用户说“尽快”“今晚”“半天内”“不要跑很多小时”，或单次 build+test+sim 超过 30 分钟，必须进入预算受限模式：

| 单轮完整闭环耗时 | 候选实验上限 |
|---:|---:|
| ≤ 15 分钟 | 6 |
| 15-30 分钟 | 3 |
| 30-60 分钟 | 2 |
| > 60 分钟 | 1 |

预算受限时，先做并行只读诊断和信息增益排序，只运行最高价值候选。不得机械跑满默认 `max_iterations`。

## 因果图与优先级

基线后必须输出简短因果链或依赖 DAG：

```text
症状 → 中间机制 → 候选上游根因 → 依赖关系 → 可验证干预单元
```

优先级从高到低：

| 层级 | 类型 | 例子 |
|---|---|---|
| L0 | 指标可信度 | 构建失败、日志缺失、随机源未固定、成功判定不可信 |
| L1 | 安全正确性 | CBF 符号/方向、碰撞、NaN、约束残差、slack 状态 |
| L2 | 控制所有权 | CBF 后继续修改加速度、限幅链破坏安全输出 |
| L3 | 规划稳定性 | NMPC 初值、warm reset、代价结构、线搜索、homotopy |
| L4 | 性能调参 | 速度恢复、Yaw 阻尼、姿态超调、权重尺度 |

如果存在顺序依赖，必须按依赖顺序实验。例如：CBF 约束符号不可信 → 先修 CBF；再评估 NMPC 初值；最后调姿态/Yaw 性能。

## 信息增益排序

多个候选假设同时存在时，先评分再选实验：

```text
优先级 = 证据强度 × 影响范围 × 可验证性 × 安全层级权重 ÷ 实验成本
```

优先选择：

- 能解释多个症状的共同上游根因。
- 位于依赖链前面的正确性问题。
- 一轮实验能明确证伪的假设。
- 不会牺牲硬门槛的候选。

不要优先选择：

- “参数不合适”这种不可验证假设。
- 只能改善单个表面指标、但解释不了共同时间线的调参。
- 需要多模块混改才可能见效的候选。

## 并行子 Agent 约束

主 Agent 负责顺序控制、文件修改、构建、测试、仿真和最终取舍。子 Agent 仅执行只读分析。

**允许并行：**代码扫描、日志提取、根因候选生成、因果图复核、指标对比、硬门槛复核、README 是否需要更新的判断。

**禁止并行：**子 Agent 修改文件或运行仿真；并行调参或并行修改后择优；基线未建立或候选未接受前启动下一轮。

**子 Agent 统一输出：**文件路径:行号、关键证据（代码片段或日志原文）、指标域和数值、硬失败判定、候选假设、依赖关系、建议干预单元。

## 工作流

### 1. 预检 → 并行代码扫描

```bash
git status --short
git diff <relevant files>
```

按 `focus` 派发只读子 Agent。`focus=all` 时派发全部三个；否则只派发对应项。每个 prompt 必须包含具体文件路径、分析维度、“只读，不修改任何文件”、返回格式。

| 子 Agent | 文件 | 分析重点 |
|---|---|---|
| NMPC | `Planning/NMPCAvoidance.*`、`TrajectoryTracker.*` | PGD 初值、代价结构、权重比例、线搜索、收敛判据、warm reset |
| Safety | `MultiAgent/CBFQPFilter.*`、`MultiAgentTypes.h`、`Core/UAVPawn.*` | CBF-QP 约束构造、二阶方向/符号、后置修改器链、slack、执行器可行域 |
| Flight | `Control/PositionController.*`、`AttitudeController.*`、`Dynamics/UAVDynamics.*` | 加速度到倾角映射、姿态环阻尼/超调、Yaw 基准、SO(3) 耦合、推力饱和 |

合并为预检摘要。若缺少判定数据，先补最小结构化诊断日志，不凭猜测调参。

已知风险：CBF-QP 二阶约束方向存在问题。若启用且指标依赖安全层，先修复或禁用，再进行性能优化。

### 2. 建立基线

```bash
# Windows
Script\build.bat
Script\test.bat
Script\sim.bat <duration> <slomo>

# macOS/Linux
Script/build.sh
Script/test.sh
Script/sim.sh <duration> <slomo>
```

构建失败时先判断是否来自当前工作区已有修改。没有可运行基线则不调参。

保存本轮：commit/工作区状态、场景/种子/时长/slomo、`Logs/uav.log` 和必要的 `Logs/uav_full.log`、指标摘要。默认场景有随机因素且没有固定种子时，不做精确 A/B 结论；先增加确定性场景或固定随机源。

### 3. 提取指标 → 并行日志分析

主 Agent 先扫硬失败：

```bash
rg -n "Crash|penetration|collision|NaN|Inf|Overtime timeout|force completing|Attitude anomaly|NumericalFailure" Logs/uav.log Logs/uav_full.log
```

然后并行派发只读日志分析子 Agent。每个 prompt 指定 TAG、计算逻辑、硬门槛判定标准，返回表格、关键时间线和日志证据。

| 子 Agent | 搜索 TAG | 计算指标 |
|---|---|---|
| 速度 | `SPEED_METRICS`、`LOW_SPEED_END`、`AVOIDANCE_END`、`AVOIDANCE_RECOVERY` | 巡航速度比例、低于 30% 最长持续、避障恢复时间 |
| 姿态航向 | `ATTITUDE`、`YAW_METRICS`、`YAW_ANOMALY` | Roll/Pitch 最大值、Yaw 跳变 > 30° 占比、漂移和振荡 |
| 安全NMPC | `CBF_INTEGRITY`、`CBF_SOLVE`、`NMPC_SOLVE`、`NMPC_RESULT`、`NMPC_PERCENTILE`、`DEVIATION_SEVERE` | CBF MinH/残差/slack、NMPC P50/P95/P99、代价、偏差 |
| 任务汇总 | `SIM_SUMMARY`、`TASK_SUMMARY`、`SIM_RESULT` | 型号/MaxVelocity、成功数/总数、超时、炸机、卡死、重规划 |

日志有节流时，缺少某条日志不能证明事件没有发生。优先使用结构化汇总或 CSV。

输出基线表：

| 指标 | 基线 | 目标 | 状态 |
|---|---:|---:|---|
| 碰撞/穿透 | | 0 | |
| 强制完成 | | 0 | |
| 最小净空 | | 场景要求 | |
| 最大姿态 | | ≤ 60° | |
| Yaw > 30° 占比 | | ≤ 15% | |
| 成功率 | | 100% | |
| 巡航速度比例 | | ≥ 50% | |
| 最大低速持续 | | ≤ 15s | |
| 最大横向偏差 | | ≤ 5m | |
| 严重偏离 | | < 15m 且无 20s > 10m | |
| Stuck/Replan | | 越低越好 | |
| NMPC P95 | | 控制周期内 | |

### 4. 建立因果图 → 选择干预单元

并行派发 3 个只读根因 Agent，将预检摘要、基线表和关键时间线作为上下文传入。

| 子 Agent | 重读文件 | 分析方向 |
|---|---|---|
| NMPC 根因 | `NMPCAvoidance.cpp` | 初值对称、代价权重失衡、warm reset 误判、线搜索失败 |
| Safety 根因 | `CBFQPFilter.cpp`、`UAVPawn.cpp` | 约束符号错误、后置限幅破坏、slack 增益过强、安全集不可达 |
| Flight 根因 | `PositionController.cpp`、`AttitudeController.cpp`、`UAVDynamics.cpp` | 加速度可行域不足、Yaw PID 阻尼不够、推力饱和、SO(3) 耦合 |

每个子 Agent 必须返回：根因候选、证据、因果链、顺序依赖、建议干预单元、预计影响指标、不应变化指标、回滚条件。

主 Agent 合并为候选表：

| 候选 | 层级 | 证据 | 影响指标 | 依赖 | 成本 | 信息增益 | 选择 |
|---|---|---|---|---|---:|---:|---|

选择最高信息增益且依赖最靠前的一个干预单元作为本轮实验。

### 5. 实施最小实验

- 优先修复模型、符号、量纲、状态机或控制所有权问题。
- 只有根因明确为参数尺度时才调参。
- 修改代价形式后同步重新标定相关权重，不孤立机械替换。
- 多初值优先使用确定性 `Warm/Nominal/Left/Right/Up/Down/Brake`。
- 只有设计文档中的 CEM 准入条件满足时，才加入低维 CEM。
- CBF 必须作为正常路径最后一个线加速度修改层。

诊断日志要求：日志用英文，注释用中文；高频日志使用 `UE_LOG_THROTTLE`；优先输出一行结构化字段；修复后删除临时日志，或降频为长期诊断；不使用 `Verbose`。

### 6. 验证候选 → 并行复核

每次 C++ 修改后严格执行：

1. 编译。
2. 运行相关单元测试；公共控制路径变更运行完整 `Script/test.*`。
3. 使用与基线完全相同的仿真参数运行。
4. 提取同一组指标。
5. 检查硬门槛。

候选仿真后，并行派发只读复核：

| 子 Agent | 输入 | 检查内容 |
|---|---|---|
| 硬门槛复核 | 基线摘要 + 候选日志 | 碰撞、NaN、超时、姿态、Yaw、CBF 完整性、偏离 |
| 指标对比 | 基线摘要 + 候选日志 | 速度/偏差/Yaw/NMPC/CBF/成功率对比 |
| README 检查 | `git diff --name-only` | 新增/移除功能、算法重大变更、用户可见行为变化 |

接受条件（缺一不可）：无新增硬失败；目标指标产生超过正常波动的改善；非目标关键指标没有显著退化；代码改变与干预单元假设一致。

否则只精确撤销本轮自身修改，不使用 `git reset --hard` 或整文件覆盖。

### 7. 迭代与停止

候选接受后，以其作为新基线继续下一轮。最多运行预算裁剪后的 `max_iterations`。

提前停止条件：全部硬门槛和用户目标达成；问题属于场景、资产或外部依赖；连续两轮没有可测改善；预算受限模式下已完成最高信息增益候选且剩余时间不足以完整验证下一候选。

达到预算未达标时，不降低目标。输出：最佳已验证版本、基线与最佳结果对比、已排除假设、当前阻塞根因、下一项最高价值实验。

## 根因分流

| 现象 | 优先检查 |
|---|---|
| 对称障碍前停滞 | 路径进度、初值对称、homotopy、预测净空 |
| 走廊中无意义偏航 | 多初值偏置过强、侧记忆未正确清除 |
| 低速不恢复 | 参考时间推进、速度意图、后置限幅、卡死状态机 |
| 频繁 warm reset | 跨帧代价误比较、初值质量、线搜索诊断 |
| `SatRatio > 1` 或加速度超限 | 速度投影反推加速度后破坏最大加速度约束 |
| CBF 后仍接近/碰撞 | 约束符号、后置修改器、执行器可行域、感知延迟 |
| slack 长期非零 | CBF 增益过强、执行器能力不足、安全集不可达 |
| 姿态异常 | 加速度命令可行域、倾角/推力限制、姿态环 |
| Yaw 振荡/漂移 | YawPID 阻尼、航向参考基准、SO(3) 运动学耦合 |
| “任务完成”但偏差巨大 | 强制完成或超时逻辑，按失败处理 |

## `/simulate add_log`

为硬门槛和性能目标中缺少日志的指标添加结构化判决日志。

**参数：** `focus` 指定指标域（`speed`/`attitude`/`yaw`/`deviation`/`safety`/`solver`/`task`/`all`），缺省自动检测。

1. 读取硬门槛和性能目标，搜索 `LogUAVMetrics` 使用情况，对比现有 TAG 清单识别缺口。
2. `[TAG] key=value` 格式，`LogUAVMetrics` 类别，多 Agent 携带 `Agent=%d`，高频用 `UE_LOG_THROTTLE`，禁止 Verbose。
3. 在 `UAVPawn.h` `// ---- 仿真指标累计器 ----` 区添加累计变量，封装到 `UpdateMetricsLog()`，编译。
4. `Script\sim.bat 40 8` → `rg "<NEW_TAG>" Logs/uav.log` 确认。

**日志 TAG 清单：**

| TAG | 位置 | 频率 | 覆盖指标 |
|---|---|---|---|
| `[NMPC_SOLVE]` | NMPCAvoidance.cpp | 每次求解 | NMPC 初值/代价/净空/进度/梯度残差/迭代/时间 |
| `[NMPC_RESULT]` | UAVPawn.cpp | 1s | Agent 维度 NMPC 类型/失败标志/卡死/求解诊断 |
| `[CBF_SOLVE]` | UAVPawn.cpp | CBF 触发时 | CBF MinH/残差/slack/活跃约束/状态 |
| `[SPEED_METRICS]` | UAVPawn.cpp | 2s | 速度/MaxVel/比例/低速持续 |
| `[LOW_SPEED_END]` | UAVPawn.cpp | 事件 | 低速持续时间 |
| `[ATTITUDE]` | UAVPawn.cpp | 0.5s (超限时) | Roll/Pitch 姿态角 |
| `[YAW_METRICS]` | UAVPawn.cpp | 2s | Yaw 跳变幅度/航向漂移/振荡次数 |
| `[YAW_ANOMALY]` | UAVPawn.cpp | 事件 | 单次 Yaw 跳变 > 30° |
| `[DEVIATION_SEVERE]` | UAVPawn.cpp | 事件 | 横向偏差 > 15m 或 10m 持续 20s |
| `[AVOIDANCE_END]` | UAVPawn.cpp | 事件 | 避障结束时间 |
| `[AVOIDANCE_RECOVERY]` | UAVPawn.cpp | 事件 | 避障恢复时间/是否恢复 |
| `[CBF_INTEGRITY]` | UAVPawn.cpp | 5s (被修改时) | CBF 后加速度被修改 |
| `[SIM_RESULT]` | UAVPawn.cpp | 事件 | 强制完成/炸机 |
| `[NMPC_PERCENTILE]` | UAVPawn.cpp | 5s | NMPC P50/P95/P99 |
| `[SIM_SUMMARY]` | UAVPawn.cpp | 5s | 全局汇总 |
| `[TASK_SUMMARY]` | AgentManager.cpp | 5s | 任务完成数/总数 |

## 常见错误

| 错误 | 正确做法 |
|---|---|
| 把“一轮一个假设”理解成“一次只能想一个根因” | 批量诊断多个候选，实验只验证一个干预单元 |
| 默认跑满 6 轮 | 先估算单轮成本，预算受限时裁剪候选数 |
| 先调性能参数 | 先修安全、符号、量纲、控制所有权 |
| 多模块混改后看指标 | 建立干预单元，保证可归因 |
| 只看最终成功 | 检查超时、force completing、偏差、CBF slack、硬失败 |
| 缺少日志就猜 | 先补最小结构化诊断日志 |

## 最终报告

必须包含：

1. 修改内容和对应干预单元。
2. 因果图/依赖链摘要。
3. 构建、测试和仿真是否成功。
4. 基线与最终指标对比。
5. 未解决风险。
6. 是否触发 CEM、CBF slack 或 degraded mode。
7. 预算受限时：为什么选择该候选，以及下一最高价值实验。
