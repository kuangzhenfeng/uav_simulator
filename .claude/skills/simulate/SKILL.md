---
name: simulate
description: 运行 UAV Simulator 的可复现优化闭环。用于构建、测试、运行仿真、分析 Logs/uav.log 与结构化指标，以及针对 NMPC、PGD 初始化、轨迹跟踪、CBF-QP、安全约束、速度或姿态问题进行基线对比和迭代修复。适用于“运行仿真并优化”“修复卡死/碰撞/低速/偏航”“验证 NMPC 或 CBF 改动”等任务。
---

# Simulate Skill

## 参数

### `/simulate add_log`

为硬门槛和性能目标中缺少日志的指标添加结构化判决日志。

**参数：**
- `focus`: 指定指标域（缺省时自动检测所有缺口）：
  - `speed`: 速度相关（巡航速度比例、低速持续时间、避障恢复）
  - `attitude`: 姿态相关（Roll/Pitch 超限、姿态失稳持续时间）
  - `yaw`: 航向相关（Yaw 跳变、航向漂移、航向振荡）
  - `deviation`: 横向偏差相关（偏差 > 5m、严重偏离航线）
  - `safety`: 安全约束相关（CBF 完整性、约束残差）
  - `solver`: 求解器相关（NMPC 分位数、CBF 求解统计）
  - `task`: 任务相关（成功率、强制完成）
  - `all`: 所有指标域

**执行流程：**

1. **缺口分析**：读取本文件的硬门槛和性能目标，搜索代码中 `LogUAVMetrics` 使用情况，对比下方"现有日志 TAG"清单，识别缺口。
2. **设计日志**：遵循 `[TAG] key=value` 格式，使用 `LogUAVMetrics` 类别，多 Agent 场景携带 `Agent=%d`，高频路径用 `UE_LOG_THROTTLE`，禁止 Verbose。
3. **实施修改**：优先在 `UAVPawn.h` 的 `// ---- 仿真指标累计器 ----` 区添加累计变量，采集逻辑封装到 `UpdateMetricsLog()`。每次修改后编译。
4. **验证**：`Script/sim.sh 40 8` → `rg "<NEW_TAG>" Logs/uav.log` 确认输出。

**现有日志 TAG 清单：**

| TAG | 位置 | 频率 | 覆盖指标 |
|---|---|---|---|
| `[NMPC_SOLVE]` | NMPCAvoidance.cpp | 每次求解 | NMPC 初值/代价/净空/进度/梯度残差/迭代/时间 |
| `[NMPC_RESULT]` | UAVPawn.cpp | 1s | Agent 维度 NMPC 类型/失败标志/卡死/求解诊断 |
| `[CBF_SOLVE]` | UAVPawn.cpp | CBF 触发时 | CBF MinH/残差/Slack/活跃约束/状态 |
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

---

## 核心规则

1. 先建立可比较基线，再修改代码。
2. 一轮只验证一个根因假设，不同时重构 NMPC、CBF 和姿态控制器。
3. 使用同一场景、种子、时长和 `slomo` 比较基线与候选。
4. 安全失败直接否决候选，不能用速度或跟踪改善抵消。
5. 不自动降低指标。达到迭代预算仍未达标时，报告证据和剩余风险。
6. 不把 `Mission completed` 单独视为成功；出现 `Overtime timeout` 或 `force completing` 即失败。
7. 不撤销用户已有修改。失败实验只精确撤销本轮由自己产生的改动。

## 并行子 Agent 策略

允许创建子 Agent 提高只读分析效率，但主 Agent 始终负责顺序控制、文件修改、构建、测试、仿真和最终取舍。

允许并行：

- 预检阶段按 `nmpc`、`safety`、`flight` 等领域只读扫描代码和 `git diff`。
- 基线仿真完成后，按速度、姿态、偏差、CBF、NMPC、任务完成等指标域并行提取日志。
- 形成假设前，按 NMPC、CBF、飞控、任务完成逻辑并行做只读根因分析。
- 候选验证后，并行复核硬门槛、指标对比、日志完整性和 README 更新需求。

禁止并行：

- 多个子 Agent 在同一工作区同时修改 C++、配置、测试或 README。
- 多个子 Agent 同时运行会覆盖 `Logs/uav.log` 或 `Logs/uav_full.log` 的仿真。
- 并行调参或并行修改 NMPC、CBF、姿态控制器后择优。
- 基线未建立、上一轮候选未被主 Agent 接受前启动下一轮实验。

每个子 Agent 输出必须包含读取的文件或日志、证据、对应指标、是否触发硬失败，以及一个可验证的单一假设。子 Agent 不直接决定接受候选。

## 调用参数

将用户参数映射为以下概念；缺省时使用括号内默认值：

- `focus`: `baseline`、`nmpc`、`safety`、`flight` 或 `all`（`all`）。
- `duration`: 仿真实时时长秒数（80）。
- `slomo`: UE 时间倍率（8）。
- `max_iterations`: 最大实验轮数（6）。
- `scenario`: 指定场景；未指定时使用当前默认场景。

用户明确指定目标、场景或预算时，以用户要求为准。

## 硬门槛

以下任一情况判定失败：

- `[Crash]`、`Obstacle penetration` 或机间碰撞。
- NaN、Inf、断言、崩溃或未处理的求解失败。
- `Overtime timeout`、`force completing`。
- Roll 或 Pitch 超过 60 度，或出现持续姿态失稳。
- 单次 Yaw 跳变超过 45 度，或连续 5 秒内 Yaw 振荡幅度超过 30 度（航向失稳）。
- 修改安全层后没有报告约束残差、slack 或求解状态。
- 正常控制路径在 CBF 输出后继续修改线加速度。
- 无人机横向偏差超过 15 米，或连续 20 秒偏差超过 10 米（严重偏离航线）。

硬门槛不得降级。

## 默认性能目标

按每个 Agent 的 `MaxVelocity` 归一化：

- 稳定巡航速度不低于 50%。
- 避障后 8 秒内恢复到 40%。
- 低于 30% 的单次持续时间不超过 15 秒。
- 默认横向偏差不超过 5 米。
- 不连续 20 秒负进度、原地徘徊或绕圈。
- Yaw 跳变超过 30° 的帧占比不超过 15%（航向稳定性）。
- 确定性回归场景任务成功率 100%。

同时报告 NMPC 求解时间 P50、P95、P99；不要只报告平均值。

## 工作流

### 1. 预检

1. 执行 `git status --short` 和相关文件的 `git diff`。
2. 识别用户未提交修改，后续与其共存。
3. 读取与 `focus` 对应的代码：
   - `nmpc`: `Planning/NMPCAvoidance.*`、`TrajectoryTracker.*`。
   - `safety`: `MultiAgent/CBFQPFilter.*`、`MultiAgentTypes.h`、`Core/UAVPawn.*`。
   - `flight`: `PositionController.*`、`AttitudeController.*`、`UAVDynamics.*`。
4. 检查日志是否能区分 Agent、真实成功、净空、进度和求解状态。
5. 若缺少判定当前问题所需的数据，先补最小结构化诊断，不凭猜测调参。

已知风险：当前 CBF-QP 二阶约束方向存在问题。若该实现处于启用状态，先修复或禁用，再进行性能优化。

### 2. 建立基线

修改 C++ 前执行：

```bash
# macOS/Linux
Script/build.sh
Script/test.sh
Script/sim.sh <duration> <slomo>

# Windows
Script\build.bat
Script\test.bat
Script\sim.bat <duration> <slomo>
```

若构建失败，先判断失败是否来自当前工作区已有修改。不要在没有可运行基线时开始调参。

保存本轮：

- 使用的 commit/工作区状态。
- 场景、种子、时长、`slomo`。
- `Logs/uav.log` 和必要的 `Logs/uav_full.log`。
- 指标摘要。

默认场景具有随机因素且没有固定种子时，不做精确 A/B 结论；先增加确定性场景或固定随机源。

### 3. 提取指标

先搜索硬失败：

```bash
rg -n "Crash|penetration|collision|NaN|Inf|Overtime timeout|force completing|Attitude anomaly|NumericalFailure" Logs/uav.log Logs/uav_full.log
```

再提取结构化指标（TAG 说明见"子命令 > add_log > 现有日志 TAG 清单"）：

```bash
# 速度指标
rg "\[SPEED_METRICS\]" Logs/uav.log
rg "\[LOW_SPEED_END\]" Logs/uav.log

# 姿态指标
rg "\[ATTITUDE\]" Logs/uav.log

# 航向指标
rg "\[YAW_METRICS\]|\[YAW_ANOMALY\]" Logs/uav.log

# 横向偏差与严重偏离
rg "\[DEVIATION_SEVERE\]" Logs/uav.log

# 避障恢复
rg "\[AVOIDANCE_END\]|\[AVOIDANCE_RECOVERY\]" Logs/uav.log

# 安全完整性
rg "\[CBF_INTEGRITY\]|\[CBF_SOLVE\]|\[SIM_RESULT\]" Logs/uav.log

# NMPC 求解统计
rg "\[NMPC_SOLVE\]|\[NMPC_PERCENTILE\]" Logs/uav.log

# 全局汇总
rg "\[SIM_SUMMARY\]|\[TASK_SUMMARY\]" Logs/uav.log
```

再提取：

- 每个 Agent 的型号和 `MaxVelocity`。
- 速度比例、低速区间和恢复时间。
- 轨迹进度、横向偏差、最终目标距离。
- Yaw 跳变幅度分布、航向漂移量、振荡次数。
- 最小实际和预测净空。
- 卡死、绕行侧切换和全局重规划次数。
- NMPC 初始/最终代价、进度、梯度残差、候选类型和 P50/P95/P99。
- CBF `MinH`、最大约束残差、slack、活跃约束和状态。

日志有节流时，缺少某条日志不能证明事件没有发生。优先使用结构化汇总或 CSV。

输出基线表：

| 指标 | 基线 | 目标 | 状态 |
|---|---:|---:|---|
| 碰撞/穿透 | | 0 | |
| 强制完成 | | 0 | |
| 最小净空 | | 场景要求 | |
| 最大姿态 | | <= 60 deg | |
| Yaw 跳变 > 30° 占比 | | <= 15% | |
| 成功率 | | 100% deterministic | |
| 巡航速度比例 | | >= 50% | |
| 最大低速持续 | | <= 15 s | |
| 最大横向偏差 | | <= 5 m | |
| 严重偏离航线 | | 偏差 < 15 m 且无 20s > 10 m | |
| Stuck/Replan | | 越低越好 | |
| NMPC P95 | | 控制周期内 | |

### 4. 形成单一假设

给出可以被本轮实验推翻的陈述，例如：

- “对称初值导致 PGD 无法选择绕行侧。”
- “CBF 后置限幅重新破坏了安全约束。”
- “平方误差改动后沿用了旧权重，导致终端项主导。”

不要使用“可能是参数不合适”这种不可验证假设。

修改前记录：

- 证据。
- 预计影响的指标。
- 不应变化的指标。
- 回滚条件。

### 5. 实施最小实验

- 优先修复模型、符号、量纲、状态机或控制所有权问题。
- 只有根因明确为参数尺度时才调参。
- 修改代价形式后同步重新标定相关权重，不孤立机械替换。
- 多初值优先使用确定性 `Warm/Nominal/Left/Right/Up/Down/Brake`。
- 只有设计文档中的 CEM 准入条件满足时，才加入低维 CEM。
- CBF 必须作为正常路径最后一个线加速度修改层。

诊断日志要求：

- 日志用英文，注释用中文。
- 高频日志使用 `UE_LOG_THROTTLE`。
- 优先输出一行结构化字段。
- 修复后删除临时日志，或降频为长期诊断。
- 不使用 `Verbose`。

### 6. 验证候选

每次 C++ 修改后严格执行：

1. 编译。
2. 运行相关单元测试；公共控制路径变更运行完整 `Script/test.*`。
3. 使用与基线完全相同的仿真参数运行。
4. 提取同一组指标。
5. 检查硬门槛。

接受候选必须同时满足：

- 无新增硬失败。
- 目标指标产生超过正常波动的改善。
- 非目标关键指标没有显著退化。
- 代码改变与实验假设一致。

否则只撤销本轮自身修改，不使用 `git reset --hard` 或整文件覆盖。

### 7. 迭代与停止

候选接受后，以其作为新基线继续下一轮。最多运行 `max_iterations` 轮。

提前停止条件：

- 所有硬门槛和用户目标达成。
- 证据表明问题属于场景、资产或外部依赖，当前代码无法继续推进。
- 连续两轮没有可测改善，需要回到模型或架构层重新诊断。

达到预算未达标时，不降低目标。输出：

- 最佳已验证版本。
- 基线与最佳结果对比。
- 已排除的假设。
- 当前阻塞根因。
- 下一项最高价值实验。

## 根因分流

| 现象 | 优先检查 |
|---|---|
| 对称障碍前停滞 | 路径进度、初值对称、homotopy、预测净空 |
| 走廊中无意义偏航 | 多初值偏置过强、侧记忆未正确清除 |
| 低速不恢复 | 参考时间推进、速度意图、后置限幅、卡死状态机 |
| 频繁 warm reset | 跨帧代价误比较、初值质量、线搜索诊断 |
| `SatRatio > 1` 或输出加速度超限 | 速度投影反推加速度后破坏最大加速度约束 |
| CBF 后仍接近/碰撞 | 约束符号、后置修改器、执行器可行域、感知延迟 |
| slack 长期非零 | CBF 增益过强、执行器能力不足、安全集不可达 |
| 姿态异常 | 加速度命令可行域、倾角/推力限制、姿态环，而非先调障碍权重 |
| Yaw 振荡/漂移 | YawPID 阻尼、航向参考基准、SO(3) 运动学耦合 |
| “任务完成”但偏差巨大 | 强制完成或超时逻辑，按失败处理 |

## 最终报告

最终回答必须包含：

1. 修改内容和对应根因。
2. 构建、测试和仿真是否成功。
3. 基线与最终指标对比。
4. 未解决风险。
5. 是否触发 CEM、CBF slack 或 degraded mode。
