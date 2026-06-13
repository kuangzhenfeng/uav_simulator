# 仿真判决参考

仅在性能调优、安全改动、回归验证或 `add_log` 时读取本文件。

## 硬失败

以下任一情况直接否决候选：

- `[Crash]`、`Obstacle penetration`、机间碰撞、NaN、Inf、断言、崩溃或未处理的求解失败。
- `Overtime timeout` 或 `force completing`。
- Roll/Pitch 超过 60 度或持续姿态失稳。
- 单次 Yaw 跳变超过 45 度，或连续 5 秒 Yaw 振荡超过 30 度。
- 横向偏差超过 15 米，或连续 20 秒偏差超过 10 米。
- 修改安全层后没有约束残差、slack 或求解状态。
- CBF 输出后正常路径继续修改线加速度。

日志有节流时，缺少一条事件日志不能证明事件未发生。优先使用结构化汇总。

## 默认性能目标

用户或场景有明确目标时覆盖以下默认值：

- 稳定巡航速度不低于每个 Agent `MaxVelocity` 的 50%。
- 避障后 8 秒内恢复到 40%。
- 低于 30% 的单次持续时间不超过 15 秒。
- 横向偏差不超过 5 米。
- 不连续 20 秒负进度、原地徘徊或绕圈。
- Yaw 跳变超过 30 度的帧占比不超过 15%。
- 确定性回归场景成功率 100%。
- NMPC 求解时间报告 P50、P95、P99。

## 按症状选择指标

| 症状 | 优先 TAG | 重点检查 |
|---|---|---|
| 碰撞、净空不足 | `CBF_SOLVE`、`CBF_INTEGRITY`、`SIM_RESULT` | 约束方向、残差、slack、后置修改器 |
| 对称障碍前停滞 | `NMPC_SOLVE`、`NMPC_RESULT` | 初值对称、进度、代价、梯度残差 |
| 低速不恢复 | `SPEED_METRICS`、`LOW_SPEED_END`、`AVOIDANCE_RECOVERY` | 参考推进、速度意图、限幅、卡死状态 |
| 姿态异常 | `ATTITUDE` | 加速度可行域、倾角/推力限制、姿态环 |
| Yaw 振荡或漂移 | `YAW_METRICS`、`YAW_ANOMALY` | 航向参考、阻尼、SO(3) 耦合 |
| 偏离航线 | `DEVIATION_SEVERE`、`SIM_SUMMARY` | 跟踪误差、强制完成、超时 |
| 求解变慢或失败 | `NMPC_SOLVE`、`NMPC_PERCENTILE` | P50/P95/P99、迭代、线搜索、warm reset |
| 任务完成异常 | `TASK_SUMMARY`、`SIM_RESULT` | 成功数、超时、force completing |

## 日志 TAG

| TAG | 位置 | 覆盖指标 |
|---|---|---|
| `[NMPC_SOLVE]` | `NMPCAvoidance.cpp` | 初值、代价、净空、进度、残差、迭代、时间 |
| `[NMPC_RESULT]` | `UAVPawn.cpp` | Agent 维度结果、失败标志、卡死 |
| `[CBF_SOLVE]` | `UAVPawn.cpp` | MinH、残差、slack、活跃约束、状态 |
| `[CBF_INTEGRITY]` | `UAVPawn.cpp` | CBF 后加速度是否被修改 |
| `[SPEED_METRICS]` | `UAVPawn.cpp` | 速度、MaxVel、比例、低速持续 |
| `[LOW_SPEED_END]` | `UAVPawn.cpp` | 低速区间结束和持续时间 |
| `[AVOIDANCE_END]` | `UAVPawn.cpp` | 避障结束时间 |
| `[AVOIDANCE_RECOVERY]` | `UAVPawn.cpp` | 恢复时间和状态 |
| `[ATTITUDE]` | `UAVPawn.cpp` | Roll/Pitch |
| `[YAW_METRICS]` | `UAVPawn.cpp` | 跳变、漂移、振荡 |
| `[YAW_ANOMALY]` | `UAVPawn.cpp` | Yaw 异常事件 |
| `[DEVIATION_SEVERE]` | `UAVPawn.cpp` | 严重横向偏差 |
| `[SIM_RESULT]` | `UAVPawn.cpp` | 强制完成、炸机 |
| `[NMPC_PERCENTILE]` | `UAVPawn.cpp` | NMPC P50/P95/P99 |
| `[SIM_SUMMARY]` | `UAVPawn.cpp` | 全局仿真汇总 |
| `[TASK_SUMMARY]` | `AgentManager.cpp` | 任务完成汇总 |

## `add_log` 检查

1. 搜索现有 `LogUAVMetrics` 和目标 TAG，避免重复埋点。
2. 优先扩展已有结构化汇总，不新增高频逐帧日志。
3. 只记录能影响当前决策的字段。
4. 状态变量放在实际拥有该状态的类中，不默认集中到 `UAVPawn`。
5. 编译并运行 20 秒 smoke，确认 TAG 出现且字段可解析。
