# 可视化前端功能演进：跨 Run 对比 · 诊断面板 · 参数扫描

- **状态**: ready-for-agent
- **类型**: PRD
- **创建**: 2026-06-27
- **来源**: 前端功能 grilling 讨论

关联 ADR：ADR-0001（场景资产化）、ADR-0002（风场场景级单例）。
关联前序 PRD：`.scratch/web-control-panel/00-prd.md`（控制面板已完成，本项目在其基础上演进）。

---

## Problem Statement

当前可视化前端（`Tools/vis/web/`，约 2,600 行纯 Vanilla ES Module + Three.js）已实现了完整的 3D 可视化、实时遥测 SSE 推流、场景配置表单、实时调参滑块、预设管理的能力。但仍存在三个关键功能缺口，使得仿真工程师的工作流断裂：

**1. 调参闭环缺少"对比"这一步（🟥 P0）**

工程师的典型工作流是：调 PID/CBF 参数 → 跑仿真 → 看结果 → 改参数 → 再跑。但当前每次重跑后，上一次的遥测数据和判决结果**完全消失**——前端没有运行记忆。工程师只能靠人脑记住上次 Kp=0.03 时速度超调了多少，然后对比当前 Kp=0.05 的结果。没有并排对比视图，没有指标叠加图。

**2. 验收判决维度太薄（🟨 P1）**

验收面板只展示 PASS/FAIL 和一个汇总表格。但 `telemetry.ndjson` 里实际记录了远比这丰富的诊断数据——NMPC 每帧求解状态（收敛/线搜索失败/卡死逃逸）、CBF-QP 的 slack 触发频次和最大 slack 值、各子模块（RK4/NMPC/CBF/传感器）的逐帧耗时、机间净空的最小值分布。这些数据**在 ndjson 里有，但在前端完全不可见**。目前只能 SSH 到机器翻日志。

**3. 无法批量探索参数空间（🟩 P2）**

工程师想知道"风速 5/10/15/20 m/s 时验收能否通过"或"CBF dSafe 从 1m 到 5m，通过率怎么变"。目前只能逐次手动改参数 → 手动重跑 → 手动记录结果。没有参数扫描机制，没有结果聚合视图。

## Solution

在现有架构上扩展三个功能层，保持零构建 vanilla ES Module 路线，**不引入前端框架**，**不改变现有遥测协议**。

### 1. 跨 Run 对比（Pass-Through Comparison）

每次仿真完成后，后端保留该次运行的遥测快照与判决元数据，编入运行历史。前端支持多选 run → 并排查看指标曲线（速度/高度/净空/偏差叠加绘制，不同 run 用不同色阶）、汇总指标 diff 表格、验收判决对比。

### 2. 诊断面板（Diagnostic Dashboard）

将 `telemetry.ndjson` 中已有的诊断级数据聚合后暴露到前端：NMPC 求解成功率和失败分类饼图、CBF slack 激活频率和峰值、控制链路各模块耗时分布（avg/P95/max）、per-agent 轨迹跟踪偏差统计。

### 3. 参数扫描（Parameter Sweep）

前端构建参数扫描定义（一个或多个维度的参数变化范围），提交给后端 SimManager 以队列方式依次运行每个变体，结果聚合为表格或热力图展示——行是参数变体，列是验收指标，色阶标注好坏。

## User Stories

### 跨 Run 对比

1. 作为仿真工程师，我想在控制面板重跑后，上一次的运行数据仍保留在侧边栏运行列表中，这样不必靠记忆或截图。
2. 作为仿真工程师，我想勾选 2-4 条运行记录进入对比视图，这样能直观看到不同参数配置下的行为差异。
3. 作为仿真工程师，我想在速度/高度/净空/偏差图表上叠加多条曲线（每 run 一条，带 label），这样能一眼看出哪组参数超调更小。
4. 作为仿真工程师，我想看到 diff 表格（指标名 | Run A | Run B | 差异率），这样能快速判断参数调整方向是否正确。
5. 作为仿真工程师，我想在 3D 视图中切换显示某条 run 的历史轨迹，这样能对比不同参数的实际飞行路径。
6. 作为仿真工程师，我想在对比视图中锁定某条 run 为 baseline，这样其他 run 的指标都能显示相对于 baseline 的变化百分比。
7. 作为仿真工程师，我想删除不再需要的运行记录，这样列表不会被过期的测试数据淹没。

### 诊断面板

8. 作为仿真工程师，我想看到 NMPC 求解收敛率、线搜索失败率、卡死逃逸率，这样能快速评估求解器在当前场景下的健康度。
9. 作为仿真工程师，我想看到 CBF-QP 的 active-frame 比例和最大 slack 值，这样能判断安全约束是否过于激进或过于宽松。
10. 作为仿真工程师，我想看到各控制模块（RK4 / NMPC / CBF / 传感器 / 姿态控制）的耗时分布柱状图（avg / P95 / max），这样能定位性能瓶颈。
11. 作为仿真工程师，我想看到 Per-Agent 的最大横向偏差和航点到达时间，这样能逐机评估任务执行质量。
12. 作为仿真工程师，我想看到安全失败事件时间线（什么时候触发了 BrakeFallback / 振荡检测 / 位置卡死），这样能定位故障发生的具体时刻。
13. 作为仿真工程师，我想在诊断面板中按 Agent 过滤，这样多机场景下能逐一诊断。

### 参数扫描

14. 作为仿真工程师，我想定义参数维度（如"风速 [5,10,15,20] m/s"或"PID Kp [0.01, 0.03, 0.05]"），这样能系统化地探索参数空间。
15. 作为仿真工程师，我想一键提交扫描任务，后端自动依次运行每个变体，这样不用手动反复改参数和点重跑。
16. 作为仿真工程师，我想在扫描进行中看到进度条（3/10 完成）和当前运行状态，这样知道还要等多久。
17. 作为仿真工程师，我想看到扫描结果表格（行 = 参数变体，列 = 验收指标 + PASS/FAIL），这样能快速定位 sweet spot。
18. 作为仿真工程师，我想用色阶热力图渲染结果表格（绿=好/红=差），这样视觉上一眼看出参数空间的安全域和危险域。
19. 作为仿真工程师，我想从扫描结果中选取两行进入跨 Run 对比视图，这样能深入对比两个感兴趣的参数点。
20. 作为仿真工程师，我想在扫描进行中取消剩余任务（当前 run 跑完即停），这样发现方向错了不必浪费算力。
21. 作为仿真工程师，我想将扫描结果导出为 CSV，这样能在外部工具中做进一步分析。

## Implementation Decisions

### 数据架构

- **Run 元数据文件**：每次仿真在 `Logs/runs/<run_id>.json` 写一份元数据文件，含：
  - `runId`、`timestamp`、`durationMs`
  - `params`（发起时的 Scenario DTO 快照，含机队/障碍/风场/仿真参数）
  - `result`（Pass/Fail + ScenarioEvaluator 的完整指标）
- **遥测文件复用**：TelemetryRecorder 继续写 `Logs/telemetry.ndjson`，**每次 run 开始 truncate**（行为不变，向后兼容）。Run 完成后，后端将 ndjson 归档到 `Logs/runs/<run_id>.ndjson`。
- **诊断聚合**：Python 端从归档 ndjson 解析如下指标：
  - `nmpc_convergence`：求解成功/失败的逐帧计数与分类
  - `cbf_stats`：active-frame 比例、slack 均值/峰值、激活频次
  - `module_timing`：各模块 avg/P95/max 耗时
- **运行索引**：`Logs/runs/index.json` 按时间逆序存储 run metadata 摘要（runId、timestamp、params 摘要、result 摘要），后端启动时加载，新增 run 时追加。

### 后端 API 扩展（扩展 `server.py`）

| 端点 | 方法 | 说明 |
|---|---|---|
| `/api/runs` | GET | 返回运行列表（最近 N 条，支持分页） |
| `/api/runs/<id>` | GET | 单 run 完整详情（含 telemetry 解析后的时间序列） |
| `/api/runs/compare` | GET | 接受 `?ids=a,b,c` 返回聚合对比数据 |
| `/api/sweep` | POST | 提交参数扫描定义，返回 sweepId |
| `/api/sweep/<id>/status` | GET | 扫描进度（已完成/总数/当前状态） |
| `/api/sweep/<id>/results` | GET | 扫描聚合结果 |
| `/api/sweep/<id>/cancel` | POST | 扫描进行中取消 |

- **不修改**现有 `/api/data`、`/api/stream`、`/api/schema`、`/api/control/*` 的行为与响应格式。
- `/api/sweep` 接受如下参数扫描定义：

```python
# 原型片段 — 编码了扫描定义的 schema 契约
{
  "base_dto": { ... },          # 基线 Scenario DTO
  "dimensions": [
    { "path": "wind.steady[0]", "values": [5, 10, 15, 20], "label": "风速 x (m/s)" },
    { "path": "fleet[0].pid.kp", "values": [0.01, 0.03, 0.05], "label": "Roll Kp" }
  ],
  "duration": 60,
  "slomo": 8
}
```

- SimManager 扩展为队列模式：接受扫描任务后依次启动每个变体（当前 run 结束 → 下一个 run 启动）。失败不中断，标记 error 继续下一变体。

### 前端模块扩展（`Tools/vis/web/` 新增 ES Module）

| 模块 | 职责 |
|---|---|
| `runs.js` | 运行历史列表 + 多选勾选 + 删除 + 导航到对比/诊断视图 |
| `compare.js` | 多 run 指标叠加图表 + diff 表格 + 3D 轨迹切换 |
| `diagnostics.js` | NMPC/CBF/模块耗时诊断面板，按 agent 过滤 |
| `sweep.js` | 参数扫描定义表单 + 进度条 + 热力图结果表格 |

- **入口集成**：在现有 `#panel` 右侧栏顶部新增 tab 栏（`实时` / `历史` / `诊断` / `扫描`），实时 tab 是当前已有的实时指标面板，其他三个是新功能。不改 `#topbar` 和 `#control-panel` 的结构。
- **复用 charts.js**：对比视图的叠加图表复用现有 Canvas 图表绘制基础设施，新增多系列支持。
- **复用 scene.js**：3D 视图中用不同颜色渲染不同 run 的历史轨迹（从 `runs/<id>.ndjson` 回放）。

### 一致性约束

- 现有只读可视化、SSE 实时流、`sim.sh` 退出码协议（0/1/2）行为**完全不变**。
- 运行历史文件**不自动清理**——用户定期手动处理或用脚本归档。
- 所有新增术语遵循 `CONTEXT.md` 词汇（Scenario、ScenarioEvaluator、AcceptanceCriteria、ObstacleLayout 等）。
- 前后端单位契约不变：前端展示用米/m/s，后端 API 返回米制，归档 ndjson 保持 UE cm 原生单位（归档时的坐标不变换，与 `telemetry.ndjson` 保持一致）。

## Testing Decisions

测试哲学：只测外部可观察行为，不测实现细节。优先复用现有 seam。

### Python 后端 — pytest 单测（现有 seam 扩展）

- **运行归档**：模拟 ndjson → 归档到 `Logs/runs/<id>.ndjson` → 查询 `/api/runs/<id>` 返回正确数据。
- **对比聚合**：喂入两组已知 ndjson → `/api/runs/compare` 返回的统计值与手算一致。
- **扫描队列**：mock SimManager → 验证 N 个变体依次排队执行、单个失败不阻断后续、取消正确停止。
- 沿用现有 `test_server.py` 的测试基础设施（mock HTTP handler + 临时目录）。

### 诊断指标聚合

- 构造含指定 NMPC/CBF/timing 事件的 ndjson → 聚合结果与手算一致。
- 断言缺失字段时的降级行为（返回 null 或部分数据，不报 500）。

### 前端 — 人工验证清单（不引入新测试基础设施）

关键验证点：
1. 跨 Run 对比：多选 2 条 run → 图表叠加 → 标签/颜色正确 → diff 数值与详情页一致。
2. 诊断面板：指标数值与 `Logs/telemetry.ndjson` 原始数据抽样手算一致。
3. 参数扫描：3 变体扫描 → 进度 0/3 → 1/3 → 2/3 → 3/3 → 热力图渲染正确 → CSV 导出内容正确。

## Out of Scope

- **多用户并发访问**：本机单用户工具，后端不处理并发冲突。
- **实时对比**：当前只支持历史 run 对比，不支持"边跑边与历史 run 实时叠加"。
- **大规模性能优化**：运行列表超过 500 条时的分页性能未优化（当前场景 unlikely）。
- **框架迁移**：不使用 React/Vue/Svelte，保持 vanilla ES Module。
- **运行历史自动清理/归档策略**：用户自行管理。
- **WebSocket**：SSE + HTTP 轮询足够，不引入 WebSocket。
- **UE 端改动**：本项目所有数据来自已有的 TelemetryRecorder 和 ScenarioEvaluator 输出，不修改 UE C++ 代码。
- **前端单元测试**：沿用现有零测试现状，不引入 vitest/Playwright。

## Further Notes

- 本项目是 `.scratch/web-control-panel/00-prd.md` 的自然后续：控制面板解决了"配置 → 重跑 → 调参"的链路，本项目补齐"记录 → 对比 → 诊断 → 探索"的闭环。
- 后端新增 API 端点均为 `Tools/vis/server.py` 的 `Handler` 类扩展，不引入 FastAPI/Flask 等 WSGI 依赖（沿用标准库 `ThreadingHTTPServer`）。
- 参数扫描的 SimManager 队列模式复用现有 `start()`/`stop()` 生命周期，仅在队列调度层新增 FIFO 缓冲。
- 实施建议按 P0（运行历史基础设施 + 跨 Run 对比）→ P1（诊断面板）→ P2（参数扫描）推进，每阶段独立可验证、可提交。
- 新增内容应新增 ADR：运行历史存储格式与归档策略。
