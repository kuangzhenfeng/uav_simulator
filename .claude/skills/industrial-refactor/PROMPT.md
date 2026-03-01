# Industrial Refactor Skill - 执行提示

你现在正在执行 **Industrial Refactor Skill**，目标是系统性审查 UAV Simulator 项目中的简化实现，生成工业级重构计划。

## 执行步骤

### 步骤 1：解析参数

从用户输入中解析以下参数：
- `--scope`: 扫描范围（control, planning, sensors, dynamics, all）
- `--priority`: 优先级过滤（P0, P1, P2, all）

默认值：`--scope=all --priority=all`

### 步骤 2：代码扫描

根据 `--scope` 参数确定扫描范围：

**control 范围**：
- `Source/uav_simulator/Control/StateEstimator.*`
- `Source/uav_simulator/Control/AttitudeController.*`
- `Source/uav_simulator/Control/PositionController.*`

**planning 范围**：
- `Source/uav_simulator/Planning/NMPCAvoidance.*`
- `Source/uav_simulator/Planning/LinearMPCAvoidance.*`
- `Source/uav_simulator/Planning/TrajectoryOptimizer.*`

**sensors 范围**：
- `Source/uav_simulator/Sensors/IMUSensor.*`
- `Source/uav_simulator/Sensors/GPSSensor.*`

**dynamics 范围**：
- `Source/uav_simulator/Dynamics/UAVDynamics.*`

**all 范围**：
- `Source/uav_simulator/**/*.{cpp,h}`

执行以下扫描操作：

1. **使用 Grep 搜索关键词**：
   ```
   pattern: "简化|TODO|FIXME|HACK|简单实现|假设"
   path: <根据 scope 确定>
   output_mode: content
   context: 5
   ```

2. **使用 Glob 定位关键文件**：
   ```
   pattern: <根据 scope 确定的文件模式>
   ```

3. **使用 Read 读取关键文件**，提取完整上下文

### 步骤 3：影响评估

对每个识别的简化实现进行评估，使用以下规则：

| 简化类型 | 关键词/模式 | 严重性 | 范围 | 优先级 |
|---------|------------|--------|------|--------|
| 协方差矩阵对角化 | "对角"、"FVector" | 高 | 模块 | P0 |
| 线性 MPC 梯度简化 | "简化梯度"、"数值梯度" | 高 | 模块 | P0 |
| IMU 加速度假设为零 | "假设.*零"、"FVector::ZeroVector" | 高 | 模块 | P0 |
| Yaw 力矩线性化 | "线性化"、"简化力矩" | 中 | 局部 | P1 |
| 轨迹优化多项式简化 | "3次多项式"、"简单插值" | 中 | 模块 | P1 |
| GPS 噪声模型简化 | "高斯噪声"、"简化噪声" | 中 | 局部 | P1 |
| 硬编码控制频率 | "50.0f"、"硬编码" | 低 | 局部 | P2 |
| 固定有限差分步长 | "1e-6"、"固定步长" | 低 | 局部 | P2 |
| 欧拉积分 | "Euler"、"简单积分" | 低 | 局部 | P2 |

根据 `--priority` 参数过滤结果。

### 步骤 4：生成审查报告

创建文件 `Docs/IndustrialRefactorReport.md`，包含以下内容：

```markdown
# 工业级代码审查报告

生成时间：<当前时间>
扫描范围：<scope 参数>
优先级过滤：<priority 参数>

## 执行摘要

- 扫描文件数：<数量>
- 识别简化实现：<数量>
- 高优先级问题（P0）：<数量>
- 中优先级问题（P1）：<数量>
- 低优先级问题（P2）：<数量>

## 影响评估矩阵

| 文件 | 行号 | 简化内容 | 严重性 | 范围 | 优先级 |
|------|------|---------|--------|------|--------|
| <文件路径> | <行号> | <简化描述> | <高/中/低> | <局部/模块/全局> | <P0/P1/P2> |
...

## 详细扫描结果

### P0 优先级问题

#### 1. <问题标题>
- **文件**：<文件路径>:<行号>
- **函数**：<函数名>
- **简化内容**：<详细描述>
- **代码片段**：
  ```cpp
  <代码片段>
  ```
- **影响分析**：<影响描述>
- **工业级方案**：<改进建议>

...

### P1 优先级问题
...

### P2 优先级问题
...

## 分阶段重构计划

### 阶段 1：修复高严重性问题（P0）

**目标**：
1. <目标 1>
2. <目标 2>
...

**实施方案**：

#### 1.1 <方案标题>
- **文件**：<文件列表>
- **修改内容**：
  - <修改点 1>
  - <修改点 2>
- **验证方法**：
  - 单元测试：<测试内容>
  - 集成测试：<测试内容>
  - 仿真验证：<验证方法>
- **预估工作量**：<小时数>

...

**阶段 1 总工作量**：<小时数>

### 阶段 2：改进中严重性问题（P1）
...

### 阶段 3：优化低严重性问题（P2）
...

## 工业级实现参考

### 状态估计
- **完整 EKF 实现**：
  - 论文：Kalman, R. E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
  - 开源库：Eigen (矩阵运算)、FilterPy (Python 参考实现)
  - 关键改进：完整协方差矩阵、Cholesky 分解

- **UKF 实现**：
  - 论文：Julier, S. J., & Uhlmann, J. K. (1997). "Unscented Filtering and Nonlinear Estimation"
  - 优势：无需雅可比矩阵，适用于高度非线性系统

### 控制器
- **非线性动力学反演**：
  - 论文：Slotine, J. J. E., & Li, W. (1991). "Applied Nonlinear Control"
  - 实现：完整 12 状态模型、反馈线性化

- **鲁棒控制**：
  - 论文：Zhou, K., & Doyle, J. C. (1998). "Essentials of Robust Control"
  - 实现：H∞ 控制、μ-synthesis

### 规划器
- **完整 MPC**：
  - 论文：Camacho, E. F., & Alba, C. B. (2013). "Model Predictive Control"
  - 开源库：ACADO Toolkit、CasADi、OSQP
  - 实现：QP 求解器、自动微分

- **轨迹优化**：
  - 论文：Mellinger, D., & Kumar, V. (2011). "Minimum Snap Trajectory Generation and Control for Quadrotors"
  - 实现：7 阶多项式、QP 求解

### 数值计算
- **自动微分**：
  - 开源库：CppAD、ADOL-C、Eigen::AutoDiff
  - 优势：精确梯度、无步长选择问题

- **高阶积分**：
  - RK4、RK45（自适应步长）
  - 开源库：Boost.Numeric.Odeint

## 验证计划

### 单元测试
- 测试文件位置：`Source/uav_simulator/Tests/`
- 测试框架：Unreal Automation Test
- 覆盖率目标：≥ 80%

### 集成测试
- 端到端任务测试：起飞 → 航点导航 → 避障 → 降落
- 性能指标：
  - 跟踪误差：≤ 3m
  - 速度稳定性：无震荡
  - 避障成功率：≥ 95%

### 仿真验证
- 运行标准仿真场景：`cmd //c Script\sim.bat`
- 分析日志：`Logs/uav.log`
- 关键指标：
  - 状态估计误差
  - 控制器饱和次数
  - 求解器收敛时间
  - 姿态异常次数

### 性能基准测试
- EKF 更新：< 1ms
- MPC 求解：< 10ms
- 轨迹优化：< 50ms

## 风险与缓解

**风险 1：重构引入新 Bug**
- 缓解：完善的单元测试覆盖、渐进式重构、保留旧代码作为对照

**风险 2：性能下降**
- 缓解：性能基准测试、Profiler 监控、算法优化

**风险 3：工作量超预期**
- 缓解：分阶段执行、优先级排序、可选择性跳过 P2 任务

**风险 4：依赖外部库**
- 缓解：优先使用 Unreal 内置库、评估外部库的稳定性和许可证

## 下一步行动

1. 审查本报告，确认重构优先级
2. 选择执行阶段（建议从阶段 1 开始）
3. 为每个重构任务创建详细的实施计划
4. 执行重构，运行测试和仿真验证
5. 更新文档和代码注释
```

### 步骤 5：生成重构任务清单

创建文件 `Docs/RefactorTasks.md`，包含以下内容：

```markdown
# 工业级重构任务清单

生成时间：<当前时间>

## 阶段 1：修复高严重性问题（P0）

### 任务 1.1：完整 EKF 实现
- **优先级**：P0
- **文件**：
  - `Source/uav_simulator/Control/StateEstimator.h`
  - `Source/uav_simulator/Control/StateEstimator.cpp`
- **目标**：将协方差矩阵从 FVector 改为完整矩阵
- **验收标准**：
  - [ ] 协方差矩阵为 6×6 或 9×9 完整矩阵
  - [ ] 实现完整预测步：P = F*P*F^T + Q
  - [ ] 实现完整更新步：K = P*H^T*(H*P*H^T + R)^-1
  - [ ] 使用 Cholesky 分解提高数值稳定性
  - [ ] 单元测试通过
  - [ ] 仿真验证：状态估计误差 < 0.5m
- **预估工作量**：8-12 小时

### 任务 1.2：完整梯度计算
- **优先级**：P0
- **文件**：
  - `Source/uav_simulator/Planning/LinearMPCAvoidance.h`
  - `Source/uav_simulator/Planning/LinearMPCAvoidance.cpp`
- **目标**：实现伴随法计算状态梯度
- **验收标准**：
  - [ ] 实现伴随法计算梯度
  - [ ] 梯度精度验证（对比数值梯度）
  - [ ] 收敛速度提升 > 30%
  - [ ] 单元测试通过
  - [ ] 仿真验证：避障性能改善
- **预估工作量**：12-16 小时

### 任务 1.3：真实加速度获取
- **优先级**：P0
- **文件**：
  - `Source/uav_simulator/Sensors/IMUSensor.cpp`
- **目标**：从 UAVDynamics 获取真实加速度
- **验收标准**：
  - [ ] 从 UAVDynamics 组件获取真实加速度
  - [ ] 实现完整 IMU 误差模型
  - [ ] 单元测试通过
  - [ ] 仿真验证：加速度测量准确
- **预估工作量**：4-6 小时

**阶段 1 总工作量**：24-34 小时

## 阶段 2：改进中严重性问题（P1）

### 任务 2.1：完整电机动力学模型
...

### 任务 2.2：最小 snap 轨迹优化
...

**阶段 2 总工作量**：30-40 小时

## 阶段 3：优化低严重性问题（P2）

### 任务 3.1：动态控制频率适应
...

### 任务 3.2：自适应有限差分步长
...

**阶段 3 总工作量**：16-20 小时

## 总工作量估算

- **阶段 1**：24-34 小时
- **阶段 2**：30-40 小时
- **阶段 3**：16-20 小时
- **总计**：70-94 小时

## 执行建议

1. 优先执行阶段 1（P0 问题），这些问题影响系统稳定性
2. 每个任务完成后立即运行测试和仿真验证
3. 使用 git 分支管理每个任务的修改
4. 保留旧代码作为对照，便于性能对比
5. 更新文档和代码注释
```

### 步骤 6：输出结果

向用户报告：
1. 扫描完成，识别了多少个简化实现
2. 生成的文件路径：
   - `Docs/IndustrialRefactorReport.md`
   - `Docs/RefactorTasks.md`
3. 建议用户审查报告，确认重构优先级
4. 询问用户是否需要执行特定阶段的重构

## 注意事项

1. **不要自动修改代码**：该 Skill 仅生成报告，不执行重构
2. **保持客观**：基于代码事实进行评估，不做主观臆断
3. **提供具体方案**：每个问题都应有明确的工业级改进方案
4. **考虑可行性**：工作量估算应合理，避免过于乐观
5. **遵循项目规范**：注释用中文，日志用英文
