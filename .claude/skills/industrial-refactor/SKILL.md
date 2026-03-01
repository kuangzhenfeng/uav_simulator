# Industrial Refactor Skill

## Description
系统性审查项目中的简化实现，生成工业级重构计划。

该 Skill 通过自动扫描代码中的简化标记（注释关键词、隐含假设），生成详细的审查报告，并提供分阶段的重构计划，支持增量式改进。

## Usage
```
/industrial-refactor [--scope=<module>] [--priority=<P0|P1|P2>]
```

## Parameters
- `--scope`: 限定扫描范围（可选值：control, planning, sensors, dynamics, all）
  - `control`: 仅扫描控制器相关代码（StateEstimator, AttitudeController, PositionController）
  - `planning`: 仅扫描规划器相关代码（NMPCAvoidance, LinearMPCAvoidance, TrajectoryOptimizer）
  - `sensors`: 仅扫描传感器相关代码（IMUSensor, GPSSensor）
  - `dynamics`: 仅扫描动力学模型代码（UAVDynamics）
  - `all`: 扫描所有代码（默认）

- `--priority`: 限定优先级（可选值：P0, P1, P2, all）
  - `P0`: 仅显示高严重性问题（必须修复）
  - `P1`: 仅显示中严重性问题（应该修复）
  - `P2`: 仅显示低严重性问题（可以优化）
  - `all`: 显示所有优先级（默认）

## Workflow

### 1. 代码扫描器（Code Scanner）
扫描所有 `.cpp` 和 `.h` 文件，识别简化标记：
- 注释关键词：`简化`、`TODO`、`FIXME`、`HACK`、`简单实现`、`假设`
- 代码模式：硬编码常量、固定步长、对角矩阵、标量运算

### 2. 影响评估器（Impact Assessor）
评估每个简化实现的影响：
- **严重性**：高（影响系统稳定性）、中（影响性能）、低（影响可维护性）
- **范围**：局部（单个函数）、模块（单个组件）、全局（多个组件）
- **优先级**：P0（必须修复）、P1（应该修复）、P2（可以优化）

### 3. 重构规划器（Refactor Planner）
生成分阶段重构计划：
- **阶段 1**：修复高严重性问题（P0）
- **阶段 2**：改进中严重性问题（P1）
- **阶段 3**：优化低严重性问题（P2）

### 4. 知识库（Knowledge Base）
提供工业级实现的参考资料：
- 状态估计：完整 EKF、UKF、粒子滤波
- 控制器：非线性反演、反馈线性化、鲁棒控制
- 规划器：完整 MPC、QP 求解器、轨迹优化
- 数值计算：自动微分、自适应步长、高阶积分

## Output
生成以下文件：

1. **审查报告**：`Docs/IndustrialRefactorReport.md`
   - 扫描结果（所有识别的简化实现）
   - 影响评估矩阵
   - 分阶段重构计划
   - 工业级实现参考

2. **重构任务清单**：`Docs/RefactorTasks.md`
   - 阶段 1 任务（P0）
   - 阶段 2 任务（P1）
   - 阶段 3 任务（P2）
   - 每个任务的验收标准

## Examples

### 扫描所有代码，显示所有优先级
```
/industrial-refactor
```

### 仅扫描控制器代码，显示高优先级问题
```
/industrial-refactor --scope=control --priority=P0
```

### 仅扫描规划器代码
```
/industrial-refactor --scope=planning
```

## Notes
- 该 Skill 不会自动修改代码，仅生成审查报告和重构计划
- 用户需要根据报告手动执行重构，或明确要求 Claude 执行特定阶段的重构
- 建议按阶段渐进式执行重构，避免一次性大规模修改
- 每个阶段完成后应运行完整的测试和仿真验证
