# UAV Simulator 项目指令

## 设计原则

按照主流工业级做法进行方案设计与实施。

## 基本规范

- 注释用中文，日志用英文。
- 永远不要主动提交代码，除非我明确要求你这么做。

## 日志规范

- 禁止使用Verbose等级的日志，除非我明确要求你这么做。
- 如果日志输出过于频繁，请使用UE_LOG_THROTTLE来限制输出频率，或者直接删除不必要的日志。

## 编译与仿真

- Windows: 使用 `Script\build.bat` 编译，使用 `Script\sim.bat` 运行仿真，使用 `Script\test.bat` 运行单元测试。
- macOS/Linux: 使用 `Script/build.sh` 编译，使用 `Script/sim.sh` 运行仿真，使用 `Script/test.sh` 运行单元测试。

## 工作流

每次修改 C++ 代码后，必须执行以下循环：

1. **编译**
2. **编译失败** → 读取错误输出 → 修复代码 → 重新编译
3. **编译成功** → **运行仿真**
4. **分析日志**: 读取 `Logs\uav.log`，深度分析日志，从以下多个维度综合评估：
   - 与问题高度相关的日志
   - **速度**：巡航速度是否达标，避障时速度损失是否合理
   - **稳定性**：姿态/速度是否震荡，控制输出是否平滑
   - **避障轨迹合理性**：绕障路径是否流畅，有无不必要的大幅偏转或原地徘徊
   - **轨迹跟踪误差**：实际轨迹与期望轨迹的偏差是否在合理范围内（≤ 3m）
5. **发现问题** → 找出根本问题，修复代码，必要时可以添加重要日志 → 回到步骤 1
6. **行为正常** → 完成

## Unreal Engine control

`soft-ue-cli` controls this UE project via the SoftUEBridge plugin (installed via pipx).
Run `soft-ue-cli --help` to see all available commands.
The game or editor must be running with SoftUEBridge enabled before using UE commands.

After the user rebuilds and launches UE, verify with:
  `soft-ue-cli check-setup`
