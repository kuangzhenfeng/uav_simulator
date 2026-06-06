# UAV Simulator 项目指令

## 设计原则

按照主流工业级做法进行方案设计与实施。

## 基本规范

- 注释用中文，日志用英文。
- 永远不要主动提交代码，除非我明确要求你这么做。
- 新增或修改功能时，需要更新README.md中的功能特性描述，保持文档与代码的一致性。
- README.md文档修改尽量保持简洁，最小化修改。
- 如果有需要再UE的编辑器上才能添加或者修改的，可以调用soft-ue-cli进行修改

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
4. **分析日志**: 读取 `Logs\uav.log`或者`Logs\uav_full.log`：
5. **发现问题** → 找出根本问题，修复代码，必要时可以添加重要日志 → 回到步骤 1
6. **行为正常** → 完成

## Unreal Engine control

`soft-ue-cli` controls this UE project via the SoftUEBridge plugin (installed via pipx).
Run `soft-ue-cli --help` to see all available commands.
The game or editor must be running with SoftUEBridge enabled before using UE commands.

After the user rebuilds and launches UE, verify with:
  `soft-ue-cli check-setup`
