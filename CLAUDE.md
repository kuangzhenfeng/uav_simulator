# UAV Simulator - Claude Code 项目指令

## 重要：Shell 环境

不要使用 MSYS bash 执行命令。所有脚本和命令必须通过 `cmd //c` 在 Windows cmd 中运行。

## 工作流

每次修改 C++ 代码后，必须执行以下循环：

1. **编译**: `cmd //c Script\\build.bat`
2. **编译失败** → 读取错误输出 → 修复代码 → 重新编译
3. **编译成功** → **运行仿真**: `cmd //c Script\\sim.bat`
4. **分析日志**: 读取 `Logs\uav.log`，关注以下内容：
   - 与问题高度相关的日志
   - `[NMPC]` 状态变化（Tracking/Avoidance 切换）
   - `Stuck` 标记（无人机卡住）
   - `ObsCost` 值（障碍物代价）
   - `Warning` / `Error` 消息
5. **发现问题** → 修复代码 → 回到步骤 1
6. **行为正常** → 完成

## 可选：单元测试

运行已有的测试套件验证算法正确性：
```
cmd //c Script\\test.bat
```
