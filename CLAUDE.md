# UAV Simulator 项目指令

## 设计原则

按照主流工业级做法进行方案设计与实施。分析解决问题时遵循工业级根治原则。

## 基本规范

- 默认用中文回答
- 注释用中文，日志用英文。
- 永远不要主动提交代码，除非我明确要求你这么做。如需提交代码，提交纪录不要带有Co-Authored-By。
- 新增或修改功能时，必须更新README.md中的功能特性描述，保持文档与代码的一致性。
- 修改README.md文档时最小化修改，尽量保持简洁。
- 如果需要在 UE 编辑器中添加或修改内容，使用官方 Unreal MCP 工具。
- 分析日志时需要注意：部分日志被降频了，所以可能不是每个时间点都有输出的。
- 分析时可以适当添加必要日志，知道你有足够把握定位到根因。
- 不考虑兼容问题，不考虑改动量，不要为了处理资产迁移风险而妥协，一切以最主流的工业级做法来实现。
- *.sh脚本和*.bat脚本的功能应该完全一样，保持一致性。
- 遵循奥卡姆剃刀原则，如无必要，勿增实体。

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

## Agent规范

- 如有需要，自行启动 UE 5.8 编辑器，使用官方 Unreal MCP 控制，并在完成后关闭自行启动的编辑器。
- 若WebSearch返回结果为空，尝试换用Fetch进行网络搜索
- 当调用grill-with-docs skill进行拷问时，请使用AskUserQuestion工具并给出推荐选项
- 当调用improve-codebase-architecture skill生成可视化网页时用中文，并且调用使用python启动一个本地服务器，返回访问地址
- 不要在代码中添加临时的、仅针对本次变更的说明性注释，但必要注释仍需要添加
- 不要做超过20000字节的大编辑。如果编辑失败了，把一个编辑拆分成多个较小的编辑
- 当给你代码review的诊断报告时，你需要基于代码实事进行分析，直到你有十足把握为止，对于不确定的地方禁止进行任何代码修改
- 分析大量日志时需要启动subagent进行上下文隔离，避免过多的日志污染上下文

## Unreal Engine 控制

本项目通过官方 `ModelContextProtocol` 和 `AllToolsets` 插件提供 Unreal MCP，编辑器默认监听 `http://127.0.0.1:8000/mcp`。Codex 配置位于 `.codex/config.toml`，Claude Code 配置位于 `.mcp.json`。

使用 UE 工具前先启动编辑器，通过 `list_toolsets`、`describe_toolset` 发现工具，再执行一次只读查询验证连接。工具调用必须串行执行。服务器未启动时可使用编辑器控制台命令 `ModelContextProtocol.StartServer`。

headless 仿真使用 `Script/sim.bat` / `Script/sim.sh` 和项目自身 HTTP 控制端，不依赖编辑器 MCP。

## Agent skills

### Issue tracker

本地 markdown：issues 以文件形式存放在 `.scratch/<feature>/` 下，不走 GitHub/GitLab。See `docs/agents/issue-tracker.md`.

### Triage labels

五个 triage 角色采用默认字符串：needs-triage / needs-info / ready-for-agent / ready-for-human / wontfix。See `docs/agents/triage-labels.md`.

### Domain docs

单上下文：根目录 `CONTEXT.md` + `docs/adr/`。See `docs/agents/domain.md`.
