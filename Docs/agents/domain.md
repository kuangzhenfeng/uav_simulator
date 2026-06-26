# 领域文档

部分 skill（`improve-codebase-architecture`、`diagnosing-bugs`、`tdd`）会读取领域文档，以了解项目的领域语言与历史架构决策。

## 布局：单上下文

本仓库为**单上下文**（非 monorepo）：

- `CONTEXT.md`（仓库根）— 项目术语表（glossary），定义本上下文特有的领域概念与应避免的同义词。
- `docs/adr/`（仓库 `Docs/adr/`，macOS 大小写不敏感）— 架构决策记录（ADR），每条决策一个文件，编号命名，例如 `0001-scenario-as-dataasset.md`、`0002-windfield-as-scene-singleton.md`。

## Skill 消费规则

- 读取领域语言：读 `CONTEXT.md`。
- 查阅架构决策：读 `docs/adr/` 下的 ADR 文件。
- 本仓库无 `CONTEXT-MAP.md`（非多上下文布局），无需查找。
