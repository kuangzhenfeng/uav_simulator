# Issue 跟踪器：本地 markdown

本仓库**不**使用 GitHub/GitLab Issues。Issues 以 markdown 文件形式存放在仓库内的 `.scratch/` 目录下。

## 存放位置

- 每个功能/主题一个目录：`.scratch/<feature>/`
- 每个 issue 一个文件：`.scratch/<feature>/<NN>-<slug>.md`（`NN` 为两位编号，例如 `01-fix-wind-drift.md`）

## 文件约定

每个 issue 文件建议包含以下结构：

```markdown
# <issue 标题>

- **状态**: needs-triage | needs-info | ready-for-agent | ready-for-human | wontfix | done
- **创建**: <日期>
- **来源**: <谁提出，或哪次讨论>

## 描述
<问题或需求的描述>

## 验收标准
- <可验证的条件>
```

## Skill 行为约定

- `to-issues` / `triage` / `to-prd` / `qa`：读写 `.scratch/` 下的文件，**不**调用 `gh` / `glab`。
- 本地 markdown 模式无 PR 概念，外部 PR 不作为 triage 来源（GitHub/GitLab 的 "PRs as a request surface" 选项不适用）。
- 状态流转通过修改文件中的"状态"字段实现，对应字符串见 `triage-labels.md`。
