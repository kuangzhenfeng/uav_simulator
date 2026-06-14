# Issue 跟踪:GitHub

本仓库的 issues 与 PRD 以 GitHub issues 形式存在。所有操作均使用 `gh` CLI。

## 约定

- **创建 issue**:`gh issue create --title "..." --body "..."`。多行正文使用 heredoc。
- **读取 issue**:`gh issue view <number> --comments`,用 `jq` 过滤评论,同时获取标签。
- **列出 issues**:`gh issue list --state open --json number,title,body,labels,comments --jq '[.[] | {number, title, body, labels: [.labels[].name], comments: [.comments[].body]}]'`,配合相应的 `--label` 与 `--state` 过滤。
- **评论 issue**:`gh issue comment <number> --body "..."`
- **添加 / 移除标签**:`gh issue edit <number> --add-label "..."` / `--remove-label "..."`
- **关闭**:`gh issue close <number> --comment "..."`

仓库从 `git remote -v` 推断 —— `gh` 在 clone 内运行时会自动识别。

## 当 skill 要求"发布到 issue tracker"时

创建一个 GitHub issue。

## 当 skill 要求"获取相关工单"时

运行 `gh issue view <number> --comments`。
