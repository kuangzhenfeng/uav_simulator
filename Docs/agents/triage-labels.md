# Triage 标签词表

`triage` skill 处理一个 issue 时，通过状态机推进：待评估 → 等待报告人 → AFK agent 可接手 → 需人工实现 → 不予处理。各角色字符串与名称一致，无重命名映射。

## 五个标准角色

| 角色 | 字符串 | 含义 |
|------|--------|------|
| 待评估 | `needs-triage` | 维护者需要评估 |
| 等待报告人 | `needs-info` | 等待提交者补充信息 |
| AFK 可接手 | `ready-for-agent` | 描述充分，agent 可无需人工上下文直接接手 |
| 需人工实现 | `ready-for-human` | 需要人工实现 |
| 不予处理 | `wontfix` | 不会处理 |

## 本地 markdown 模式

由于采用本地 markdown issue 跟踪器（见 `issue-tracker.md`），标签以 issue 文件中"状态"字段值的形式存在，无需在某个平台上预先配置 label。
