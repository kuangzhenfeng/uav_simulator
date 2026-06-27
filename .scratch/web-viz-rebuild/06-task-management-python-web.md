# Slice 6 — 任务切割: Python 索引 + Web 任务选择器

- **状态**: ready-for-agent
- **Parent**: 00-prd.md

## What to build
Python 扫描 Logs/tasks/ 暴露 /api/tasks + /api/task/<id> + DELETE /api/task/<id>; Web 任务下拉 (时间/场景/PASS-FAIL) + 切换查看 + 删除。

## Acceptance criteria
- [ ] Web 切换不同任务, 3D 视图与判决刷新
- [ ] 删除任务后列表更新
- [ ] pytest: list_tasks 正确索引多个任务目录

## Blocked by
- 05-task-slicing-ue-archive.md
