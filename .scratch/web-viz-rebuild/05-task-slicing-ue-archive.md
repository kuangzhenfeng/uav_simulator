# Slice 5 — 任务切割: UE 存储归档

- **状态**: ready-for-agent
- **Parent**: 00-prd.md

## What to build
TelemetryRecorder 改为每次仿真归档到 Logs/tasks/<时间戳>_<场景名>/ (telemetry.ndjson + result.json), 保留 Logs/telemetry.ndjson 镜像当前任务。

## Acceptance criteria
- [ ] 连跑两次仿真, Logs/tasks/ 出现两个独立目录各含 ndjson + result.json
- [ ] Logs/telemetry.ndjson 镜像最新任务

## Blocked by
- 01-sphere-tracer-bullet.md
