# Slice 7 — 回放/验收面板 + 端到端验证

- **状态**: ready-for-agent
- **Parent**: 00-prd.md

## What to build
硬化 SSE 实时 + 游标回放 (瞬时原语按时刻过期) + 验收判决/机队指标/事件流面板; UE 编辑器截图 vs Web 截图 @t=5s 对比; 更新文档。

## Acceptance criteria
- [ ] 回放游标拖到任意时刻原语正确显隐
- [ ] 判决面板 PASS/FAIL 正常
- [ ] 截图对比位置 <=2cm 容差

## Blocked by
- 03-migrate-all-76-callsites.md
- 04-layer-toggle-overlay.md
- 06-task-management-python-web.md
