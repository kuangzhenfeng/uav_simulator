# Slice 2 — 全 6 种原语类型扩展

- **状态**: ready-for-agent
- **Parent**: 00-prd.md

## What to build
在 Slice 1 (仅 Sphere) 的端到端骨架上, 补全剩余 5 种原语 (Line/Arrow/Box/Point/Text) 的完整链路: UE buffer 的 Add* 方法 + Python _convert_debug_prim 的全 6 种分支 + Web debug-renderer.js 的全 6 种渲染器。用合成 ndjson TDD 验证。

## Acceptance criteria
- [ ] pytest: 喂含 6 种原语各一例的 debug 行, snapshot 全部正确 to_web 变换
- [ ] Web 渲染 Line/Arrow/Box/Point/Text 各类型, 位置/颜色/尺寸匹配
- [ ] Text 原语用 THREE.Sprite + CanvasTexture 渲染, 可读不糊

## Blocked by
- 01-sphere-tracer-bullet.md
