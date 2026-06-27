# Slice 3 — 全部 76 调用点迁移 + 18 层 taxonomy

- **状态**: ready-for-agent
- **Parent**: 00-prd.md

## What to build
把 6 个组件 (PlanningVisualizer/DebugVisualizer/ObstacleManager/PathPlanner/ObstacleDetector/StabilityScorer) 的全部 76 个 DrawDebug* 调用换成 UDebugDrawBuffer::Add*, 按 PRD 的 18 层 taxonomy 打 layer 标签。

## Acceptance criteria
- [ ] git grep "DrawDebug(Sphere|Line|Arrow|DirectionalArrow|Point|Box|String|Cylinder)" Source/.../{Planning,Debug,Sensors}/*.cpp 返回 0 (除 DebugDrawBuffer.cpp 代理)
- [ ] 跑仿真后 Web 显示全部 18 层

## Blocked by
- 02-all-six-primitive-types.md
