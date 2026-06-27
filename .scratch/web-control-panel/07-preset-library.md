# 预设库（保存 / 加载 / 导入导出）

- **状态**: ready-for-agent
- **创建**: 2026-06-27
- **来源**: PRD `.scratch/web-control-panel/00-prd.md` 切片 7

## Parent

PRD：`.scratch/web-control-panel/00-prd.md`。设计文档：`docs/design/web-control-panel.md`（§9 Python 后端、§10.4 预设库）。

## What to build

让用户能把当前一套 Web 端配置存为命名预设、加载、以及导入/导出，避免每次重填。预设以 JSON 落在 Python 侧目录（不生成 `.uasset`，避免版本库与烹饪复杂度）。

端到端行为：用户填好一套配置 → 存为命名预设 → 下次打开从预设列表一键加载填入表单 → 或导出 JSON 文件分享 / 导入他人 JSON 加载。

实现：
- Python 端新增 `/api/presets`：GET 列举、GET `{name}` 读取、POST `{name}` 保存、DELETE `{name}` 删除；预设文件落在 `Tools/vis/presets/*.json`。
- 前端预设库 UI：预设列表（加载/删除）、"另存为预设"、导出 JSON 文件、导入 JSON 文件。
- 预设内容即完整的 `ScenarioDto`（切片 1~4 定义），与重跑/调参共享同一 DTO 契约。

## Acceptance criteria

- [ ] 用户能把当前表单配置另存为命名预设，预设以 JSON 持久化到 `Tools/vis/presets/`。
- [ ] 重启 Python/前端后，预设列表仍可加载（持久化生效）。
- [ ] 用户加载某预设后，所有配置 Tab 字段被正确回填（机队/障碍/航线/风场·验收/仿真）。
- [ ] 用户能导出当前配置为 JSON 文件、能导入 JSON 文件回填表单（跨机器/同事分享）。
- [ ] 预设与重跑共享同一 DTO 契约（`ScenarioDto`），加载的预设可直接点重跑触发热重载。
- [ ] 删除预设后从列表消失且文件被移除。
- [ ] Python `/api/presets` 有 pytest 单测覆盖列举/读取/保存/删除与非法预设名防护（mock 文件系统或用临时目录）。

## Blocked by

- `01-control-channel-and-hot-reload.md`（DTO 契约与反向通道基座）
- `06-frontend-control-panel.md`（前端表单与配置 Tab，预设需回填这些字段）
