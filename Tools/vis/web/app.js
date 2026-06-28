import { log } from './logger.js';
import { initThree, rebuildScene, updateAgentPose, updateFollowCamera } from './scene.js';
import { initLayerUI } from './layer-ui.js';
import { initSimPanel } from './sim-panel.js';
import { initCharts, updateCharts } from './charts.js';
import { initCameraUI, rebuildCameraUI } from './camera-ui.js';
import { updateHudLabels, clearHudSprites, rebuildDebug } from './debug-renderer.js';

export const App = {
  data: null,
  scene: null, renderer: null, camera: null, controls: null,
  agentObjects: new Map(),
  debugGroups: new Map(),
  gridGroup: null,
  hudSprites: new Map(),
  hudSeries: new Map(),
  playing: true,
  cursorT: 0,
  durationSec: 0,
  es: null,
  finished: false,
  layerVisible: {},
  _lastReloadEpoch: undefined,
  _lastStructKey: undefined,
  tasks: [],
  currentTaskId: null,
};

if (typeof window !== 'undefined') window.__vis = { App };

// 必须在 loop() 首次调用前初始化，否则 ES 模块顶层求值时 lastTs 处于
// 暂时性死区(TDZ)，loop() 第一帧即抛 ReferenceError 导致模块求值中断、
// renderer.render 永不执行（画面空白）。见 app.js:26 loop() 调用处。
let lastTs = performance.now();

// 播放按钮是 App 状态的纯函数，每帧由 syncPlayButton() 重算——
// 任何代码改 App.playing 都不会导致“图标与实际状态不一致”。
// ▶=已暂停 ⏸=正在播放 ↻ 重新播放=回放已到末尾
const PLAY_BTN_TEXT_PLAY = '▶';
const PLAY_BTN_TEXT_PAUSE = '⏸';
const PLAY_BTN_TEXT_REPLAY = '↻';
let lastPlayBtnText = null;
let lastPlayBtnDisabled = null;

function isPlaybackEnded() {
  // 实时模式 data.finished=false，永不为 true，避免误判成“播放完毕”。
  return !!App.data && !!App.data.finished && App.cursorT >= App.durationSec - 1e-6;
}

function syncPlayButton() {
  const playBtn = document.getElementById('play-btn');
  if (!playBtn) return;
  const live = isLiveMode();
  const ended = isPlaybackEnded();
  // 实时流无法暂停：UE 仍在 Tick，放任点击会让 cursorT 冻结而 trace 持续增长，
  // sampleTrace(cursorT) 会让 3D 无人机定格在旧位姿，与仍在刷新的面板割裂。
  const disabled = live;
  const text = live ? PLAY_BTN_TEXT_PAUSE
    : ended ? PLAY_BTN_TEXT_REPLAY
    : (App.playing ? PLAY_BTN_TEXT_PAUSE : PLAY_BTN_TEXT_PLAY);
  const title = live ? '实时中'
    : ended ? '重新播放'
    : (App.playing ? '暂停' : '播放');
  if (text !== lastPlayBtnText) {
    playBtn.textContent = text;
    lastPlayBtnText = text;
  }
  if (disabled !== lastPlayBtnDisabled) {
    playBtn.disabled = disabled;
    lastPlayBtnDisabled = disabled;
  }
  playBtn.title = title;
}

function onPlayButtonClick() {
  if (isPlaybackEnded()) {
    App.cursorT = 0;
    App.playing = true;
  } else {
    App.playing = !App.playing;
  }
  syncPlayButton();
}

init();
loop();

async function init() {
  log.info('app', '初始化', { ts: new Date().toISOString() });
  initThree();
  initLayerUI();
  initCameraUI();
  initSimPanel();
  initCharts();
  initScrub();
  await refreshTasks();
  await refreshData();
  connectSSE();
}

export async function refreshData() {
  try {
    const url = App.currentTaskId ? `/api/task/${App.currentTaskId}` : '/api/data';
    const r = await fetch(url, { cache: 'no-store' });
    if (!r.ok && App.currentTaskId) {
      // 历史任务数据缺失(被删除/归档损坏) → 回退实时，避免把错误 JSON 当任务数据渲染
      log.warn('data', '任务数据不可用, 回退实时', { id: App.currentTaskId, status: r.status });
      App.currentTaskId = null;
      const sel = document.getElementById('task-select');
      if (sel) sel.value = '';
      const fallback = await fetch('/api/data', { cache: 'no-store' });
      App.data = await fallback.json();
    } else {
      App.data = await r.json();
    }
    onNewData();
  } catch (e) {
    log.error('data', 'fetch 失败', { error: e.message });
    setStatus('连接失败', false);
  }
}

async function refreshTasks() {
  try {
    const r = await fetch('/api/tasks');
    App.tasks = await r.json();
    renderTaskSelector();
  } catch (e) {
    log.warn('tasks', '任务列表获取失败', { error: e.message });
  }
}

function renderTaskSelector() {
  const sel = document.getElementById('task-select');
  if (!sel) return;
  const current = App.currentTaskId;
  sel.innerHTML = '<option value="">(当前实时任务)</option>';
  for (const t of App.tasks) {
    const opt = document.createElement('option');
    opt.value = t.id;
    // 原生 <select> 在 macOS 上会给选中项额外绘制一个 ✓ 选中指示符;
    // 若这里再用 ✓ 表示 PASS, 选中该项时会出现"两个勾", 故改用纯文本标签。
    const tag = t.verdict === 'PASS' ? 'PASS' : t.verdict === 'FAIL' ? 'FAIL' : '?';
    const time = t.id.split('_')[1] || t.id;
    opt.textContent = `[${tag}] ${time} · ${t.scenario || t.id}`;
    if (t.id === current) opt.selected = true;
    sel.appendChild(opt);
  }
}

function initScrub() {
  const scrub = document.getElementById('scrub');
  const playBtn = document.getElementById('play-btn');
  if (!scrub) return;
  scrub.addEventListener('input', () => {
    App.playing = false;
    App.cursorT = (scrub.value / 1000) * App.durationSec;
    syncPlayButton();
  });
  if (playBtn) {
    playBtn.addEventListener('click', onPlayButtonClick);
  }
  const taskSel = document.getElementById('task-select');
  if (taskSel) {
    taskSel.addEventListener('change', async () => {
      App.currentTaskId = taskSel.value || null;
      log.info('task', '切换任务', { id: App.currentTaskId });
      App.cursorT = 0;
      App.playing = true;
      if (App.currentTaskId) closeSSE();
      await refreshData();
      if (!App.currentTaskId && !App.finished) connectSSE();
    });
  }
}

function closeSSE() {
  if (!App.es) return;
  try { App.es.close(); } catch {}
  App.es = null;
}

export function connectSSE() {
  if (App.currentTaskId) return;
  closeSSE();
  try {
    App.es = new EventSource('/api/stream');
    log.info('sse', 'EventSource 连接中');
    App.es.onmessage = (ev) => {
      try {
        App.data = JSON.parse(ev.data);
        onNewData();
      } catch (e) {
        log.warn('sse', 'JSON parse failed', { error: e.message });
      }
    };
    App.es.addEventListener('done', () => {
      App.finished = true;
      setStatus('仿真结束 · 回放', false);
      closeSSE();
      refreshTasksAfterDone();
    });
    App.es.onerror = () => setStatus('连接中断,重试…', false);
  } catch (e) {
    log.error('sse', 'EventSource 构造失败', { error: e.message });
  }
}

// UE ArchiveCurrentTask 的文件复制与 SSE done 事件存在竞态:
// done 可能在归档完成前发出, 单次或固定延迟刷新都可能错过新任务。
// 退避重试直到新任务出现或耗尽尝试次数。
async function refreshTasksAfterDone() {
  const knownIds = new Set(App.tasks.map(t => t.id));
  const delays = [500, 1000, 1500, 2000, 3000, 3000, 3000];
  for (let i = 0; i < delays.length; i++) {
    await new Promise(r => setTimeout(r, delays[i]));
    await refreshTasks();
    const hasNew = App.tasks.some(t => !knownIds.has(t.id));
    if (hasNew) {
      log.info('tasks', '归档任务已出现', { attempts: i + 1 });
      return;
    }
  }
  log.warn('tasks', '归档任务未在重试窗口内出现', { attempts: delays.length });
}

function onNewData() {
  const d = App.data;
  if (!d) return;

  // 冷启动保护: 无 UE 进程背书的旧 telemetry 不渲染, 避免用户看到上一次仿真的残留画面
  if (isStaleDataWithoutSim()) {
    setStatus('等待启动仿真', false);
    updateScrubUI();
    syncPlayButton();
    return;
  }

  if (!d.agents) d.agents = [];
  if (!d.obstacles) d.obstacles = [];
  if (!d.perceivedObstacles) d.perceivedObstacles = [];
  if (!d.waypoints) d.waypoints = [];

  App.durationSec = Math.max((d.duration_ms || 0) / 1000, 0.001);
  if (d.reloadEpoch != null && d.reloadEpoch !== App._lastReloadEpoch) {
    const first = App._lastReloadEpoch === undefined;
    App._lastReloadEpoch = d.reloadEpoch;
    if (!first) {
      App.cursorT = 0;
      App.playing = true;
      App.finished = false;
    }
  }
  App.finished = !!d.finished;
  // 实时模式不在此吸附 cursorT=durationSec: SSE 离散推送会让无人机每帧跳跃,
  // 改由 loop() 平滑追逐 durationSec, sampleTrace 按插值产出连续位姿。

  setStatus(d.finished ? `已结束 · ${d.scenario || ''}` : `实时 · ${d.scenario || ''}`, !d.finished);
  rebuildHudSeries(d);

  // 结构指纹: 仅场景拓扑变化(spawn/障碍/航点/任务切换)才全量重建 Three.js 对象。
  // 实时 trace 增长不触发重建, agent 位姿由 updateAgentPose 每帧按 cursorT 插值反映,
  // 消除每帧销毁重建 ground/agent/obstacle 的主线程卡顿。
  const structKey = [
    App.currentTaskId ?? '', d.reloadEpoch ?? '', d.agents.length,
    (d.obstacles || []).length, (d.perceivedObstacles || []).length, (d.waypoints || []).length,
    d.agents.map(a => `${a.id}:${a.model}:${(a.initPos || []).map(x => x.toFixed(1)).join(',')}`).join('|'),
  ].join('#');

  if (structKey !== App._lastStructKey) {
    App._lastStructKey = structKey;
    clearHudSprites();
    rebuildScene(d);
  } else {
    rebuildDebug(d.debug || {});
  }

  renderVerdict(d);
  renderSummary(d);
  renderEvents(d);
  updateCharts(d);
  rebuildCameraUI();
  updateScrubUI();
}

function rebuildHudSeries(d) {
  App.hudSeries.clear();
  for (const a of (d.agents || [])) {
    if (Array.isArray(a.hudHistory) && a.hudHistory.length) {
      App.hudSeries.set(a.id, a.hudHistory);
    }
  }
}

function renderVerdict(d) {
  const el = document.getElementById('verdict');
  const det = document.getElementById('verdict-detail');
  const r = d.verdictResult || d.verdict;
  const final = d.verdictResult?.final || d.verdict?.final || 'UNKNOWN';
  if (el) {
    el.textContent = final === 'PASS' ? '✓ PASS' : (final === 'FAIL' ? '✗ FAIL' : '? 待定');
    el.className = 'verdict ' + final;
  }
  if (!r || !det) { if (det) det.innerHTML = '<span class="muted">等待数据…</span>'; return; }
  const rows = [];
  rows.push(['场景', d.scenario || '-']);
  rows.push(['判决', final]);
  if (r.reached != null) rows.push(['航点完成', r.reached]);
  rows.push(['最小净空', formatClearance(r.clearanceM)]);
  if (r.lateralDevM != null) rows.push(['最大横向偏差', `${r.lateralDevM} m`]);
  if (r.elapsedS != null) rows.push(['耗时', `${r.elapsedS} s`]);
  if (r.collided != null) rows.push(['碰撞', r.collided ? '是' : '否']);
  if (r.failures?.length) rows.push(['失败项', r.failures.join('; ')]);
  det.innerHTML = rows.map(([k, v]) => `<div class="row"><span class="k">${k}</span><span class="v">${v}</span></div>`).join('');
}

function formatClearance(value) {
  if (value == null) return 'N/A';
  const n = Number(value);
  if (!Number.isFinite(n) || n < 0) return 'N/A';
  if (n > 9999) return 'N/A';
  return `${n.toFixed(2)} m`;
}

function renderSummary(d) {
  const el = document.getElementById('summary-table');
  if (!el) return;
  const rows = d.summary;
  if (!rows?.length) { el.innerHTML = '<span class="muted">尚无汇总</span>'; return; }
  const cols = [
    ['Agent', s => s.agent],
    ['速度比', s => s.speedRatio],
    ['低速时长', s => `${s.lowSpeedDur}s`],
    ['最大偏差', s => s.maxDev > 100 ? `${(s.maxDev/100).toFixed(1)}m` : `${s.maxDev}cm`],
    ['Roll', s => s.maxRoll],
    ['Pitch', s => s.maxPitch],
    ['卡死', s => s.stuck ? '是' : '否'],
  ];
  let html = '<table><thead><tr>';
  cols.forEach(c => html += `<th>${c[0]}</th>`);
  html += '</tr></thead><tbody>';
  rows.forEach(s => {
    html += '<tr>';
    cols.forEach(c => { html += `<td>${c[1](s)}</td>`; });
    html += '</tr>';
  });
  html += '</tbody></table>';
  el.innerHTML = html;
}

function renderEvents(d) {
  const el = document.getElementById('events');
  if (!el) return;
  const evs = d.events || [];
  if (!evs.length) { el.innerHTML = '<span class="muted">无</span>'; return; }
  el.innerHTML = evs.map(e =>
    `<div class="ev"><span class="ev-t">${(e.t ?? 0).toFixed(1)}s</span>${e.type}: ${e.detail || ''}</div>`
  ).join('');
}

function updateScrubUI() {
  const scrub = document.getElementById('scrub');
  const label = document.getElementById('time-label');
  const badge = document.getElementById('live-badge');
  const live = isLiveMode();

  // 实时模式下 durationSec 是"已用时"而非"总时长"，scrubber 无百分比可言；
  // 且 cursorT 被 onNewData 持续同步到 durationSec，渲染会永远顶到 100% 满格。
  // durationSec 为 0(冷启动/无数据) 时 (cursorT/durationSec) = NaN, 浏览器会
  // 把 range 回退到 min/max 中点(500), 表现为"进度条默认在中间"。此时显式置 0。
  if (scrub) {
    scrub.disabled = live;
    if (!live) {
      const ratio = App.durationSec > 0 ? (App.cursorT / App.durationSec) : 0;
      scrub.value = String(ratio * 1000);
    }
  }
  if (badge) badge.hidden = !live;
  if (label) {
    label.textContent = live
      ? `已运行 ${App.durationSec.toFixed(1)}s`
      : `${App.cursorT.toFixed(1)}s`;
  }
}

// 实时态的权威定义: UE 进程在跑 + 未选历史任务 + 数据未结束。
// 三个条件全满足才算 LIVE; 任何一个不满足都按回放/空态处理。
// 这避免了 vis 重启后旧 telemetry 被误判为 LIVE 的 bug —— 旧 telemetry
// 没有 UE 进程背书, 不应展示为实时画面。
function isLiveMode() {
  const simRunning = !!App.data?.simStatus?.running;
  return simRunning && !App.currentTaskId && App.data != null && !App.data.finished;
}

// 冷启动保护: 数据来自旧 telemetry 文件但无 UE 进程背书 -> 视为脏数据, 不渲染。
// 用户看到空场景 + "等待启动仿真" 提示, 而不是上一次仿真的残留画面。
function isStaleDataWithoutSim() {
  if (!App.data) return false;
  if (App.currentTaskId) return false; // 历史任务回放是合法的
  const simRunning = !!App.data?.simStatus?.running;
  return !simRunning;
}

function setStatus(text, live) {
  const el = document.getElementById('status');
  if (!el) return;
  el.textContent = text;
  el.classList.toggle('live', !!live);
}
export { setStatus };

function loop() {
  requestAnimationFrame(loop);
  const now = performance.now();
  const dt = (now - lastTs) / 1000;
  lastTs = now;

  if (App.data && App.playing) {
    if (App.data.finished) {
      App.cursorT += dt;
      if (App.cursorT > App.durationSec) App.cursorT = App.durationSec;
    } else if (isLiveMode()) {
      // 实时: cursorT 平滑追逐 durationSec(SSE 离散推送的目标仿真时间),
      // sampleTrace 按 cursorT 在 trace 点间插值, 消除帧间跳跃让动画连续。
      const diff = App.durationSec - App.cursorT;
      App.cursorT = diff > 0.05 ? App.cursorT + diff * Math.min(1, dt * 10) : App.durationSec;
    }
  }

  if (App.controls) App.controls.update();
  updateAgentPose();
  updateFollowCamera();
  updateHudLabels();
  updateScrubUI();
  syncPlayButton();
  if (App.renderer && App.scene && App.camera) App.renderer.render(App.scene, App.camera);
}
