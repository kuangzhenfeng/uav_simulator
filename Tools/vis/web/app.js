import { log } from './logger.js';
import { initThree, rebuildScene, updateAgentPose, updateFollowCamera } from './scene.js';
import { initLayerUI } from './layer-ui.js';
import { initSimPanel } from './sim-panel.js';
import { initCharts, updateCharts } from './charts.js';
import { initCameraUI, rebuildCameraUI } from './camera-ui.js';

export const App = {
  data: null,
  scene: null, renderer: null, camera: null, controls: null,
  agentObjects: new Map(),
  debugGroups: new Map(),
  gridGroup: null,
  playing: true,
  cursorT: 0,
  durationSec: 0,
  es: null,
  finished: false,
  layerVisible: {},
  _lastReloadEpoch: undefined,
  tasks: [],
  currentTaskId: null,
};

if (typeof window !== 'undefined') window.__vis = { App };

// 必须在 loop() 首次调用前初始化，否则 ES 模块顶层求值时 lastTs 处于
// 暂时性死区(TDZ)，loop() 第一帧即抛 ReferenceError 导致模块求值中断、
// renderer.render 永不执行（画面空白）。见 app.js:26 loop() 调用处。
let lastTs = performance.now();

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
    App.data = await r.json();
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
    const verdict = t.verdict === 'PASS' ? '✓' : t.verdict === 'FAIL' ? '✗' : '?';
    opt.textContent = `${verdict} ${t.id} · ${t.scenario || ''}`;
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
    if (playBtn) playBtn.textContent = '▶';
  });
  if (playBtn) {
    playBtn.addEventListener('click', () => {
      App.playing = !App.playing;
      playBtn.textContent = App.playing ? '⏸' : '▶';
    });
  }
  const taskSel = document.getElementById('task-select');
  if (taskSel) {
    taskSel.addEventListener('change', async () => {
      App.currentTaskId = taskSel.value || null;
      log.info('task', '切换任务', { id: App.currentTaskId });
      App.cursorT = 0;
      App.playing = true;
      await refreshData();
    });
  }
}

function connectSSE() {
  if (App.currentTaskId) return;
  if (App.es) { try { App.es.close(); } catch {} App.es = null; }
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
      refreshTasks();
      try { App.es.close(); } catch {}
      App.es = null;
      setTimeout(() => connectSSE(), 3000);
    });
    App.es.onerror = () => setStatus('连接中断,重试…', false);
  } catch (e) {
    log.error('sse', 'EventSource 构造失败', { error: e.message });
  }
}

function onNewData() {
  const d = App.data;
  if (!d) return;
  if (!d.agents) d.agents = [];
  if (!d.obstacles) d.obstacles = [];
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
  if (!d.finished && App.playing) App.cursorT = App.durationSec;

  setStatus(d.finished ? `已结束 · ${d.scenario || ''}` : `实时 · ${d.scenario || ''}`, !d.finished);
  rebuildScene(d);
  renderVerdict(d);
  renderSummary(d);
  renderEvents(d);
  updateCharts(d);
  rebuildCameraUI();
  updateScrubUI();
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
  if (r.clearanceM != null) rows.push(['最小净空', `${r.clearanceM} m`]);
  if (r.lateralDevM != null) rows.push(['最大横向偏差', `${r.lateralDevM} m`]);
  if (r.elapsedS != null) rows.push(['耗时', `${r.elapsedS} s`]);
  if (r.collided != null) rows.push(['碰撞', r.collided ? '是' : '否']);
  if (r.failures?.length) rows.push(['失败项', r.failures.join('; ')]);
  det.innerHTML = rows.map(([k, v]) => `<div class="row"><span class="k">${k}</span><span class="v">${v}</span></div>`).join('');
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
  if (scrub) scrub.value = (App.cursorT / App.durationSec) * 1000;
  if (label) label.textContent = `${App.cursorT.toFixed(1)}s`;
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

  if (App.data && App.data.finished && App.playing) {
    App.cursorT += dt;
    if (App.cursorT > App.durationSec) App.cursorT = App.durationSec;
  }

  if (App.controls) App.controls.update();
  updateAgentPose();
  updateFollowCamera();
  updateScrubUI();
  if (App.renderer && App.scene && App.camera) App.renderer.render(App.scene, App.camera);
}
