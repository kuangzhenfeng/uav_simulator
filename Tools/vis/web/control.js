// Web 控制面板前端(原生 ES Module, 零构建)
// 职责:配置 Tab(机队/障碍/风验收/仿真) + 实时调参 Tab + 预设 Tab。
// 只与可视化端口(:8765)通信;控制端不可用时面板置灰。
// 坐标约定:表单用 Web 展示单位(米, 右手系 Y-up),提交时经 toUE()/toWebVec() 变换为
// UE cm 左手系 DTO(与 server.py to_web 互逆)。

// ---- 坐标变换(与 server.py to_web 互逆) ----
// UE(左手 X前/Y右/Z上, cm) <-> Web(右手 Y-up, m)
// web->ue: x_ue=x_web*100, y_ue=-z_web*100, z_ue=y_web*100
function vecToUE(web) {
  if (!Array.isArray(web) || web.length < 3) return [0, 0, 0];
  return [web[0] * 100, -web[2] * 100, web[1] * 100];
}
function vecToWeb(ue) {
  if (!Array.isArray(ue) || ue.length < 3) return [0, 0, 0];
  return [ue[0] / 100, ue[2] / 100, -ue[1] / 100];
}

// ---- 全局状态 ----
const CP = {
  schema: null,        // /api/schema 结果
  available: false,    // 控制端是否可用
  dto: null,           // 当前编辑中的 ScenarioDto(从 schema.defaults 初始化)
  tuneHistory: [],     // 实时调参历史 [{t,target,key,old,new}]
  activePickTarget: null, // 3D 拾取当前目标 {kind:'agent'|'obstacle'|'waypoint', idx, field}
  reloadEpochSeen: 0,
};

// 暴露给 app.js 做 3D 拾取注入
if (typeof window !== 'undefined') window.__cp = CP;

init();

async function init() {
  bindTabs();
  bindPanelToggle();
  bindPickToggle();
  await loadSchema();
  await refreshAvail();
  if (CP.dto) renderConfigTab();
  renderTuneTab();
  renderPresetTab();
  // 周期刷新控制端可用性 + reload epoch
  setInterval(refreshAvail, 3000);
}

// 底部"3D拾取"勾选框:手动开关拾取模式(无目标时点击会提示先选字段)
function bindPickToggle() {
  const cb = document.getElementById('toggle-pick');
  if (!cb) return;
  cb.addEventListener('change', () => {
    if (window.__vis) window.__vis.pickMode = cb.checked;
    if (!cb.checked) {
      CP.activePickTarget = null;
      document.querySelectorAll('.cp-pick-active').forEach(b => b.classList.remove('cp-pick-active'));
    }
  });
}

// ---------------------------------------------------------------------------
// 顶部抽屉开关 + Tab 切换
// ---------------------------------------------------------------------------
function bindPanelToggle() {
  document.getElementById('toggle-panel-btn').addEventListener('click', () => {
    const p = document.getElementById('control-panel');
    p.classList.toggle('closed');
  });
}

function bindTabs() {
  document.querySelectorAll('.cp-tab').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.cp-tab').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const tab = btn.dataset.cpTab;
      document.querySelectorAll('.cp-section').forEach(s => s.classList.remove('active'));
      document.getElementById('cp-' + tab).classList.add('active');
    });
  });
}

// ---------------------------------------------------------------------------
// schema + 控制端可用性
// ---------------------------------------------------------------------------
async function loadSchema() {
  try {
    const r = await fetch('/api/schema', { cache: 'no-store' });
    const s = await r.json();
    CP.schema = s;
    CP.dto = deepClone(s.defaults);
  } catch (e) {
    console.error('[control] schema 加载失败', e);
  }
}

function deepClone(o) { return JSON.parse(JSON.stringify(o)); }

async function refreshAvail() {
  try {
    const r = await fetch('/api/control/status', { cache: 'no-store' });
    const j = await r.json();
    CP.available = !!j.controlAvailable;
  } catch (e) {
    CP.available = false;
  }
  const badge = document.getElementById('cp-avail-badge');
  if (badge) {
    badge.textContent = CP.available ? '● 控制端在线' : '○ 控制端离线';
    badge.className = 'cp-badge ' + (CP.available ? 'online' : 'offline');
  }
  // 面板内提交类按钮根据可用性置灰
  document.querySelectorAll('.cp-submit').forEach(b => { b.disabled = !CP.available; });
}

// ---------------------------------------------------------------------------
// 通用 DOM 辅助
// ---------------------------------------------------------------------------
function el(tag, cls, html) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (html != null) e.innerHTML = html;
  return e;
}

// 监听 SSE reload epoch(由 app.js 推送 /api/data 里的 reloadEpoch 字段)
// 这里用一个轮询:每秒拉 /api/data 检查 reloadEpoch 变化,触发回放重置提示。
setInterval(async () => {
  try {
    const r = await fetch('/api/data', { cache: 'no-store' });
    const d = await r.json();
    const epoch = d.reloadEpoch || 0;
    if (epoch !== CP.reloadEpochSeen && CP.reloadEpochSeen !== 0) {
      toast('场景已热重载,回放已重置');
    }
    CP.reloadEpochSeen = epoch;
  } catch (e) {}
}, 1500);

function toast(msg) {
  let t = document.getElementById('cp-toast');
  if (!t) {
    t = el('div', 'cp-toast', '');
    t.id = 'cp-toast';
    document.body.appendChild(t);
  }
  t.textContent = msg;
  t.classList.add('show');
  clearTimeout(t._h);
  t._h = setTimeout(() => t.classList.remove('show'), 2500);
}

// 提交 reload/slomo/wind/params 的统一封装
async function postControl(subPath, body) {
  if (!CP.available) { toast('控制端离线,无法提交'); return null; }
  try {
    const r = await fetch('/api/control/' + subPath, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    return await r.json();
  } catch (e) {
    toast('提交失败: ' + e.message);
    return null;
  }
}

// 把 3D 拾取到的 web 坐标注入当前活动表单字段
window.__cpPick = function (webPos) {
  if (!CP.activePickTarget) { toast('请先在表单点"拾取"按钮选择目标字段'); return; }
  const t = CP.activePickTarget;
  // 找到对应输入框组(由 render 时写入 data-pick 属性)
  const inputs = document.querySelectorAll(`[data-pick="${t.kind}:${t.idx}:${t.field}"]`);
  if (inputs.length < 3) { CP.activePickTarget = null; return; }
  inputs[0].value = round3(webPos[0]);
  inputs[1].value = round3(webPos[1]);
  inputs[2].value = round3(webPos[2]);
  toast('已拾取坐标');
  CP.activePickTarget = null;
  // 清除拾取高亮
  document.querySelectorAll('.cp-pick-active').forEach(b => b.classList.remove('cp-pick-active'));
  const cb = document.getElementById('toggle-pick');
  if (cb) cb.checked = false;
  if (window.__vis) window.__vis.pickMode = false;
};
function round3(v) { return (Math.round(v * 1000) / 1000).toString(); }

// 由配置区"拾取"按钮调用,设定拾取目标并开启拾取模式
function armPick(kind, idx, field, btn) {
  CP.activePickTarget = { kind, idx, field };
  document.querySelectorAll('.cp-pick-active').forEach(b => b.classList.remove('cp-pick-active'));
  if (btn) btn.classList.add('cp-pick-active');
  const cb = document.getElementById('toggle-pick');
  if (cb) cb.checked = true;
  if (window.__vis) window.__vis.pickMode = true;
  toast('已开启拾取,在 3D 视图点击地面/点');
}

// ---------------------------------------------------------------------------
// 配置 Tab:机队 / 障碍 / 风·验收 / 仿真
// ---------------------------------------------------------------------------
function renderConfigTab() {
  const root = document.getElementById('cp-config');
  root.innerHTML = '';

  // 顶部:场景名 + 重跑按钮
  const head = el('div', 'cp-field-row');
  head.appendChild(mkInput('text', '场景名', CP.dto.name, v => CP.dto.name = v));
  const runBtn = el('button', 'cp-submit cp-run', '▶ 重跑(热重载)');
  runBtn.addEventListener('click', onReload);
  head.appendChild(runBtn);
  root.appendChild(head);

  root.appendChild(mkSection('机队', renderFleet()));
  root.appendChild(mkSection('障碍', renderObstacles()));
  root.appendChild(mkSection('风场 · 验收', renderWindAcceptance()));
  root.appendChild(mkSection('仿真控制', renderSim()));
}

function mkSection(title, contentNode) {
  const sec = el('div', 'cp-card');
  sec.appendChild(el('div', 'cp-card-title', title));
  sec.appendChild(contentNode);
  return sec;
}

function mkInput(type, label, val, onChange, opts = {}) {
  const wrap = el('div', 'cp-input');
  const lab = el('label', 'cp-label', label);
  const inp = document.createElement('input');
  inp.type = type;
  inp.value = val;
  inp.addEventListener('input', () => onChange(type === 'number' ? parseFloat(inp.value) : inp.value));
  if (opts.step) inp.step = opts.step;
  if (opts.placeholder) inp.placeholder = opts.placeholder;
  wrap.appendChild(lab);
  wrap.appendChild(inp);
  return wrap;
}

function mkSelect(label, options, val, onChange) {
  const wrap = el('div', 'cp-input');
  wrap.appendChild(el('label', 'cp-label', label));
  const sel = document.createElement('select');
  options.forEach(o => {
    const opt = document.createElement('option');
    opt.value = o.id || o;
    opt.textContent = o.label || o;
    if ((o.id || o) === val) opt.selected = true;
    sel.appendChild(opt);
  });
  sel.addEventListener('change', () => onChange(sel.value));
  wrap.appendChild(sel);
  return wrap;
}

// ---- 机队 ----
function renderFleet() {
  const wrap = el('div', '');
  CP.dto.fleet.forEach((agent, i) => {
    wrap.appendChild(renderAgent(agent, i));
  });
  const addBtn = el('button', 'cp-add-btn', '+ 增加无人机');
  addBtn.addEventListener('click', () => {
    const def = deepClone(CP.schema.defaults.fleet[0]);
    def.isLeader = (CP.dto.fleet.length === 0);
    def.initPos = [0, 0, 1];
    CP.dto.fleet.push(def);
    renderConfigTab();
  });
  wrap.appendChild(addBtn);
  return wrap;
}

function renderAgent(agent, i) {
  const card = el('div', 'cp-subcard');
  const head = el('div', 'cp-subcard-head');
  head.appendChild(el('span', '', `无人机 #${i + 1}`));
  if (i > 0) {
    const del = el('button', 'cp-icon-btn', '✕ 删除');
    del.addEventListener('click', () => {
      CP.dto.fleet.splice(i, 1);
      renderConfigTab();
    });
    head.appendChild(del);
  }
  card.appendChild(head);

  const row1 = el('div', 'cp-field-row');
  row1.appendChild(mkSelect('机型', CP.schema.models, agent.model, v => agent.model = v));
  const modeWrap = mkSelect('任务模式', CP.schema.missionModes, agent.mode, v => agent.mode = v);
  row1.appendChild(modeWrap);
  row1.appendChild(mkCheckbox('长机', agent.isLeader, v => {
    agent.isLeader = v;
    if (v) CP.dto.fleet.forEach((a, j) => { if (j !== i) a.isLeader = false; });
    renderConfigTab();
  }));
  card.appendChild(row1);

  // 初始位置(web m, 三输入 + 拾取)
  const posRow = el('div', 'cp-vec-row');
  posRow.appendChild(el('label', 'cp-label', '初始位置 (m, x/y/z)'));
  const posInputs = mkVec3Inputs(agent.initPos, v => agent.initPos = v);
  posRow.appendChild(posInputs.node);
  const pickBtn = el('button', 'cp-pick-btn', '◎拾取');
  pickBtn.addEventListener('click', () => armPick('agent', i, 'initPos', pickBtn));
  // 给三个输入框标记 data-pick,供 __cpPick 定位
  posInputs.inputs.forEach((inp, k) => inp.setAttribute('data-pick', `agent:${i}:initPos`));
  posRow.appendChild(pickBtn);
  card.appendChild(posRow);

  // 朝向
  card.appendChild(mkInput('number', '初始朝向 (°)', agent.yaw, v => agent.yaw = v));

  // 航点序列(可折叠)
  const wpWrap = el('div', 'cp-waypoints');
  const wpToggle = el('button', 'cp-collapse-btn', `▾ 航点 (${agent.waypoints.length})`);
  const wpList = el('div', 'cp-wp-list cp-collapsed');
  wpToggle.addEventListener('click', () => {
    wpList.classList.toggle('cp-collapsed');
    wpToggle.textContent = wpList.classList.contains('cp-collapsed')
      ? `▾ 航点 (${agent.waypoints.length})` : `▴ 航点 (${agent.waypoints.length})`;
  });
  agent.waypoints.forEach((wp, wi) => {
    const wRow = el('div', 'cp-vec-row');
    wRow.appendChild(el('label', 'cp-label', `航点 ${wi + 1} (m, x/y/z)`));
    const wiNode = mkVec3Inputs(wp.pos, v => wp.pos = v);
    wRow.appendChild(wiNode.node);
    const wpk = el('button', 'cp-pick-btn', '◎');
    wpk.addEventListener('click', () => armPick('waypoint', `${i}:${wi}`, 'pos', wpk));
    wiNode.inputs.forEach(inp => inp.setAttribute('data-pick', `waypoint:${i}:${wi}:pos`));
    // 这里 data-pick 用 kind:idx:field,waypoint idx 用 "i:wi" 复合
    // 但 __cpPick 按 kind:idx:field 查找,需对齐;统一改用 waypoint 字符串
    wiNode.inputs.forEach(inp => inp.setAttribute('data-pick', `waypoint:${i}:${wi}:pos`));
    wRow.appendChild(wpk);
    const del = el('button', 'cp-icon-btn', '✕');
    del.addEventListener('click', () => { agent.waypoints.splice(wi, 1); renderConfigTab(); });
    wRow.appendChild(del);
    wpList.appendChild(wRow);
  });
  const addWp = el('button', 'cp-add-btn', '+ 增加航点');
  addWp.addEventListener('click', () => {
    agent.waypoints.push({ pos: [round3((agent.initPos[0]) / 100), 1, round3(agent.initPos[2] / 100)], speed: 0, hover: 0 });
    // 航点存 web m;initPos 也是 web m,直接复用
    agent.waypoints[agent.waypoints.length - 1].pos = [agent.initPos[0], 1, agent.initPos[2]];
    renderConfigTab();
  });
  wpList.appendChild(addWp);
  wpWrap.appendChild(wpToggle);
  wpWrap.appendChild(wpList);
  card.appendChild(wpWrap);

  return card;
}

function mkVec3Inputs(webVec, onChange) {
  const node = el('div', 'cp-vec-inputs');
  const inputs = [];
  ['x', 'y', 'z'].forEach((axis, k) => {
    const inp = document.createElement('input');
    inp.type = 'number';
    inp.step = '0.1';
    inp.value = round3(webVec[k] || 0);
    inp.addEventListener('input', () => {
      const v = [parseFloat(inputs[0].value) || 0, parseFloat(inputs[1].value) || 0, parseFloat(inputs[2].value) || 0];
      onChange(v);
    });
    inputs.push(inp);
    node.appendChild(inp);
  });
  return { node, inputs };
}

function mkCheckbox(label, checked, onChange) {
  const wrap = el('label', 'cp-input cp-check');
  const inp = document.createElement('input');
  inp.type = 'checkbox';
  inp.checked = !!checked;
  inp.addEventListener('change', () => onChange(inp.checked));
  wrap.appendChild(inp);
  wrap.appendChild(el('span', 'cp-label-inline', label));
  return wrap;
}

// ---- 障碍 ----
function renderObstacles() {
  const wrap = el('div', '');
  CP.dto.obstacles.forEach((obs, i) => {
    wrap.appendChild(renderObstacle(obs, i));
  });
  const addBtn = el('button', 'cp-add-btn', '+ 增加静态障碍');
  addBtn.addEventListener('click', () => {
    CP.dto.obstacles.push({ type: 'Box', center: [0, 0, 1], extents: [1, 1, 1], movement: 'Static', safetyMargin: 0.5 });
    renderConfigTab();
  });
  wrap.appendChild(addBtn);
  const addDyn = el('button', 'cp-add-btn', '+ 增加动态障碍(巡逻)');
  addDyn.addEventListener('click', () => {
    CP.dto.obstacles.push({ type: 'Box', center: [10, 0, 1], extents: [0.8, 0.8, 0.8], movement: 'PatrolLoop', patrolPoints: [[10, 0, 1], [10, 5, 1]], patrolSpeed: 3, safetyMargin: 0.3 });
    renderConfigTab();
  });
  wrap.appendChild(addDyn);
  return wrap;
}

function renderObstacle(obs, i) {
  const card = el('div', 'cp-subcard');
  const head = el('div', 'cp-subcard-head');
  head.appendChild(el('span', '', `障碍 #${i + 1} (${obs.movement})`));
  const del = el('button', 'cp-icon-btn', '✕ 删除');
  del.addEventListener('click', () => { CP.dto.obstacles.splice(i, 1); renderConfigTab(); });
  head.appendChild(del);
  card.appendChild(head);

  const row1 = el('div', 'cp-field-row');
  row1.appendChild(mkSelect('类型', CP.schema.obstacleTypes, obs.type, v => obs.type = v));
  row1.appendChild(mkSelect('运动', CP.schema.movements, obs.movement, v => { obs.movement = v; renderConfigTab(); }));
  card.appendChild(row1);

  const cRow = el('div', 'cp-vec-row');
  cRow.appendChild(el('label', 'cp-label', '中心 (m, x/y/z)'));
  const cNode = mkVec3Inputs(obs.center, v => obs.center = v);
  cRow.appendChild(cNode.node);
  const cpk = el('button', 'cp-pick-btn', '◎拾取');
  cpk.addEventListener('click', () => armPick('obstacle', i, 'center', cpk));
  cNode.inputs.forEach(inp => inp.setAttribute('data-pick', `obstacle:${i}:center`));
  cRow.appendChild(cpk);
  card.appendChild(cRow);

  const eRow = el('div', 'cp-vec-row');
  eRow.appendChild(el('label', 'cp-label', '半尺寸 (m, x/y/z)'));
  eRow.appendChild(mkVec3Inputs(obs.extents, v => obs.extents = v).node);
  card.appendChild(eRow);

  card.appendChild(mkInput('number', '安全边距 (m)', obs.safetyMargin, v => obs.safetyMargin = v));

  // 动态障碍:巡逻点/速度
  if (obs.movement !== 'Static') {
    card.appendChild(mkInput('number', '巡逻速度 (m/s)', obs.patrolSpeed || 3, v => obs.patrolSpeed = v));
    if (!obs.patrolPoints) obs.patrolPoints = [];
    const ppWrap = el('div', '');
    ppWrap.appendChild(el('div', 'cp-label', '巡逻点 (m)'));
    obs.patrolPoints.forEach((pp, pi) => {
      const ppRow = el('div', 'cp-vec-row');
      ppRow.appendChild(el('label', 'cp-label', `巡逻点 ${pi + 1}`));
      const ppNode = mkVec3Inputs(pp, v => obs.patrolPoints[pi] = v);
      ppRow.appendChild(ppNode.node);
      const ppk = el('button', 'cp-pick-btn', '◎');
      ppk.addEventListener('click', () => armPick('patrol', `${i}:${pi}`, 'pos', ppk));
      ppNode.inputs.forEach(inp => inp.setAttribute('data-pick', `patrol:${i}:${pi}:pos`));
      ppRow.appendChild(ppk);
      const pdel = el('button', 'cp-icon-btn', '✕');
      pdel.addEventListener('click', () => { obs.patrolPoints.splice(pi, 1); renderConfigTab(); });
      ppRow.appendChild(pdel);
      ppWrap.appendChild(ppRow);
    });
    const addPp = el('button', 'cp-add-btn', '+ 增加巡逻点');
    addPp.addEventListener('click', () => { obs.patrolPoints.push([0, 0, 1]); renderConfigTab(); });
    ppWrap.appendChild(addPp);
    card.appendChild(ppWrap);
  }
  return card;
}

// ---- 风场 · 验收 ----
function renderWindAcceptance() {
  const wrap = el('div', '');
  const w = CP.dto.wind;
  const rowW = el('div', 'cp-field-row');
  rowW.appendChild(mkSelect('风类型', CP.schema.windTypes, w.type, v => w.type = v));
  rowW.appendChild(mkCheckbox('启用风场', w.enabled, v => w.enabled = v));
  wrap.appendChild(rowW);

  const steadyRow = el('div', 'cp-vec-row');
  steadyRow.appendChild(el('label', 'cp-label', '稳态风 (m/s, x/y/z)'));
  steadyRow.appendChild(mkVec3Inputs(w.steady, v => w.steady = v).node);
  wrap.appendChild(steadyRow);
  wrap.appendChild(mkInput('number', '阵风幅度', w.gustAmplitude, v => w.gustAmplitude = v));
  wrap.appendChild(mkInput('number', '湍流强度', w.turbulenceIntensity, v => w.turbulenceIntensity = v));

  const a = CP.dto.acceptance;
  const aRow = el('div', 'cp-field-row');
  aRow.appendChild(mkCheckbox('要求全部航点', a.requireAllWaypoints, v => a.requireAllWaypoints = v));
  aRow.appendChild(mkInput('number', '航点半径(m)', a.waypointRadiusCm / 100, v => a.waypointRadiusCm = v * 100));
  wrap.appendChild(aRow);
  wrap.appendChild(mkInput('number', '最小净空(m)', a.minClearanceCm / 100, v => a.minClearanceCm = v * 100));
  wrap.appendChild(mkInput('number', '最大横向偏差(m)', a.maxLateralDeviationCm / 100, v => a.maxLateralDeviationCm = v * 100));
  wrap.appendChild(mkInput('number', '超时(s)', a.timeoutSec, v => a.timeoutSec = v));
  return wrap;
}

// ---- 仿真 ----
function renderSim() {
  const wrap = el('div', '');
  const s = CP.dto.sim;
  wrap.appendChild(mkInput('number', '时标倍速(slomo)', s.slomo, v => s.slomo = v));
  wrap.appendChild(mkInput('number', '仿真时长(s)', s.durationSec, v => s.durationSec = v));
  wrap.appendChild(mkInput('number', '随机种子', CP.dto.randomSeed || 0, v => CP.dto.randomSeed = v));
  const cm = mkSelect('控制模式(留空=默认)', [''].concat(CP.schema.controlModes), s.controlMode, v => s.controlMode = v);
  wrap.appendChild(cm);
  const mt = mkSelect('MPC 类型(留空=默认)', [''].concat(CP.schema.mpcTypes), s.mpcType, v => s.mpcType = v);
  wrap.appendChild(mt);
  return wrap;
}

// ---------------------------------------------------------------------------
// 重跑:收集 DTO(web m) -> UE cm 变换 -> POST
// ---------------------------------------------------------------------------
async function onReload() {
  const dto = collectDto();
  const btn = document.querySelector('.cp-run');
  const orig = btn.textContent;
  btn.disabled = true; btn.textContent = '提交中…';
  const res = await postControl('reload', dto);
  btn.disabled = false; btn.textContent = orig;
  if (res && res.ok) {
    toast(`重跑成功: ${res.fleetCount} 架无人机`);
    CP.reloadEpochSeen = 0; // 重置,等下一次 epoch 变化触发提示
  } else {
    toast('重跑失败: ' + (res && res.error ? res.error : '未知'));
  }
}

function collectDto() {
  const d = deepClone(CP.dto);
  // 坐标:web m -> UE cm(逆变换)
  d.fleet.forEach(a => {
    a.initPos = vecToUE(a.initPos);
    a.waypoints.forEach(wp => { wp.pos = vecToUE(wp.pos); });
  });
  d.obstacles.forEach(o => {
    o.center = vecToUE(o.center);
    o.extents = o.extents.map(v => Math.abs(v) * 100); // 半尺寸:仅单位换算
    if (o.patrolPoints) o.patrolPoints = o.patrolPoints.map(p => vecToUE(p));
  });
  d.wind.steady = vecToUE(d.wind.steady);
  // 验收:前端按 m 编辑,DTO 用 cm
  d.acceptance.waypointRadiusCm = d.acceptance.waypointRadiusCm;
  d.acceptance.minClearanceCm = d.acceptance.minClearanceCm;
  d.acceptance.maxLateralDeviationCm = d.acceptance.maxLateralDeviationCm;
  return d;
}

// ---------------------------------------------------------------------------
// 实时调参 Tab
// ---------------------------------------------------------------------------
function renderTuneTab() {
  const root = document.getElementById('cp-tune');
  root.innerHTML = '';
  root.appendChild(mkSelect('作用范围', [
    { id: 'fleet', label: '全队' }, { id: 'leader', label: '仅长机' }, { id: '0', label: '仅 Agent 0' },
  ], 'fleet', v => CP._tuneTarget = v));

  root.appendChild(el('div', 'cp-card-title', '姿态 PID (Roll/Pitch/Yaw)'));
  ['Roll', 'Pitch', 'Yaw'].forEach(axis => {
    const card = el('div', 'cp-subcard');
    card.appendChild(el('div', 'cp-label', axis));
    const r = el('div', 'cp-field-row');
    const kp = mkSlider('Kp', 0, 0.2, 0.001, 0);
    const ki = mkSlider('Ki', 0, 0.1, 0.001, 0);
    const kd = mkSlider('Kd', 0, 0.3, 0.001, 0);
    const apply = el('button', 'cp-submit cp-small', '应用');
    apply.addEventListener('click', () => {
      const body = { target: CP._tuneTarget || 'fleet' };
      body['attitude' + axis] = { kp: kp.val(), ki: ki.val(), kd: kd.val() };
      applyTune(body, `${axis} PID`);
    });
    r.appendChild(kp.node); r.appendChild(ki.node); r.appendChild(kd.node); r.appendChild(apply);
    card.appendChild(r);
    root.appendChild(card);
  });

  root.appendChild(el('div', 'cp-card-title', 'CBF-QP'));
  const cbfCard = el('div', 'cp-subcard');
  const dsafe = mkSlider('dSafe (m)', 0, 10, 0.1, 2);
  const alpha0 = mkSlider('alpha0', 0, 5, 0.05, 1);
  const cbfApply = el('button', 'cp-submit cp-small', '应用');
  cbfApply.addEventListener('click', () => applyTune({
    target: CP._tuneTarget || 'fleet',
    cbfqp: { dSafe: dsafe.val(), alpha0: alpha0.val() },
  }, 'CBF-QP'));
  cbfCard.appendChild(dsafe.node); cbfCard.appendChild(alpha0.node); cbfCard.appendChild(cbfApply);
  root.appendChild(cbfCard);

  root.appendChild(el('div', 'cp-card-title', '环境 · 时标'));
  const envCard = el('div', 'cp-subcard');
  const slomo = mkSlider('slomo', 0.1, 16, 0.1, 8);
  const windEnable = mkCheckbox('启用风场', true, () => {});
  const windApply = el('button', 'cp-submit cp-small', '应用');
  windApply.addEventListener('click', () => {
    postControl('slomo', { scale: slomo.val() }).then(r => r && r.ok && toast('slomo=' + slomo.val()));
    postControl('wind', { enabled: windEnable.querySelector('input').checked });
  });
  envCard.appendChild(slomo.node); envCard.appendChild(windEnable); envCard.appendChild(windApply);
  root.appendChild(envCard);

  // 调参历史
  root.appendChild(el('div', 'cp-card-title', '调参历史(可回滚)'));
  const hist = el('div', 'cp-history', '<span class="muted">暂无</span>');
  hist.id = 'cp-tune-history';
  root.appendChild(hist);
}

function mkSlider(label, min, max, step, initVal) {
  const wrap = el('div', 'cp-input cp-slider');
  wrap.appendChild(el('label', 'cp-label', `${label}: <span class="cp-slider-val">${initVal}</span>`));
  const inp = document.createElement('input');
  inp.type = 'range'; inp.min = min; inp.max = max; inp.step = step; inp.value = initVal;
  inp.addEventListener('input', () => {
    wrap.querySelector('.cp-slider-val').textContent = inp.value;
  });
  wrap.appendChild(inp);
  return { node: wrap, val: () => parseFloat(inp.value) };
}

async function applyTune(body, label) {
  // 记录 old(new 从 body 取;old 这里无法精确取,简化为记录此次值)
  CP.tuneHistory.unshift({ t: new Date().toLocaleTimeString(), label, body: deepClone(body) });
  if (CP.tuneHistory.length > 20) CP.tuneHistory.pop();
  renderTuneHistory();
  const res = await postControl('params', body);
  toast(res && res.ok ? `${label} 已应用 (${res.applied} 架)` : ('调参失败: ' + (res && res.error)));
}

function renderTuneHistory() {
  const hist = document.getElementById('cp-tune-history');
  if (!hist) return;
  if (!CP.tuneHistory.length) { hist.innerHTML = '<span class="muted">暂无</span>'; return; }
  hist.innerHTML = '';
  CP.tuneHistory.forEach((h, i) => {
    const row = el('div', 'cp-hist-row');
    row.appendChild(el('span', 'cp-hist-t', h.t));
    row.appendChild(el('span', '', h.label));
    const rb = el('button', 'cp-icon-btn', '↺ 回滚');
    rb.addEventListener('click', () => {
      // 回滚:重新提交同 body(无 old 状态时,最实用的回滚=重放上一组)
      postControl('params', h.body).then(r => toast('已回滚 ' + h.label));
    });
    row.appendChild(rb);
    hist.appendChild(row);
  });
}

// ---------------------------------------------------------------------------
// 预设 Tab
// ---------------------------------------------------------------------------
async function renderPresetTab() {
  const root = document.getElementById('cp-preset');
  root.innerHTML = '';
  root.appendChild(el('div', 'cp-card-title', '预设库'));

  const saveRow = el('div', 'cp-field-row');
  const nameInp = document.createElement('input');
  nameInp.type = 'text'; nameInp.placeholder = '预设名';
  saveRow.appendChild(nameInp);
  const saveBtn = el('button', 'cp-submit cp-small', '另存为预设');
  saveBtn.addEventListener('click', async () => {
    const name = nameInp.value.trim();
    if (!name) { toast('请输入预设名'); return; }
    const r = await fetch('/api/presets/' + encodeURIComponent(name), {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(collectDto()),
    });
    const j = await r.json();
    toast(j.ok ? '已保存预设' : ('保存失败: ' + (j.error || '')));
    if (j.ok) renderPresetTab();
  });
  saveRow.appendChild(saveBtn);
  root.appendChild(saveRow);

  // 导出/导入
  const ioRow = el('div', 'cp-field-row');
  const expBtn = el('button', 'cp-small', '⬇ 导出 JSON');
  expBtn.addEventListener('click', () => {
    const blob = new Blob([JSON.stringify(collectDto(), null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = (CP.dto.name || 'scenario') + '.json'; a.click();
    URL.revokeObjectURL(url);
  });
  const impBtn = el('button', 'cp-small', '⬆ 导入 JSON');
  impBtn.addEventListener('click', () => {
    const inp = document.createElement('input');
    inp.type = 'file'; inp.accept = '.json,application/json';
    inp.addEventListener('change', async () => {
      const f = inp.files[0]; if (!f) return;
      try {
        const j = JSON.parse(await f.text());
        CP.dto = j;
        renderConfigTab();
        toast('已导入配置');
      } catch (e) { toast('导入失败: ' + e.message); }
    });
    inp.click();
  });
  ioRow.appendChild(expBtn); ioRow.appendChild(impBtn);
  root.appendChild(ioRow);

  // 预设列表
  const listWrap = el('div', '', '');
  listWrap.id = 'cp-preset-list';
  root.appendChild(listWrap);
  await refreshPresetList();
}

async function refreshPresetList() {
  const lw = document.getElementById('cp-preset-list');
  if (!lw) return;
  let presets = [];
  try {
    const r = await fetch('/api/presets', { cache: 'no-store' });
    const j = await r.json();
    presets = j.presets || [];
  } catch (e) {}
  lw.innerHTML = '';
  if (!presets.length) { lw.appendChild(el('div', 'muted', '暂无预设')); return; }
  presets.forEach(name => {
    const row = el('div', 'cp-preset-row');
    row.appendChild(el('span', '', name));
    const load = el('button', 'cp-small', '加载');
    load.addEventListener('click', async () => {
      const r = await fetch('/api/presets/' + encodeURIComponent(name), { cache: 'no-store' });
      if (r.ok) {
        CP.dto = await r.json();
        renderConfigTab();
        toast('已加载预设: ' + name);
      }
    });
    const del = el('button', 'cp-icon-btn', '✕');
    del.addEventListener('click', async () => {
      const r = await fetch('/api/presets/' + encodeURIComponent(name), { method: 'DELETE' });
      const j = await r.json();
      if (j.ok) { toast('已删除'); refreshPresetList(); }
    });
    row.appendChild(load); row.appendChild(del);
    lw.appendChild(row);
  });
}
