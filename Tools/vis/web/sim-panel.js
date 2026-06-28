import { App, connectSSE, refreshData, setStatus } from './app.js';
import { log } from './logger.js';
import { vecToUE } from './coord.js';

const STORAGE_KEY = 'vis_sim_config';
const CONFIG_VERSION_KEY = 'vis_sim_config_v';
const CONFIG_VERSION = 1;
const DEFAULT_ALTITUDE_M = 5;
const LEGACY_DEFAULT_ALTITUDE_M = 1;
const DEFAULT_WAYPOINT = [50, 0, DEFAULT_ALTITUDE_M];
const DEFAULT_OBSTACLE_CENTER = [25, 0, DEFAULT_ALTITUDE_M];

function waypointPosition(waypoint) {
  if (Array.isArray(waypoint)) return waypoint;
  if (Array.isArray(waypoint?.pos)) return waypoint.pos;
  return DEFAULT_WAYPOINT;
}

let schema = null;
let dto = null;
let controlStatusTimer = null;
let startToken = 0;

export async function initSimPanel() {
  const panel = document.getElementById('sim-panel');
  if (!panel) return;
  await loadSchema();
  if (!schema) {
    panel.innerHTML = '<span class="muted">控制端离线</span>';
    return;
  }
  dto = JSON.parse(JSON.stringify(schema.defaults));
  restoreConfig();
  renderForm();
  pollControlStatus();
}

function restoreConfig() {
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}');
    if (!saved) return;
    const savedVersion = parseInt(localStorage.getItem(CONFIG_VERSION_KEY) || '0', 10);
    if (savedVersion < CONFIG_VERSION) {
      migrateLegacyAltitude(saved);
      localStorage.setItem(CONFIG_VERSION_KEY, String(CONFIG_VERSION));
    }
    if (saved.fleet?.[0]) Object.assign(dto.fleet[0], saved.fleet[0]);
    if (saved.wind) Object.assign(dto.wind, saved.wind);
    if (saved.sim) Object.assign(dto.sim, saved.sim);
    // 空数组不覆盖 SCHEMA 默认值（JS 中 [] 是 truthy，旧代码会静默清空 10 个默认障碍）
    if (saved.obstacles?.length) dto.obstacles = saved.obstacles;
    if (saved.name) dto.name = saved.name;
    log.debug('sim', '配置已从 localStorage 恢复', saved);
  } catch (e) {
    log.warn('sim', '配置恢复失败', { error: e.message });
  }
}

function saveConfig() {
  try {
    const fleet = dto.fleet[0] || {};
    const wp = waypointPosition(fleet.waypoints?.[0]);
    dto.name = document.getElementById('sim-scenario')?.value || dto.name;
    dto.sim.durationSec = parseInt(document.getElementById('sim-duration')?.value) || dto.sim.durationSec;
    dto.sim.slomo = parseFloat(document.getElementById('sim-slomo')?.value) || dto.sim.slomo;
    const data = {
      name: dto.name,
      sim: {
        durationSec: dto.sim.durationSec,
        slomo: dto.sim.slomo,
      },
      fleet: [{
        model: document.getElementById('sim-model')?.value || fleet.model,
        initPos: [
          parseFloat(document.getElementById('sim-init-x')?.value) ?? fleet.initPos?.[0] ?? 0,
          parseFloat(document.getElementById('sim-init-y')?.value) ?? fleet.initPos?.[1] ?? 0,
          parseFloat(document.getElementById('sim-init-z')?.value) ?? fleet.initPos?.[2] ?? DEFAULT_ALTITUDE_M,
        ],
        waypoints: [[
          parseFloat(document.getElementById('sim-wp-x')?.value) ?? wp[0],
          parseFloat(document.getElementById('sim-wp-y')?.value) ?? wp[1],
          parseFloat(document.getElementById('sim-wp-z')?.value) ?? wp[2],
        ]],
      }],
      wind: {
        type: document.getElementById('sim-wind-type')?.value || dto.wind.type,
        steady: [
          parseFloat(document.getElementById('sim-wind-x')?.value) ?? dto.wind.steady?.[0] ?? 0,
          parseFloat(document.getElementById('sim-wind-y')?.value) ?? dto.wind.steady?.[1] ?? 0,
          parseFloat(document.getElementById('sim-wind-z')?.value) ?? dto.wind.steady?.[2] ?? 0,
        ],
      },
      obstacles: dto.obstacles || [],
    };
    localStorage.setItem(STORAGE_KEY, JSON.stringify(data));
  } catch (e) {
    log.warn('sim', '配置保存失败', { error: e.message });
  }
}

function migrateLegacyAltitude(saved) {
  const fleet = saved.fleet?.[0];
  if (isLegacyVec(fleet?.initPos, 0, 0)) fleet.initPos[2] = DEFAULT_ALTITUDE_M;
  const waypoint = fleet?.waypoints?.[0];
  if (isLegacyVec(waypoint, 50, 0)) waypoint[2] = DEFAULT_ALTITUDE_M;
  for (const obstacle of saved.obstacles || []) {
    if (isLegacyVec(obstacle.center, 25, 0)) obstacle.center[2] = DEFAULT_ALTITUDE_M;
  }
}

function isLegacyVec(vec, x, y) {
  return Array.isArray(vec) && vec[0] === x && vec[1] === y && vec[2] === LEGACY_DEFAULT_ALTITUDE_M;
}

async function loadSchema() {
  try {
    const r = await fetch('/api/schema');
    schema = await r.json();
    log.info('sim', 'schema 加载', { models: schema.models?.length });
  } catch (e) {
    log.warn('sim', 'schema 加载失败', { error: e.message });
    schema = null;
  }
}

function updateSimButtons(state) {
  const startBtn = document.getElementById('sim-start-btn');
  const stopBtn = document.getElementById('sim-stop-btn');
  if (startBtn && !startBtn.dataset.starting) {
    startBtn.disabled = state === 'running' || state === 'stopping';
  }
  if (stopBtn) {
    stopBtn.disabled = state === 'idle' || state === 'stopping';
  }
}

async function pollControlStatus() {
  const check = async () => {
    try {
      const r = await fetch('/api/control/status');
      const d = await r.json();
      const available = d.controlAvailable;
      const simRunning = d.simStatus?.running;
      updateSimButtons(simRunning ? 'running' : 'idle');
      const badge = document.getElementById('sim-status-badge');
      if (badge) {
        if (simRunning) {
          badge.textContent = `● 仿真运行中 (${d.simStatus.elapsedSec}s/${d.simStatus.duration}s)`;
          badge.className = 'sim-badge online';
        } else if (available) {
          badge.textContent = '● 控制端在线';
          badge.className = 'sim-badge online';
        } else {
          badge.textContent = '○ 离线(点启动拉起)';
          badge.className = 'sim-badge offline';
        }
      }
    } catch {}
  };
  check();
  if (controlStatusTimer) clearInterval(controlStatusTimer);
  controlStatusTimer = setInterval(check, 2000);
}

function renderForm() {
  const panel = document.getElementById('sim-panel');
  const s = dto.sim;
  const w = dto.wind;
  const fleet = dto.fleet[0] || {};
  const wp = waypointPosition(fleet.waypoints?.[0]);

  panel.innerHTML = `
    <div class="sim-section">
      <label class="sim-label">仿真时长 (秒)</label>
      <input type="number" id="sim-duration" class="sim-input" value="${s.durationSec}" min="1" max="3600" />
    </div>
    <div class="sim-section">
      <label class="sim-label">仿真倍速</label>
      <input type="number" id="sim-slomo" class="sim-input" value="${s.slomo}" min="0.1" max="100" step="0.5" />
    </div>
    <div class="sim-section">
      <label class="sim-label">场景名</label>
      <input type="text" id="sim-scenario" class="sim-input" value="${dto.name}" />
    </div>
    <div class="sim-section">
      <label class="sim-label">无人机型号</label>
      <select id="sim-model" class="sim-input">
        ${schema.models.map(m => `<option value="${m.id}" ${m.id === fleet.model ? 'selected' : ''}>${m.label}</option>`).join('')}
      </select>
    </div>
    <div class="sim-section">
      <label class="sim-label">起点 (x y z, 米)</label>
      <div class="sim-row3">
        <input type="number" id="sim-init-x" class="sim-input-sm" value="${fleet.initPos?.[0] ?? 0}" step="0.5" />
        <input type="number" id="sim-init-y" class="sim-input-sm" value="${fleet.initPos?.[1] ?? 0}" step="0.5" />
        <input type="number" id="sim-init-z" class="sim-input-sm" value="${fleet.initPos?.[2] ?? DEFAULT_ALTITUDE_M}" step="0.5" />
      </div>
    </div>
    <div class="sim-section">
      <label class="sim-label">目标航点 (x y z, 米)</label>
      <div class="sim-row3">
        <input type="number" id="sim-wp-x" class="sim-input-sm" value="${wp[0]}" step="0.5" />
        <input type="number" id="sim-wp-y" class="sim-input-sm" value="${wp[1]}" step="0.5" />
        <input type="number" id="sim-wp-z" class="sim-input-sm" value="${wp[2]}" step="0.5" />
      </div>
    </div>
    <div class="sim-section">
      <label class="sim-label">风向风速</label>
      <div class="sim-row3">
        <input type="number" id="sim-wind-x" class="sim-input-sm" value="${w.steady[0]}" step="0.5" />
        <input type="number" id="sim-wind-y" class="sim-input-sm" value="${w.steady[1]}" step="0.5" />
        <input type="number" id="sim-wind-z" class="sim-input-sm" value="${w.steady[2]}" step="0.5" />
      </div>
    </div>
    <div class="sim-section">
      <label class="sim-label">风类型</label>
      <select id="sim-wind-type" class="sim-input">
        ${schema.windTypes.map(t => `<option value="${t}" ${t === w.type ? 'selected' : ''}>${t}</option>`).join('')}
      </select>
    </div>
    <div class="sim-section">
      <label class="sim-label">障碍物 (中心 x y z, 半径/尺寸 xyz, 米)</label>
      <div id="sim-obstacles"></div>
      <div class="sim-btn-row" style="margin-top:4px">
        <button id="sim-add-obstacle" class="sim-btn" style="background:var(--card2);color:var(--text);border:1px solid var(--border)">+ 添加障碍</button>
      </div>
    </div>
    <div class="sim-actions">
      <button id="sim-start-btn" class="sim-btn sim-btn-start">启动仿真</button>
      <button id="sim-stop-btn" class="sim-btn sim-btn-stop">停止</button>
    </div>
    <span id="sim-status-badge" class="sim-badge offline">○ 检测中…</span>
  `;

  document.getElementById('sim-start-btn').addEventListener('click', startSim);
  document.getElementById('sim-stop-btn').addEventListener('click', stopSim);
  document.getElementById('sim-add-obstacle').addEventListener('click', () => {
    dto.obstacles = dto.obstacles || [];
    dto.obstacles.push(defaultObstacle());
    renderObstacles();
    saveConfig();
  });

  renderObstacles();

  panel.querySelectorAll('input, select').forEach(el => {
    el.addEventListener('change', saveConfig);
  });
}

function defaultObstacle() {
  return { type: 'Box', center: [...DEFAULT_OBSTACLE_CENTER], extents: [2, 2, 2], movement: 'Static', safetyMargin: 0.5 };
}

function renderObstacles() {
  const wrap = document.getElementById('sim-obstacles');
  if (!wrap) return;
  const obs = dto.obstacles || [];
  wrap.innerHTML = obs.map((o, i) => {
    const c = o.center || DEFAULT_OBSTACLE_CENTER;
    const e = o.extents || [2, 2, 2];
    return `
      <div class="sim-obstacle-row" data-i="${i}" style="border:1px solid var(--border);border-radius:4px;padding:6px;margin-bottom:4px;background:var(--card2)">
        <div class="sim-row3" style="align-items:center;margin-bottom:4px">
          <select class="sim-input-sm obs-type" style="flex:1.5">
            ${schema.obstacleTypes.map(t => `<option value="${t}" ${t === o.type ? 'selected' : ''}>${t}</option>`).join('')}
          </select>
          <select class="sim-input-sm obs-move" style="flex:1.5">
            ${schema.movements.map(m => `<option value="${m}" ${m === o.movement ? 'selected' : ''}>${m}</option>`).join('')}
          </select>
          <button class="sim-btn obs-del" style="flex:0.4;background:var(--bad);color:#fff;padding:4px">×</button>
        </div>
        <div style="font-size:11px;color:var(--muted);margin-bottom:2px">中心 (x y z)</div>
        <div class="sim-row3">
          <input type="number" class="sim-input-sm obs-cx" value="${c[0]}" step="0.5" />
          <input type="number" class="sim-input-sm obs-cy" value="${c[1]}" step="0.5" />
          <input type="number" class="sim-input-sm obs-cz" value="${c[2]}" step="0.5" />
        </div>
        <div style="font-size:11px;color:var(--muted);margin:2px 0">尺寸/半径 (x y z)</div>
        <div class="sim-row3">
          <input type="number" class="sim-input-sm obs-ex" value="${e[0]}" step="0.5" min="0.1" />
          <input type="number" class="sim-input-sm obs-ey" value="${e[1]}" step="0.5" min="0.1" />
          <input type="number" class="sim-input-sm obs-ez" value="${e[2]}" step="0.5" min="0.1" />
        </div>
      </div>`;
  }).join('');

  wrap.querySelectorAll('.sim-obstacle-row').forEach(row => {
    const i = parseInt(row.dataset.i);
    const update = (key, val) => { dto.obstacles[i][key] = val; saveConfig(); };
    row.querySelector('.obs-type').addEventListener('change', e => update('type', e.target.value));
    row.querySelector('.obs-move').addEventListener('change', e => update('movement', e.target.value));
    row.querySelector('.obs-del').addEventListener('click', () => { dto.obstacles.splice(i, 1); renderObstacles(); saveConfig(); });
    row.querySelector('.obs-cx').addEventListener('change', e => updateCenter(dto.obstacles[i], 0, parseFloat(e.target.value) || 0));
    row.querySelector('.obs-cy').addEventListener('change', e => updateCenter(dto.obstacles[i], 1, parseFloat(e.target.value) || 0));
    row.querySelector('.obs-cz').addEventListener('change', e => updateCenter(dto.obstacles[i], 2, parseFloat(e.target.value) || 0));
    row.querySelector('.obs-ex').addEventListener('change', e => updateExtents(dto.obstacles[i], 0, parseFloat(e.target.value) || 1));
    row.querySelector('.obs-ey').addEventListener('change', e => updateExtents(dto.obstacles[i], 1, parseFloat(e.target.value) || 1));
    row.querySelector('.obs-ez').addEventListener('change', e => updateExtents(dto.obstacles[i], 2, parseFloat(e.target.value) || 1));
  });
}

function updateCenter(obs, axis, val) {
  obs.center = obs.center || [...DEFAULT_OBSTACLE_CENTER];
  obs.center[axis] = val;
  saveConfig();
}

function updateExtents(obs, axis, val) {
  obs.extents = obs.extents || [2, 2, 2];
  obs.extents[axis] = val;
  saveConfig();
}

async function startSim() {
  const token = ++startToken;
  saveConfig();
  const duration = parseInt(document.getElementById('sim-duration').value) || 60;
  const slomo = parseFloat(document.getElementById('sim-slomo').value) || 8;
  const scenario = document.getElementById('sim-scenario').value || '';
  const model = document.getElementById('sim-model').value;
  const initPos = [
    parseFloat(document.getElementById('sim-init-x').value) || 0,
    parseFloat(document.getElementById('sim-init-y').value) || 0,
    parseFloat(document.getElementById('sim-init-z').value) || DEFAULT_ALTITUDE_M,
  ];
  const wp = [
    parseFloat(document.getElementById('sim-wp-x').value) || 50,
    parseFloat(document.getElementById('sim-wp-y').value) || 0,
    parseFloat(document.getElementById('sim-wp-z').value) || DEFAULT_ALTITUDE_M,
  ];
  const windSteady = [
    parseFloat(document.getElementById('sim-wind-x').value) || 0,
    parseFloat(document.getElementById('sim-wind-y').value) || 0,
    parseFloat(document.getElementById('sim-wind-z').value) || 0,
  ];
  const windType = document.getElementById('sim-wind-type').value;

  const configDto = {
    fleet: [{
      model,
      initPos: vecToUE(initPos),
      yaw: 0,
      isLeader: true,
      mode: 'Once',
      waypoints: [{ pos: vecToUE(wp), speed: 0, hover: 0 }],
    }],
    wind: {
      type: windType,
      steady: vecToUE(windSteady),
      enabled: true,
      gustAmplitude: 2,
      turbulenceIntensity: 1,
    },
    obstacles: (dto.obstacles || []).map(o => {
      const out = {
        type: o.type || 'Box',
        center: vecToUE(o.center || DEFAULT_OBSTACLE_CENTER),
        extents: (o.extents || [2, 2, 2]).map(v => Math.max(0.1, v) * 100),
        safetyMargin: Math.max(0, o.safetyMargin || 0) * 100,
        movement: o.movement || 'Static',
      };
      if (o.movement === 'LinearVelocity' && o.velocity) {
        out.velocity = vecToUE(o.velocity);
      } else if ((o.movement === 'PatrolLoop' || o.movement === 'PatrolPingPong') && o.patrolPoints) {
        out.patrolPoints = o.patrolPoints.map(p => vecToUE(p));
        out.patrolSpeed = Math.max(0, o.patrolSpeed || 3) * 100;
      }
      return out;
    }),
    sim: { slomo, durationSec: duration, controlMode: '', mpcType: '' },
    name: scenario,
  };

  const btn = document.getElementById('sim-start-btn');
  btn.disabled = true;
  btn.dataset.starting = '1';
  updateSimButtons('running');

  try {
    btn.textContent = '启动中…';
    setStatus('启动仿真(后端自动拉起UE+推送配置)…', false);
    log.info('sim', 'POST /api/sim/start (统一启动)', { duration, slomo, scenario });

    const r = await fetch('/api/sim/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(configDto),
    });
    const res = await r.json();
    if (!r.ok || !res.ok) {
      throw new Error(res.error || `启动失败 (HTTP ${r.status})`);
    }

    log.info('sim', '仿真已启动并配置完成', res);
    App.currentTaskId = null;
    const taskSel = document.getElementById('task-select');
    if (taskSel) taskSel.value = '';
    App.cursorT = 0;
    App.playing = true;
    App.finished = false;
    App._lastReloadEpoch = undefined;
    connectSSE();
    setStatus('仿真运行中', true);
    setTimeout(() => refreshData(), 1000);
  } catch (e) {
    if (token === startToken) {
      log.error('sim', '启动失败', { error: e.message });
      setStatus('启动失败: ' + e.message, false);
    } else {
      log.info('sim', '启动已取消');
    }
  } finally {
    if (token === startToken) {
      delete btn.dataset.starting;
      btn.textContent = '启动仿真';
      pollControlStatus();
    } else {
      updateSimButtons('idle');
    }
  }
}

async function stopSim() {
  startToken++;
  const stopBtn = document.getElementById('sim-stop-btn');
  if (stopBtn) stopBtn.disabled = true;
  try {
    const statusResponse = await fetch('/api/control/status');
    const statusData = await statusResponse.json();
    if (statusData.controlAvailable) {
      await fetch('/api/control/stop', { method: 'POST' });
    }
    const simStopResponse = await fetch('/api/sim/stop', { method: 'POST' });
    const simStop = await simStopResponse.json();
    log.info('sim', '停止请求已发送', simStop);
    setStatus('已停止', false);
  } catch (e) {
    log.error('sim', '停止异常', { error: e.message });
  } finally {
    const btn = document.getElementById('sim-start-btn');
    if (btn) {
      delete btn.dataset.starting;
      btn.textContent = '启动仿真';
    }
    updateSimButtons('idle');
  }
}
