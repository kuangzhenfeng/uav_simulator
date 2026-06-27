// UAV 仿真可视化前端主逻辑
// 依赖: three(本地内置)、原生 Canvas 图表、EventSource(SSE)实时增量。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';

// ---------------------------------------------------------------------------
// 全局状态
// ---------------------------------------------------------------------------
const App = {
  data: null,            // 最近一次 /api/data 快照
  scene: null, renderer: null, camera: null, controls: null,
  agentObjects: new Map(),   // id -> { group, trail, body }
  obstacleObjects: [],
  waypointObjects: [],
  windArrow: null,
  gridGroup: null,
  // 时间控制
  playing: true,
  cursorT: 0,            // 当前回放游标(秒)
  durationSec: 0,
  // 图表
  charts: {},
  // 实时
  es: null,
  finished: false,
  // 未来轨迹显示开关（优化轨迹默认开，规划路径/NMPC 预测默认关，避免初始过载）
  showFuture: { opt: true, plan: false, nmpc: false },
};

// 仅用于运行时调试/自检:暴露内部状态(无副作用)
if (typeof window !== 'undefined') window.__vis = { App };

// 主循环状态变量(模块级,避免在 loop 调用前进入 TDZ)
let lastTs = performance.now();
let chartAccum = 0;

const COLORS = {
  ground: 0x1a2240,
  grid: 0x2c3552,
  obstacle: 0xff6b6b,
  obstacleEdge: 0xff8a8a,
  waypoint: 0xfcc419,
  wind: 0x51cf66,
};

// ---------------------------------------------------------------------------
// 启动
// ---------------------------------------------------------------------------
init();
loop();

async function init() {
  initThree();
  bindUI();
  // 先拉一次全量数据,再连 SSE 增量
  await refreshData();
  connectSSE();
}

// ---------------------------------------------------------------------------
// Three.js 初始化
// ---------------------------------------------------------------------------
function initThree() {
  const canvas = document.getElementById('three-canvas');
  const wrap = document.getElementById('scene-wrap');
  App.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  App.renderer.setPixelRatio(window.devicePixelRatio);
  App.renderer.setSize(wrap.clientWidth, wrap.clientHeight, false);

  App.scene = new THREE.Scene();
  App.scene.background = new THREE.Color(0x0a0d18);
  App.scene.fog = new THREE.Fog(0x0a0d18, 120, 400);

  App.camera = new THREE.PerspectiveCamera(55, wrap.clientWidth / wrap.clientHeight, 0.1, 2000);
  App.camera.position.set(60, 70, 90);

  App.controls = new OrbitControls(App.camera, canvas);
  App.controls.enableDamping = true;
  App.controls.target.set(30, 5, 0);

  // 灯光
  const hemi = new THREE.HemisphereLight(0xbfd4ff, 0x202840, 0.9);
  App.scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 0.7);
  dir.position.set(50, 100, 40);
  App.scene.add(dir);

  // 地面 + 网格占位,数据到了再重建
  rebuildGround([[-50, -50], [150, 50]]);

  window.addEventListener('resize', onResize);
}

function rebuildGround(bounds) {
  if (App.gridGroup) App.scene.remove(App.gridGroup);
  App.gridGroup = new THREE.Group();
  const [min, max] = bounds;
  const w = max[0] - min[0], d = max[1] - min[1];
  const cx = (min[0] + max[0]) / 2, cz = (min[1] + max[1]) / 2;
  const size = Math.max(w, d, 40);

  const grid = new THREE.GridHelper(size, Math.ceil(size / 5), COLORS.grid, COLORS.grid);
  grid.position.set(cx, 0, cz);
  App.gridGroup.add(grid);

  // 地面板
  const planeGeo = new THREE.PlaneGeometry(size, size);
  const planeMat = new THREE.MeshBasicMaterial({ color: COLORS.ground, transparent: true, opacity: 0.5 });
  const plane = new THREE.Mesh(planeGeo, planeMat);
  plane.rotation.x = -Math.PI / 2;
  plane.position.set(cx, -0.02, cz);
  App.gridGroup.add(plane);

  App.scene.add(App.gridGroup);
}

function onResize() {
  const wrap = document.getElementById('scene-wrap');
  App.camera.aspect = wrap.clientWidth / wrap.clientHeight;
  App.camera.updateProjectionMatrix();
  App.renderer.setSize(wrap.clientWidth, wrap.clientHeight, false);
}

// ---------------------------------------------------------------------------
// 数据刷新
// ---------------------------------------------------------------------------
async function refreshData() {
  try {
    const r = await fetch('/api/data', { cache: 'no-store' });
    App.data = await r.json();
    onNewData();
  } catch (e) {
    setStatus('连接失败', false);
  }
}

function connectSSE() {
  try {
    App.es = new EventSource('/api/stream');
    App.es.onmessage = (ev) => {
      try {
        App.data = JSON.parse(ev.data);
        onNewData();
      } catch (e) {}
    };
    App.es.addEventListener('done', () => {
      App.finished = true;
      setStatus('仿真结束 · 回放', false);
    });
    App.es.onerror = () => {
      setStatus('实时连接中断,5s 后重试…', false);
    };
  } catch (e) {}
}

function onNewData() {
  const d = App.data;
  if (!d) return;
  const live = !d.finished;
  setStatus(d.finished ? (d.scenario ? `已结束 · ${d.scenario}` : '已结束') : `实时仿真中 · ${d.scenario || ''}`, live);

  App.durationSec = Math.max(d.duration_ms / 1000, 0.001);
  // 实时播放时游标跟随到末尾
  if (live && App.playing) {
    App.cursorT = App.durationSec;
  }

  rebuildScene(d);
  renderVerdict(d);
  renderSummary(d);
  renderEvents(d);
  renderLegend(d);
  renderCharts(d);
  renderWindLabel(d);
  updateTimelineUI();
}

// ---------------------------------------------------------------------------
// 3D 场景重建
// ---------------------------------------------------------------------------
function clearObjects(list) {
  for (const o of list) {
    App.scene.remove(o);
    disposeObj(o);
  }
  list.length = 0;
}

function disposeObj(obj) {
  obj.traverse?.(c => {
    c.geometry?.dispose?.();
    if (c.material) {
      Array.isArray(c.material) ? c.material.forEach(m => m.dispose()) : c.material.dispose();
    }
  });
}

function rebuildScene(d) {
  // 计算包围盒用于地面
  const pts = [];
  d.agents.forEach(a => { if (a.initPos) pts.push(a.initPos); a.trace.forEach(t => pts.push(t.pos)); });
  d.obstacles.forEach(o => o.center && pts.push(o.center));
  d.waypoints.forEach(w => pts.push(w));
  const bounds = pts.length ? bbox2d(pts) : [[-50, -50], [150, 50]];
  rebuildGround(bounds);

  // 障碍物
  clearObjects(App.obstacleObjects);
  d.obstacles.forEach(o => {
    const mesh = makeObstacle(o);
    App.scene.add(mesh);
    App.obstacleObjects.push(mesh);
  });

  // 航点
  clearObjects(App.waypointObjects);
  d.waypoints.forEach((w, i) => {
    const mesh = makeWaypoint(w, i, d.waypoints.length);
    App.scene.add(mesh);
    App.waypointObjects.push(mesh);
  });

  // 飞机对象(按 id 复用,避免重建闪烁)
  const ids = new Set(d.agents.map(a => a.id));
  for (const [id, obj] of App.agentObjects) {
    if (!ids.has(id)) { App.scene.remove(obj.group); disposeObj(obj.group); App.agentObjects.delete(id); }
  }
  d.agents.forEach(a => {
    let obj = App.agentObjects.get(a.id);
    if (!obj) {
      obj = makeAgent(a);
      App.scene.add(obj.group);
      App.agentObjects.set(a.id, obj);
    }
    obj.def = a; // 保留最新定义(颜色/半径)
    obj.body.traverse(c => { if (c.material && c.material.color) c.material.color.set(a.color); });
    obj.sphere.material.color.set(a.color);
    obj.sphere.material.opacity = 0.12;
    // 轨迹（历史，累积式）
    obj.trail.geometry.dispose();
    obj.trail.geometry = trailGeometry(a.trace);
    obj.trail.material.color.set(a.color);

    // 未来轨迹（覆盖式最新快照）
    // 优化轨迹：实线，可见性受开关控制
    obj.futureOpt.geometry.dispose();
    obj.futureOpt.geometry = pointsGeometry(a.futureOpt || []);
    obj.futureOpt.material.color.set(a.color);
    obj.futureOpt.visible = App.showFuture.opt && (a.futureOpt && a.futureOpt.length >= 2);
    // 规划路径：蓝色，受开关控制
    obj.futurePlan.geometry.dispose();
    obj.futurePlan.geometry = pointsGeometry(a.futurePlan || []);
    obj.futurePlan.visible = App.showFuture.plan && (a.futurePlan && a.futurePlan.length >= 2);
    // NMPC 预测：橙色，受开关控制；点按 cost 热度着色（线色仍橙，热点另见下）
    obj.futureNmpc.geometry.dispose();
    obj.futureNmpc.geometry = pointsGeometry(a.futureNmpc || []);
    obj.futureNmpc.visible = App.showFuture.nmpc && (a.futureNmpc && a.futureNmpc.length >= 2);
  });

  // 风场箭头(角落指示全局风向)
  updateWindArrow(d);
}

function bbox2d(pts) {
  let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
  for (const [x, , z] of pts) {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  const pad = Math.max(8, (maxX - minX) * 0.12, (maxZ - minZ) * 0.12);
  return [[minX - pad, minZ - pad], [maxX + pad, maxZ + pad]];
}

function makeObstacle(o) {
  const grp = new THREE.Group();
  // web extents = [x_half, y_half, z_half], y 向上
  const sx = Math.max((o.extents[0] || 1) * 2, 0.2);
  const sy = Math.max((o.extents[1] || 1) * 2, 0.2);
  const sz = Math.max((o.extents[2] || 1) * 2, 0.2);
  const geo = new THREE.BoxGeometry(sx, sy, sz);
  const mat = new THREE.MeshLambertMaterial({ color: COLORS.obstacle, transparent: true, opacity: 0.32 });
  const box = new THREE.Mesh(geo, mat);
  grp.add(box);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color: COLORS.obstacleEdge }));
  grp.add(edges);
  // center.y 抬升半个高度,使其坐落在地面
  grp.position.set(o.center[0], (o.center[1] || 0) + (sy / 2), o.center[2]);
  return grp;
}

function makeWaypoint(w, idx, total) {
  const grp = new THREE.Group();
  const isStart = idx === 0;
  const isEnd = idx === total - 1 && total > 1;
  const color = isStart ? 0x4f8cff : (isEnd ? 0x51cf66 : COLORS.waypoint);
  const ringGeo = new THREE.TorusGeometry(1.2, 0.12, 8, 32);
  const ring = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color }));
  ring.rotation.x = -Math.PI / 2;
  grp.add(ring);
  // 竖杆
  const poleGeo = new THREE.CylinderGeometry(0.06, 0.06, 3, 8);
  const pole = new THREE.Mesh(poleGeo, new THREE.MeshBasicMaterial({ color }));
  pole.position.y = 1.5;
  grp.add(pole);
  grp.position.set(w[0], w[1] || 0, w[2]);
  grp.userData.idx = idx;
  return grp;
}

function makeAgent(a) {
  const grp = new THREE.Group();
  // 机身
  const body = new THREE.Group();
  const hub = new THREE.Mesh(
    new THREE.SphereGeometry(0.18, 12, 10),
    new THREE.MeshLambertMaterial({ color: a.color })
  );
  body.add(hub);
  // 4 臂 + 旋翼
  const armLen = 0.55;
  for (let i = 0; i < 4; i++) {
    const ang = (Math.PI / 2) * i + Math.PI / 4;
    const arm = new THREE.Mesh(
      new THREE.CylinderGeometry(0.03, 0.03, armLen, 6),
      new THREE.MeshLambertMaterial({ color: a.color })
    );
    arm.rotation.z = Math.PI / 2;
    arm.rotation.y = ang;
    arm.position.set(Math.cos(ang) * armLen / 2, 0, Math.sin(ang) * armLen / 2);
    body.add(arm);
    const rotor = new THREE.Mesh(
      new THREE.CylinderGeometry(0.28, 0.28, 0.02, 12),
      new THREE.MeshLambertMaterial({ color: a.color, transparent: true, opacity: 0.4 })
    );
    rotor.position.set(Math.cos(ang) * armLen, 0.03, Math.sin(ang) * armLen);
    body.add(rotor);
  }
  grp.add(body);

  // 包围球
  const r = Math.max(a.collisionRadiusM || 0.6, 0.3);
  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(r, 16, 12),
    new THREE.MeshBasicMaterial({ color: a.color, transparent: true, opacity: 0.12, wireframe: false })
  );
  grp.add(sphere);

  // 轨迹折线（历史）
  const trail = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: a.color, transparent: true, opacity: 0.7 })
  );
  grp.add(trail);

  // 未来轨迹（覆盖式最新快照，区别于历史 trail 的累积式）。
  // 三类各一线：优化轨迹(实线高透明)、规划路径(虚线)、NMPC 预测(虚线+按 cost 热度)。
  // 着色沿用 agent 主色，靠透明度/虚线样式区分，保证多机可分辨。
  const futureOpt = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: a.color, transparent: true, opacity: 0.85 })
  );
  grp.add(futureOpt);

  const futurePlan = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0x4f8cff, transparent: true, opacity: 0.6 })
  );
  futurePlan.material.linewidth = 2;
  // 虚线（DashedMaterial 需 computeLineDistances）
  grp.add(futurePlan);

  const futureNmpc = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0xff922b, transparent: true, opacity: 0.7 })
  );
  grp.add(futureNmpc);

  return { group: grp, body, sphere, trail, futureOpt, futurePlan, futureNmpc, def: a };
}

function trailGeometry(trace) {
  const geo = new THREE.BufferGeometry();
  if (!trace.length) return geo;
  const pos = new Float32Array(trace.length * 3);
  trace.forEach((t, i) => { pos[i*3] = t.pos[0]; pos[i*3+1] = t.pos[1]; pos[i*3+2] = t.pos[2]; });
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  return geo;
}

// 由点数组 [x,y,z(,cost)] 构造几何体（未来轨迹快照用）。
// cost 仅用于 NMPC 热度着色，几何构造时不参与。
function pointsGeometry(pts) {
  const geo = new THREE.BufferGeometry();
  if (!pts || !pts.length) return geo;
  const pos = new Float32Array(pts.length * 3);
  pts.forEach((p, i) => { pos[i*3] = p[0]; pos[i*3+1] = p[1]; pos[i*3+2] = p[2]; });
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  return geo;
}

// NMPC 预测点按障碍代价 cost 热度着色（复刻 PlanningVisualizer 色映射：cost 越高越红）
function costToColor(cost) {
  const ratio = Math.max(0, Math.min(1, cost / 10.0));
  const r = Math.round(255 * (1 - ratio) + 255 * ratio);       // 始终接近 255
  const g = Math.round(255 * (1 - ratio) + 100 * ratio * (1 - ratio) * 0);
  return rgbToInt(r, Math.round(165 * (1 - ratio)), 0);
}
function rgbToInt(r, g, b) { return (r << 16) | (g << 8) | b; }

function updateWindArrow(d) {
  const samples = d.wind?.samples || [];
  if (!samples.length) {
    if (App.windArrow) App.windArrow.visible = false;
    return;
  }
  const last = samples[samples.length - 1];
  if (!App.windArrow) {
    App.windArrow = new THREE.Group();
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(0.15, 0.15, 4, 10),
      new THREE.MeshBasicMaterial({ color: COLORS.wind })
    );
    shaft.position.y = 2;
    App.windArrow.add(shaft);
    const head = new THREE.Mesh(
      new THREE.ConeGeometry(0.5, 1.2, 12),
      new THREE.MeshBasicMaterial({ color: COLORS.wind })
    );
    head.position.y = 4.6;
    App.windArrow.add(head);
    App.scene.add(App.windArrow);
  }
  App.windArrow.visible = true;
  // 位置放在场景角落
  const b = bboxOfScene(d);
  App.windArrow.position.set(b.maxX + 4, 0, b.minZ - 4);
  // 方向:风矢量 vecMs=[x,y,z],y 向上;只取水平 x/z 朝向
  const v = last.vecMs;
  const len = Math.hypot(v[0], v[2]);
  if (len > 1e-3) {
    // 默认箭头朝 +Y,绕 Z 轴旋转到 (x,z) 方向
    const yaw = Math.atan2(v[0], v[2]);
    App.windArrow.rotation.set(0, yaw, 0);
    // 缩放体现风速大小(2m/s 为基准)
    const s = Math.max(0.5, Math.min(3.0, last.speedMs / 2));
    App.windArrow.scale.set(s, s, s);
  }
}

function bboxOfScene(d) {
  let minX = 0, maxX = 80, minZ = -40, maxZ = 40;
  const xs = [], zs = [];
  d.agents.forEach(a => a.trace.forEach(t => { xs.push(t.pos[0]); zs.push(t.pos[2]); }));
  d.obstacles.forEach(o => { if (o.center) { xs.push(o.center[0]); zs.push(o.center[2]); } });
  d.waypoints.forEach(w => { xs.push(w[0]); zs.push(w[2]); });
  if (xs.length) { minX = Math.min(...xs); maxX = Math.max(...xs); minZ = Math.min(...zs); maxZ = Math.max(...zs); }
  return { minX, maxX, minZ, maxZ };
}

// ---------------------------------------------------------------------------
// 根据 cursorT 更新飞机位置 / 朝向
// ---------------------------------------------------------------------------
function updateAgentPose() {
  if (!App.data) return;
  for (const [id, obj] of App.agentObjects) {
    const trace = obj.def.trace;
    if (!trace.length) {
      const ip = obj.def.initPos;
      obj.group.position.set(ip ? ip[0] : 0, ip ? ip[1] : 0, ip ? ip[2] : 0);
      obj.group.visible = !!ip;
      continue;
    }
    const p = sampleTrace(trace, App.cursorT);
    obj.group.position.set(p.pos[0], p.pos[1], p.pos[2]);
    // 朝向:用与下一采样点的水平方向
    const seg = traceSegmentDir(trace, App.cursorT);
    if (seg) {
      obj.group.rotation.set(0, Math.atan2(seg[0], seg[2]), 0);
    }
  }
}

function sampleTrace(trace, t) {
  if (t <= trace[0].t) return trace[0];
  if (t >= trace[trace.length - 1].t) return trace[trace.length - 1];
  for (let i = 0; i < trace.length - 1; i++) {
    if (t >= trace[i].t && t <= trace[i+1].t) {
      const a = trace[i], b = trace[i+1];
      const k = (t - a.t) / Math.max(b.t - a.t, 1e-6);
      return { pos: lerp3(a.pos, b.pos, k), speed: a.speed };
    }
  }
  return trace[trace.length - 1];
}

function traceSegmentDir(trace, t) {
  for (let i = 0; i < trace.length - 1; i++) {
    if (t >= trace[i].t && t <= trace[i+1].t) {
      const a = trace[i].pos, b = trace[i+1].pos;
      return [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    }
  }
  return null;
}

function lerp3(a, b, k) { return [a[0]+(b[0]-a[0])*k, a[1]+(b[1]-a[1])*k, a[2]+(b[2]-a[2])*k]; }

// ---------------------------------------------------------------------------
// 面板渲染
// ---------------------------------------------------------------------------
function renderVerdict(d) {
  const el = document.getElementById('verdict');
  const det = document.getElementById('verdict-detail');
  // 优先用 scenario_result.json(更权威),否则用逐秒推算
  const r = d.verdictResult || d.verdict;
  const final = d.verdictResult?.final || d.verdict?.final || 'UNKNOWN';
  el.textContent = final === 'PASS' ? '✓ PASS' : (final === 'FAIL' ? '✗ FAIL' : '? 待定');
  el.className = 'verdict ' + final;

  if (!r) { det.innerHTML = '<span class="muted">等待数据…</span>'; return; }
  const rows = [];
  if (d.verdictResult) {
    rows.push(['场景', d.scenario || '-']);
    rows.push(['判决', final]);
    rows.push(['航点完成', r.reached]);
    rows.push(['最小净空', r.clearanceM != null ? `${r.clearanceM} m` : '-']);
    rows.push(['最大横向偏差', `${r.lateralDevM} m`]);
    rows.push(['耗时', `${r.elapsedS} s`]);
    rows.push(['碰撞', r.collided ? '是' : '否']);
    if (r.failures?.length) rows.push(['失败项', r.failures.join('; ')]);
  } else if (d.verdict) {
    rows.push(['场景', d.scenario || '-']);
    rows.push(['判决(推算)', final]);
    rows.push(['航点完成', r.reached]);
    rows.push(['最小净空', r.clearanceM != null ? `${r.clearanceM} m` : '-']);
    rows.push(['最大横向偏差', `${r.lateralDevM} m`]);
    rows.push(['耗时', `${r.elapsedS} s`]);
  }
  det.innerHTML = rows.map(([k, v]) => `<div class="row"><span class="k">${k}</span><span class="v">${v}</span></div>`).join('');
}

function renderSummary(d) {
  const el = document.getElementById('summary-table');
  const rows = d.summary;
  if (!rows?.length) { el.innerHTML = '<span class="muted">尚无汇总</span>'; return; }
  const cols = [
    ['Agent', s => s.agent],
    ['速度比', s => s.speedRatio, v => v < 0.5],
    ['低速时长', s => `${s.lowSpeedDur}s`],
    ['最大偏差', s => s.maxDev > 100 ? `${(s.maxDev/100).toFixed(1)}m` : `${s.maxDev}cm`],
    ['Roll', s => s.maxRoll, v => v > 60],
    ['Pitch', s => s.maxPitch, v => v > 60],
    ['卡死', s => s.stuck ? '是' : '否', v => v],
  ];
  let html = '<table><thead><tr>';
  cols.forEach(c => html += `<th>${c[0]}</th>`);
  html += '</tr></thead><tbody>';
  rows.forEach(s => {
    const color = App.data.agents.find(a => a.id === s.agent)?.color || '#ccc';
    html += '<tr>';
    cols.forEach((c, i) => {
      let val = c[1](s);
      const bad = c[2] && c[2](typeof val === 'number' ? val : (c[1](s)));
      const disp = i === 0
        ? `<span class="agent-cell"><span class="legend-dot" style="background:${color}"></span>${val}</span>`
        : (bad ? `<span class="bad-val">${val}</span>` : val);
      html += `<td>${disp}</td>`;
    });
    html += '</tr>';
  });
  html += '</tbody></table>';
  el.innerHTML = html;
}

function renderEvents(d) {
  const el = document.getElementById('events');
  const evs = d.events || [];
  if (!evs.length) { el.innerHTML = '<span class="muted">无</span>'; return; }
  el.innerHTML = evs.map(e =>
    `<div class="ev ${e.type.includes('Crash') ? 'crash' : ''}"><span class="ev-t">${(e.t ?? 0).toFixed(1)}s</span>${e.type}: ${escapeHtml(e.detail || '')}</div>`
  ).join('');
}

function renderLegend(d) {
  const el = document.getElementById('legend');
  const items = d.agents.map(a => `<div class="legend-item"><span class="legend-dot" style="background:${a.color}"></span>Agent ${a.id} · ${a.model}</div>`);
  items.push(`<div class="legend-item"><span class="legend-dot" style="background:#${COLORS.obstacle.toString(16).padStart(6,'0')}"></span>静态障碍</div>`);
  items.push(`<div class="legend-item"><span class="legend-dot" style="background:#${COLORS.wind.toString(16).padStart(6,'0')}"></span>风场方向</div>`);
  items.push(`<div class="legend-item"><span class="legend-dot" style="background:#4f8cff"></span>优化轨迹(未来)</div>`);
  items.push(`<div class="legend-item"><span class="legend-dot" style="background:#4f8cff;opacity:.5"></span>规划路径(未来)</div>`);
  items.push(`<div class="legend-item"><span class="legend-dot" style="background:#ff922b"></span>NMPC预测(未来)</div>`);
  el.innerHTML = items.join('');
}

function renderWindLabel(d) {
  const el = document.getElementById('wind-label');
  const s = d.wind?.samples;
  if (!s?.length) { el.textContent = ''; return; }
  const last = s[s.length - 1];
  el.textContent = `🌬 风 ${last.speedMs} m/s · ${last.dirDeg.toFixed(0)}°`;
}

function setStatus(text, live) {
  const el = document.getElementById('status');
  el.textContent = text;
  el.classList.toggle('live', !!live);
}

function escapeHtml(s) { return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }

// ---------------------------------------------------------------------------
// 图表(Canvas 自绘)
// ---------------------------------------------------------------------------
function ensureChart(id) {
  const canvas = document.getElementById(id);
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr; canvas.height = h * dpr;
  }
  return { canvas, ctx: canvas.getContext('2d'), w, h, dpr };
}

function renderCharts(d) {
  drawChart('chart-speed', d.series.speedMs, '#4f8cff');
  drawChart('chart-alt', d.series.altM, '#22b8cf');
  drawChart('chart-clear', d.series.clearanceM, '#fcc419', d.verdict?.clearanceM);
}

function drawChart(id, series, baseColor, hline) {
  const { ctx, w, h, dpr } = ensureChart(id);
  ctx.save();
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, w, h);
  // 边框背景
  ctx.fillStyle = '#0a0d18';
  ctx.fillRect(0, 0, w, h);

  const padL = 28, padR = 6, padT = 6, padB = 14;
  const plotW = w - padL - padR, plotH = h - padT - padB;

  // 数据范围
  let tMax = App.durationSec || 1;
  let yMax = 0.1, yMin = 0;
  series.forEach(s => s.points.forEach(([t, v]) => {
    if (v != null) { if (v > yMax) yMax = v; if (v < yMin) yMin = v; }
  }));
  if (hline != null && hline > yMax) yMax = hline;
  if (yMin > 0) yMin = 0;
  yMax = yMax * 1.15 + 0.01;

  // 网格 + Y 轴刻度
  ctx.strokeStyle = '#2c3552'; ctx.fillStyle = '#8b95b8'; ctx.lineWidth = 1;
  ctx.font = '9px sans-serif'; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (let i = 0; i <= 4; i++) {
    const y = padT + (plotH * i / 4);
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
    const val = yMax - (yMax - yMin) * (i / 4);
    ctx.fillText(fmt(val), padL - 3, y);
  }

  // 游标竖线
  const cx = padL + (App.cursorT / tMax) * plotW;
  ctx.strokeStyle = '#4f8cff'; ctx.globalAlpha = 0.5;
  ctx.beginPath(); ctx.moveTo(cx, padT); ctx.lineTo(cx, padT + plotH); ctx.stroke();
  ctx.globalAlpha = 1;

  // 阈值线
  if (hline != null) {
    const hy = padT + plotH * (1 - (hline - yMin) / (yMax - yMin));
    ctx.strokeStyle = '#ff6b6b'; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(padL, hy); ctx.lineTo(w - padR, hy); ctx.stroke();
    ctx.setLineDash([]);
  }

  // 曲线
  series.forEach(s => {
    ctx.strokeStyle = s.color || baseColor; ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    s.points.forEach(([t, v]) => {
      if (v == null) return;
      const x = padL + (t / tMax) * plotW;
      const y = padT + plotH * (1 - (v - yMin) / (yMax - yMin));
      if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });

  ctx.restore();
}

function fmt(v) {
  if (Math.abs(v) >= 100) return v.toFixed(0);
  if (Math.abs(v) >= 10) return v.toFixed(1);
  return v.toFixed(2);
}

// ---------------------------------------------------------------------------
// 时间轴 / 回放
// ---------------------------------------------------------------------------
function bindUI() {
  const scrub = document.getElementById('scrub');
  const playBtn = document.getElementById('play-btn');
  scrub.addEventListener('input', () => {
    App.playing = false;
    App.cursorT = (scrub.value / 1000) * App.durationSec;
    playBtn.textContent = '▶';
    updateTimelineUI();
  });
  playBtn.addEventListener('click', () => {
    App.playing = !App.playing;
    playBtn.textContent = App.playing ? '⏸' : '▶';
  });
  // 未来轨迹显示开关
  const fmap = { 'toggle-opt': 'opt', 'toggle-plan': 'plan', 'toggle-nmpc': 'nmpc' };
  Object.keys(fmap).forEach(id => {
    const cb = document.getElementById(id);
    if (cb) {
      cb.checked = App.showFuture[fmap[id]];
      cb.addEventListener('change', () => {
        App.showFuture[fmap[id]] = cb.checked;
        if (App.data) rebuildScene(App.data);
      });
    }
  });
}

function updateTimelineUI() {
  const scrub = document.getElementById('scrub');
  if (App.durationSec > 0) scrub.value = Math.round((App.cursorT / App.durationSec) * 1000);
  document.getElementById('time-label').textContent = `${App.cursorT.toFixed(1)}s / ${App.durationSec.toFixed(1)}s`;
}

// ---------------------------------------------------------------------------
// 主循环
// ---------------------------------------------------------------------------
function loop() {
  requestAnimationFrame(loop);
  const now = performance.now();
  const dt = (now - lastTs) / 1000;
  lastTs = now;

  // 实时且播放时游标跟随末尾;非实时回放时按真实时间推进
  const live = App.data && !App.data.finished;
  if (!live && App.playing) {
    App.cursorT += dt;
    if (App.cursorT > App.durationSec) App.cursorT = App.durationSec;
  }

  App.controls.update();
  updateAgentPose();
  // 回放时实时刷新图表游标(避免每帧都重绘图表太重,降频到 ~15fps)
  chartAccum += dt;
  if (chartAccum > 0.066 && App.data) {
    chartAccum = 0;
    if (live && App.playing) { /* 实时由 onNewData 刷新,这里只刷游标 } */ }
    drawChartCursorOnly();
  }
  updateTimelineUI();
  App.renderer.render(App.scene, App.camera);
}

// 仅重画图表上的游标竖线(轻量),完整重绘交给 onNewData
function drawChartCursorOnly() {
  if (!App.data) return;
  ['speed', 'alt', 'clear'].forEach(k => {
    const id = 'chart-' + k;
    const seriesMap = { speed: 'speedMs', alt: 'altM', clear: 'clearanceM' };
    const series = App.data.series[seriesMap[k]];
    const hline = k === 'clear' ? App.data.verdict?.clearanceM : null;
    drawChart(id, series, '#4f8cff', hline);
  });
}
