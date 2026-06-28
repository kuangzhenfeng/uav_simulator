import * as THREE from 'three';
import { App } from './app.js';

const LAYER_COLORS = {
  path: 0x00ff00, trajectory: 0x4f8cff, tracking: 0xffff00, obstacle: 0xff0000,
  search_bounds: 0xffffff, waypoint: 0xff00ff, nmpc: 0xff922b,
  body_axes: 0xff6b6b, velocity: 0xffff00, history_trail: 0x22b8cf,
  tracking_alt: 0xffff00, label: 0xffffff, obstacle_mgr: 0xff4444,
  planner_path: 0x00ff00, sensor_traces: 0xff8800, sensor_points: 0xffcc00,
  sensor_detected: 0xff0000, hud_stability: 0x88ff88,
};

// 这些 layer 的原语位置都派生自飞机当前位置 (机体轴/速度向量/HUD 文本/误差标签),
// 应当绑到对应 agent.group 下随飞机移动; 其他 layer (路径/障碍/航迹等) 是世界坐标, 不绑.
// hud_stability 由专用 HUD 渲染器(updateHudLabels)处理, 不走通用文本精灵路径.
const AGENT_BOUND_LAYERS = new Set(['body_axes', 'velocity', 'hud_stability', 'label']);
const HUD_LAYER = 'hud_stability';

// world-space 原语指纹缓存: layer -> Map(fingerprint -> node)。
// agent-bound 原语(机体轴/速度/HUD)挂 agent.group 随飞机移动, 不缓存, 每帧重建(数量少)。
const nodeCache = new Map();

function disposeNode(node) {
  node.traverse(c => {
    c.geometry?.dispose?.();
    c.material?.map?.dispose?.();
    c.material?.dispose?.();
  });
}

export function clearDebugCache() {
  for (const [, layerMap] of nodeCache) layerMap.clear();
  nodeCache.clear();
}

function fmtCoord(arr) {
  return Array.isArray(arr) ? arr.join(',') : String(arr);
}

function primFingerprint(p) {
  switch (p.type) {
    case 'line': return `L|${fmtCoord(p.a)}|${fmtCoord(p.b)}|${p.color}|${p.thickness || 1}`;
    case 'sphere': return `S|${fmtCoord(p.pos)}|${p.radius}|${p.color}`;
    case 'arrow': return `A|${fmtCoord(p.a)}|${fmtCoord(p.b)}|${p.size}|${p.color}`;
    case 'box': return `B|${fmtCoord(p.pos)}|${fmtCoord(p.extents)}|${fmtCoord(p.quat)}|${p.color}`;
    case 'point': return `P|${fmtCoord(p.pos)}|${p.size}|${p.color}`;
    case 'text': return `T|${fmtCoord(p.pos)}|${p.text}|${p.color}`;
    default: return null;
  }
}

export function rebuildDebug(debugData) {
  for (const [, obj] of App.agentObjects) {
    const grp = obj.group;
    for (let i = grp.children.length - 1; i >= 0; i--) {
      if (grp.children[i].userData?.agentBoundDebug) grp.remove(grp.children[i]);
    }
  }

  const presentLayers = new Set();

  for (const [layer, entries] of Object.entries(debugData)) {
    if (layer === HUD_LAYER) continue;
    presentLayers.add(layer);
    if (!App.layerVisible[layer]) continue;

    const isAgentBound = AGENT_BOUND_LAYERS.has(layer);
    let layerMap = null;
    let group = null;
    if (!isAgentBound) {
      layerMap = nodeCache.get(layer);
      if (!layerMap) { layerMap = new Map(); nodeCache.set(layer, layerMap); }
      group = App.debugGroups.get(layer);
      if (!group) {
        group = new THREE.Group();
        group.userData.layer = layer;
        App.scene.add(group);
        App.debugGroups.set(layer, group);
      }
    }

    const seen = new Set();
    for (const entry of entries) {
      const agentObj = isAgentBound && entry.agent != null && entry.agent >= 0
        ? App.agentObjects.get(entry.agent) : null;

      if (agentObj) {
        const anchor = getAgentAnchorPos(entry.agent);
        const shifted = shiftPrimCoords(entry.prim, anchor);
        const node = renderPrim(shifted);
        if (node) {
          node.userData.agentBoundDebug = true;
          agentObj.group.add(node);
        }
        continue;
      }

      if (!layerMap) continue;
      const key = primFingerprint(entry.prim);
      if (key == null) continue;
      seen.add(key);
      if (layerMap.has(key)) continue;
      const node = renderPrim(entry.prim);
      if (node) {
        group.add(node);
        layerMap.set(key, node);
      }
    }

    if (layerMap) {
      for (const [key, node] of layerMap) {
        if (!seen.has(key)) {
          group.remove(node);
          disposeNode(node);
          layerMap.delete(key);
        }
      }
    }
  }

  for (const layer of [...nodeCache.keys()]) {
    if (!presentLayers.has(layer) || !App.layerVisible[layer]) {
      const group = App.debugGroups.get(layer);
      if (group) {
        App.scene.remove(group);
        App.debugGroups.delete(layer);
      }
      const layerMap = nodeCache.get(layer);
      for (const [, node] of layerMap) disposeNode(node);
      layerMap.clear();
      nodeCache.delete(layer);
    }
  }
}

// 注意: rebuildScene 时 group.position 还是 initPos, 真实当前位置在 trace 末端.
// 用 trace 末端算偏移, 避免 initPos ≠ 当前位置导致 offset 错位.
function getAgentAnchorPos(agentId) {
  const agent = App.data?.agents?.find(a => a.id === agentId);
  const trace = agent?.trace;
  if (trace && trace.length) {
    return new THREE.Vector3().fromArray(trace[trace.length - 1].pos);
  }
  const obj = App.agentObjects.get(agentId);
  return obj ? obj.group.position.clone() : new THREE.Vector3();
}

// 把 prim 的所有世界坐标字段减去 anchor, 转成相对 agent 的本地坐标.
// 旋转/缩放字段 (quat) 不变, 仅线性平移 pos/a/b.
function shiftPrimCoords(prim, anchor) {
  const out = { ...prim };
  if (Array.isArray(prim.pos)) {
    out.pos = [prim.pos[0] - anchor.x, prim.pos[1] - anchor.y, prim.pos[2] - anchor.z];
  }
  if (Array.isArray(prim.a)) {
    out.a = [prim.a[0] - anchor.x, prim.a[1] - anchor.y, prim.a[2] - anchor.z];
  }
  if (Array.isArray(prim.b)) {
    out.b = [prim.b[0] - anchor.x, prim.b[1] - anchor.y, prim.b[2] - anchor.z];
  }
  return out;
}

function renderPrim(prim) {
  switch (prim.type) {
    case 'sphere': return renderSphere(prim);
    case 'line': return renderLine(prim);
    case 'arrow': return renderArrow(prim);
    case 'box': return renderBox(prim);
    case 'point': return renderPoint(prim);
    case 'text': return renderText(prim);
    default: return null;
  }
}

function renderSphere(prim) {
  const r = Math.max(prim.radius, 0.01);
  const geo = new THREE.SphereGeometry(r, 12, 10);
  const mat = new THREE.MeshBasicMaterial({ color: prim.color, wireframe: true, transparent: true, opacity: 0.6 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(prim.pos[0], prim.pos[1], prim.pos[2]);
  return mesh;
}

function renderLine(prim) {
  const geo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(prim.a[0], prim.a[1], prim.a[2]),
    new THREE.Vector3(prim.b[0], prim.b[1], prim.b[2]),
  ]);
  const mat = new THREE.LineBasicMaterial({ color: prim.color });
  return new THREE.Line(geo, mat);
}

function renderArrow(prim) {
  const grp = new THREE.Group();
  const start = new THREE.Vector3(prim.a[0], prim.a[1], prim.a[2]);
  const end = new THREE.Vector3(prim.b[0], prim.b[1], prim.b[2]);
  const dir = end.clone().sub(start);
  const len = dir.length();
  if (len < 0.01) return grp;
  const headLen = Math.min(prim.size, len * 0.3);
  const shaftLen = Math.max(len - headLen, 0.01);
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(0.02, 0.02, shaftLen, 6),
    new THREE.MeshBasicMaterial({ color: prim.color })
  );
  shaft.position.copy(start).add(dir.clone().multiplyScalar(shaftLen / 2 / len));
  alignToDir(shaft, dir);
  grp.add(shaft);
  const head = new THREE.Mesh(
    new THREE.ConeGeometry(prim.size * 0.3, headLen, 8),
    new THREE.MeshBasicMaterial({ color: prim.color })
  );
  head.position.copy(start).add(dir.clone().multiplyScalar((shaftLen + headLen / 2) / len));
  alignToDir(head, dir);
  grp.add(head);
  return grp;
}

function alignToDir(obj, dir) {
  const axis = new THREE.Vector3(0, 1, 0);
  const q = new THREE.Quaternion().setFromUnitVectors(axis, dir.clone().normalize());
  obj.quaternion.copy(q);
}

function renderBox(prim) {
  const geo = new THREE.BoxGeometry(prim.extents[0] * 2, prim.extents[1] * 2, prim.extents[2] * 2);
  const edges = new THREE.EdgesGeometry(geo);
  const mat = new THREE.LineBasicMaterial({ color: prim.color });
  const mesh = new THREE.LineSegments(edges, mat);
  mesh.position.set(prim.pos[0], prim.pos[1], prim.pos[2]);
  const q = prim.quat || [0, 0, 0, 1];
  mesh.quaternion.set(q[0], q[1], q[2], q[3]);
  return mesh;
}

function renderPoint(prim) {
  const geo = new THREE.SphereGeometry(Math.max(prim.size, 0.02), 6, 5);
  const mat = new THREE.MeshBasicMaterial({ color: prim.color });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(prim.pos[0], prim.pos[1], prim.pos[2]);
  return mesh;
}

function renderText(prim) {
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  ctx.font = '48px sans-serif';
  const w = ctx.measureText(prim.text).width + 20;
  canvas.width = w; canvas.height = 64;
  ctx.font = '48px sans-serif';
  ctx.fillStyle = prim.color;
  ctx.textBaseline = 'middle';
  ctx.fillText(prim.text, 10, 32);
  const tex = new THREE.CanvasTexture(canvas);
  const mat = new THREE.SpriteMaterial({ map: tex, transparent: true });
  const sprite = new THREE.Sprite(mat);
  sprite.position.set(prim.pos[0], prim.pos[1] + 0.5, prim.pos[2]);
  sprite.scale.set(w / 64 * 0.8, 0.8, 1);
  return sprite;
}

// ---------------- 稳定性 HUD 浮动文本 (专用渲染器) ----------------
// 旧实现把 hud_stability 当作每帧覆盖的瞬时 layer, 回放期间文本冻结在最后一帧;
// 这里改为按 cursorT 在 hudHistory 里采样, 让回放/拖拽时数值随时间变化.
// 同时把三行文本画进同一张带圆角背景的 canvas, 解决行间距过密的问题.

const HUD_FONT = '600 26px "PingFang SC", "Microsoft YaHei", sans-serif';
const HUD_LINE_H = 34;
const HUD_PAD = 12;
const HUD_VIS_OFFSET_Y = 2.2;   // 浮在飞机上方 (米)
const HUD_PANEL_W = 2.6;        // 面板世界宽度 (米), 高度按宽高比缩放

function scoreColor(score) {
  if (score == null || !isFinite(score)) return '#e6ebff';
  if (score >= 90) return '#88ff88';
  if (score >= 75) return '#fcc419';
  return '#ff6b6b';
}

function sampleHudAt(series, t) {
  if (!series || !series.length) return null;
  if (t <= series[0].t) return series[0];
  if (t >= series[series.length - 1].t) return series[series.length - 1];
  let lo = 0, hi = series.length - 1;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (series[mid].t < t) lo = mid + 1;
    else hi = mid - 1;
  }
  const a = series[Math.max(0, lo - 1)], b = series[lo];
  if (!b) return a;
  const k = (t - a.t) / Math.max(b.t - a.t, 1e-6);
  return k < 0.5 ? a : b;
}

function drawHudCanvas(lines, score) {
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  ctx.font = HUD_FONT;
  let maxW = 0;
  for (const ln of lines) {
    if (!ln) continue;
    maxW = Math.max(maxW, ctx.measureText(ln).width);
  }
  const w = Math.ceil(maxW) + HUD_PAD * 2 + 8;
  const h = HUD_LINE_H * lines.length + HUD_PAD * 2;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  ctx.scale(dpr, dpr);
  const r = 10;
  ctx.fillStyle = 'rgba(15, 19, 32, 0.86)';
  ctx.strokeStyle = scoreColor(score);
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(r, 1);
  ctx.arcTo(w - 1, 1, w - 1, r, r);
  ctx.arcTo(w - 1, h - 1, w - r, h - 1, r);
  ctx.arcTo(1, h - 1, 1, h - r, r);
  ctx.arcTo(1, 1, r, 1, r);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.font = HUD_FONT;
  ctx.textBaseline = 'middle';
  lines.forEach((ln, i) => {
    if (!ln) return;
    const isHeader = i === 0;
    ctx.fillStyle = isHeader ? scoreColor(score) : '#cfd6f5';
    ctx.fillText(ln, HUD_PAD, HUD_PAD + i * HUD_LINE_H + HUD_LINE_H / 2);
  });
  return { canvas, w, h };
}

export function updateHudLabels() {
  if (!App.scene) return;
  const visible = App.layerVisible[HUD_LAYER] !== false;
  for (const [agentId, series] of App.hudSeries) {
    const agentObj = App.agentObjects.get(agentId);
    let sprite = App.hudSprites.get(agentId);
    if (!visible || !agentObj) {
      if (sprite) sprite.visible = false;
      continue;
    }
    const frame = sampleHudAt(series, App.cursorT);
    if (!frame || !frame.lines || !frame.lines.some(l => l)) {
      if (sprite) sprite.visible = false;
      continue;
    }
    const fingerprint = frame.t + '|' + (frame.score ?? '');
    if (!sprite) {
      const mat = new THREE.SpriteMaterial({ transparent: true, depthTest: false });
      sprite = new THREE.Sprite(mat);
      sprite.renderOrder = 999;
      sprite.userData.hudAgent = agentId;
      agentObj.group.add(sprite);
      App.hudSprites.set(agentId, sprite);
    }
    sprite.visible = true;
    if (sprite.userData.fp !== fingerprint) {
      sprite.userData.fp = fingerprint;
      const tex = sprite.material.map;
      const { canvas, w, h } = drawHudCanvas(frame.lines, frame.score);
      const newTex = new THREE.CanvasTexture(canvas);
      newTex.minFilter = THREE.LinearFilter;
      sprite.material.map = newTex;
      sprite.material.needsUpdate = true;
      if (tex) tex.dispose();
      const ar = h / w;
      sprite.scale.set(HUD_PANEL_W, HUD_PANEL_W * ar, 1);
    }
    sprite.position.set(0, HUD_VIS_OFFSET_Y, 0);
  }
  for (const [agentId, sprite] of App.hudSprites) {
    if (!App.agentObjects.has(agentId)) {
      disposeHudSprite(sprite);
      App.hudSprites.delete(agentId);
    }
  }
}

function disposeHudSprite(sprite) {
  if (!sprite) return;
  sprite.parent?.remove(sprite);
  sprite.material?.map?.dispose();
  sprite.material?.dispose?.();
}

export function clearHudSprites() {
  for (const [, sprite] of App.hudSprites) disposeHudSprite(sprite);
  App.hudSprites.clear();
}
