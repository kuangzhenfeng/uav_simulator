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

export function rebuildDebug(debugData) {
  for (const [, group] of App.debugGroups) {
    App.scene.remove(group);
    group.traverse(c => {
      c.geometry?.dispose?.();
      c.material?.map?.dispose?.();
      c.material?.dispose?.();
    });
  }
  App.debugGroups.clear();

  for (const [layer, entries] of Object.entries(debugData)) {
    if (!App.layerVisible[layer]) continue;
    const group = new THREE.Group();
    group.userData.layer = layer;
    for (const entry of entries) {
      const mesh = renderPrim(entry.prim);
      if (mesh) group.add(mesh);
    }
    App.scene.add(group);
    App.debugGroups.set(layer, group);
  }
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
