import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { log } from './logger.js';
import { App } from './app.js';
import { rebuildDebug } from './debug-renderer.js';

// 视角跟随系统：追踪指定无人机的第三人称相机
let followTarget = null;      // 被跟随的无人机 agent ID
let followSmoothLerp = 0.12;   // 相机平滑系数（0-1）
let savedControlsTarget = new THREE.Vector3(); // 退出跟随前保存的 controls target

export function setFollowAgent(agentID) {
  if (!App.controls) return;

  if (agentID === followTarget) return;

  if (agentID === null) {
    App.controls.target.copy(savedControlsTarget);
    App.controls.enabled = true;
    log.debug('camera', '退出跟随模式');
  } else {
    if (followTarget === null) {
      savedControlsTarget.copy(App.controls.target);
    }
    App.controls.enabled = true;
    log.debug('camera', followTarget === null ? '进入跟随模式' : '切换跟随目标', { agentID });
  }

  followTarget = agentID;
}

export function getFollowAgent() {
  return followTarget;
}

export function updateFollowCamera() {
  if (!followTarget && followTarget !== 0) return;
  if (!App.controls || !App.camera) return;

  const agentObj = App.agentObjects.get(followTarget);
  if (!agentObj) return;

  const dronePos = agentObj.group.position;
  App.controls.target.lerp(dronePos, followSmoothLerp);
}

export function initThree() {
  const canvas = document.getElementById('three-canvas');
  const wrap = document.getElementById('scene-wrap');
  App.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  App.renderer.setPixelRatio(window.devicePixelRatio);
  App.renderer.setSize(wrap.clientWidth, wrap.clientHeight, false);

  App.scene = new THREE.Scene();
  App.scene.background = new THREE.Color(0x0a0d18);
  App.scene.fog = new THREE.Fog(0x0a0d18, 120, 400);

  App.camera = new THREE.PerspectiveCamera(55, wrap.clientWidth / wrap.clientHeight, 0.1, 2000);
  App.camera.position.set(40, 50, 30);

  App.controls = new OrbitControls(App.camera, canvas);
  App.controls.enableDamping = true;
  App.controls.target.set(0, 2, -40);

  const hemi = new THREE.HemisphereLight(0xbfd4ff, 0x202840, 0.9);
  App.scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 0.7);
  dir.position.set(50, 100, 40);
  App.scene.add(dir);

  rebuildGround([[-50, -50], [150, 50]]);
  window.addEventListener('resize', onResize);
}

function onResize() {
  const wrap = document.getElementById('scene-wrap');
  App.camera.aspect = wrap.clientWidth / wrap.clientHeight;
  App.camera.updateProjectionMatrix();
  App.renderer.setSize(wrap.clientWidth, wrap.clientHeight, false);
}

export function rebuildGround(bounds) {
  if (App.gridGroup) App.scene.remove(App.gridGroup);
  App.gridGroup = new THREE.Group();
  const [min, max] = bounds;
  const cx = (min[0] + max[0]) / 2, cz = (min[1] + max[1]) / 2;
  const size = Math.max(max[0] - min[0], max[1] - min[1], 40);

  const grid = new THREE.GridHelper(size, Math.ceil(size / 5), 0x2c3552, 0x2c3552);
  grid.position.set(cx, 0, cz);
  App.gridGroup.add(grid);

  const plane = new THREE.Mesh(
    new THREE.PlaneGeometry(size, size),
    new THREE.MeshBasicMaterial({ color: 0x1a2240, transparent: true, opacity: 0.5 })
  );
  plane.rotation.x = -Math.PI / 2;
  plane.position.set(cx, -0.02, cz);
  App.gridGroup.add(plane);
  App.scene.add(App.gridGroup);
}

export function rebuildScene(d) {
  const pts = [];
  d.agents.forEach(a => { if (a.initPos) pts.push(a.initPos); a.trace.forEach(t => pts.push(t.pos)); });
  d.obstacles.forEach(o => o.center && pts.push(o.center));
  d.waypoints.forEach(w => pts.push(w));
  if (pts.length) {
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (const [x, , z] of pts) {
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }
    const pad = 8;
    rebuildGround([[minX - pad, minZ - pad], [maxX + pad, maxZ + pad]]);
  }

  for (const [, obj] of App.agentObjects) {
    App.scene.remove(obj.group);
    obj.group.traverse(c => {
      c.geometry?.dispose?.();
      c.material?.map?.dispose?.();
      c.material?.dispose?.();
    });
  }
  App.agentObjects.clear();

  rebuildObstacles(d.obstacles || []);

  for (const [, group] of App.debugGroups) {
    App.scene.remove(group);
    group.traverse(c => {
      c.geometry?.dispose?.();
      c.material?.map?.dispose?.();
      c.material?.dispose?.();
    });
  }
  App.debugGroups.clear();

  const agentColors = ['#4f8cff', '#ff6b6b', '#51cf66', '#fcc419', '#cc5de8', '#22b8cf', '#ff922b', '#20c997'];
  d.agents.forEach((a, i) => {
    const color = a.color || agentColors[i % agentColors.length];
    const obj = makeAgent(color);
    obj.group.position.set(a.initPos ? a.initPos[0] : 0, a.initPos ? a.initPos[1] : 0, a.initPos ? a.initPos[2] : 0);
    App.scene.add(obj.group);
    App.agentObjects.set(a.id, obj);
  });

  rebuildDebug(d.debug || {});
  log.debug('scene', 'rebuildScene', { agents: d.agents.length, debugLayers: Object.keys(d.debug || {}).length });
}

function rebuildObstacles(obstacles) {
  if (App.obstacleGroup) {
    App.scene.remove(App.obstacleGroup);
    App.obstacleGroup.traverse(c => {
    c.geometry?.dispose?.();
    c.material?.map?.dispose?.();
    c.material?.dispose?.();
  });
  }
  App.obstacleGroup = new THREE.Group();
  App.obstacleGroup.visible = App.layerVisible.obstacle !== false;
  const obsColor = 0xff4444;
  for (const o of obstacles) {
    const mesh = makeObstacleMesh(o, obsColor);
    if (mesh) App.obstacleGroup.add(mesh);
  }
  App.scene.add(App.obstacleGroup);
}

function makeObstacleMesh(o, color) {
  if (!o.center) return null;
  const [cx, cy, cz] = o.center;
  const e = o.extents || [1, 1, 1];
  const mat = () => new THREE.MeshBasicMaterial({ color, wireframe: true, transparent: true, opacity: 0.7 });
  let mesh;
  switch (o.type) {
    case 'Sphere': {
      const r = Math.max(e[0], 0.05);
      mesh = new THREE.Mesh(new THREE.SphereGeometry(r, 14, 12), mat());
      break;
    }
    case 'Cylinder': {
      const r = Math.max(e[0], 0.05);
      const h = Math.max(e[1] * 2, 0.1);
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 16), mat());
      break;
    }
    case 'Box':
    default: {
      const w = Math.max(e[0] * 2, 0.1), h = Math.max(e[1] * 2, 0.1), d = Math.max(e[2] * 2, 0.1);
      const geo = new THREE.BoxGeometry(w, h, d);
      mesh = new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color }));
      break;
    }
  }
  mesh.position.set(cx, cy, cz);
  return mesh;
}

function makeAgent(color) {
  const grp = new THREE.Group();
  const body = new THREE.Group();
  const hub = new THREE.Mesh(
    new THREE.SphereGeometry(0.18, 12, 10),
    new THREE.MeshLambertMaterial({ color })
  );
  body.add(hub);
  const armLen = 0.55;
  for (let i = 0; i < 4; i++) {
    const ang = (Math.PI / 2) * i + Math.PI / 4;
    const arm = new THREE.Mesh(
      new THREE.CylinderGeometry(0.03, 0.03, armLen, 6),
      new THREE.MeshLambertMaterial({ color })
    );
    arm.rotation.z = Math.PI / 2;
    arm.rotation.y = ang;
    arm.position.set(Math.cos(ang) * armLen / 2, 0, Math.sin(ang) * armLen / 2);
    body.add(arm);
    const rotor = new THREE.Mesh(
      new THREE.CylinderGeometry(0.28, 0.28, 0.02, 12),
      new THREE.MeshLambertMaterial({ color, transparent: true, opacity: 0.4 })
    );
    rotor.position.set(Math.cos(ang) * armLen, 0.03, Math.sin(ang) * armLen);
    body.add(rotor);
  }
  grp.add(body);
  return { group: grp, body };
}

export function updateAgentPose() {
  if (!App.data) return;
  for (const [id, obj] of App.agentObjects) {
    const agent = App.data.agents.find(a => a.id === id);
    if (!agent) continue;
    const trace = agent.trace;
    if (!trace.length) continue;
    const p = sampleTrace(trace, App.cursorT);
    obj.group.position.set(p.pos[0], p.pos[1], p.pos[2]);
    const seg = traceSegmentDir(trace, App.cursorT);
    if (seg) {
      const yaw = Math.atan2(-seg[0], -seg[2]);
      obj.group.rotation.set(0, yaw, 0);
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
      return { pos: [a.pos[0]+(b.pos[0]-a.pos[0])*k, a.pos[1]+(b.pos[1]-a.pos[1])*k, a.pos[2]+(b.pos[2]-a.pos[2])*k] };
    }
  }
  return trace[trace.length - 1];
}

function traceSegmentDir(trace, t) {
  for (let i = 0; i < trace.length - 1; i++) {
    if (t >= trace[i].t && t <= trace[i+1].t) {
      return [trace[i+1].pos[0] - trace[i].pos[0], 0, trace[i+1].pos[2] - trace[i].pos[2]];
    }
  }
  return null;
}
