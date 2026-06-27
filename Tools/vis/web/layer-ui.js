import { App } from './app.js';
import { log } from './logger.js';

const KNOWN_LAYERS = [
  'path', 'trajectory', 'tracking', 'obstacle', 'search_bounds', 'waypoint', 'nmpc',
  'body_axes', 'velocity', 'history_trail', 'tracking_alt', 'label', 'obstacle_mgr',
  'planner_path', 'sensor_traces', 'sensor_points', 'sensor_detected', 'hud_stability',
];

const LAYER_LABELS = {
  path: '规划路径', trajectory: '优化轨迹', tracking: '跟踪点', obstacle: '障碍',
  search_bounds: '搜索边界', waypoint: '航点', nmpc: 'NMPC预测',
  body_axes: '机体坐标', velocity: '速度向量', history_trail: '历史轨迹',
  tracking_alt: '跟踪期望', label: '文本标注', obstacle_mgr: '障碍管理器',
  planner_path: 'A*路径', sensor_traces: '射线检测', sensor_points: '点云',
  sensor_detected: '检测障碍', hud_stability: '稳定性HUD',
};

export function initLayerUI() {
  const panel = document.getElementById('layer-panel');
  if (!panel) return;

  const saved = loadSaved();
  for (const layer of KNOWN_LAYERS) {
    if (!(layer in App.layerVisible)) App.layerVisible[layer] = true;
  }
  Object.assign(App.layerVisible, saved);

  panel.innerHTML = '';

  const master = document.createElement('label');
  master.className = 'layer-master';
  master.innerHTML = '<input type="checkbox" id="layer-all" checked> <b>全部分层</b>';
  panel.appendChild(master);

  const sep = document.createElement('div');
  sep.className = 'layer-sep';
  panel.appendChild(sep);

  for (const layer of KNOWN_LAYERS) {
    const label = document.createElement('label');
    label.className = 'layer-item';
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.checked = App.layerVisible[layer] !== false;
    cb.dataset.layer = layer;
    cb.addEventListener('change', () => {
      App.layerVisible[layer] = cb.checked;
      saveSaved();
      log.info('layer', '切换分层', { layer, visible: cb.checked });
      if (App.data) {
        import('./scene.js').then(m => m.rebuildScene(App.data));
      }
    });
    label.appendChild(cb);
    const swatch = document.createElement('span');
    swatch.className = 'layer-swatch';
    swatch.style.background = layerColor(layer);
    label.appendChild(swatch);
    const text = document.createElement('span');
    text.textContent = LAYER_LABELS[layer] || layer;
    label.appendChild(text);
    panel.appendChild(label);
  }

  document.getElementById('layer-all').addEventListener('change', (e) => {
    const on = e.target.checked;
    KNOWN_LAYERS.forEach(l => {
      App.layerVisible[l] = on;
      const cb = panel.querySelector(`input[data-layer="${l}"]`);
      if (cb) cb.checked = on;
    });
    saveSaved();
    if (App.data) {
      import('./scene.js').then(m => m.rebuildScene(App.data));
    }
  });
}

function layerColor(layer) {
  const colors = {
    path: '#00ff00', trajectory: '#4f8cff', tracking: '#ffff00', obstacle: '#ff0000',
    search_bounds: '#ffffff', waypoint: '#ff00ff', nmpc: '#ff922b',
    body_axes: '#ff6b6b', velocity: '#ffff00', history_trail: '#22b8cf',
    tracking_alt: '#ffff00', label: '#ffffff', obstacle_mgr: '#ff4444',
    planner_path: '#00ff00', sensor_traces: '#ff8800', sensor_points: '#ffcc00',
    sensor_detected: '#ff0000', hud_stability: '#88ff88',
  };
  return colors[layer] || '#888';
}

function loadSaved() {
  try { return JSON.parse(localStorage.getItem('vis_layer_visible') || '{}'); }
  catch { return {}; }
}

function saveSaved() {
  try { localStorage.setItem('vis_layer_visible', JSON.stringify(App.layerVisible)); }
  catch {}
}
