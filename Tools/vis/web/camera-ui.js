import { App } from './app.js';
import { setFollowAgent, getFollowAgent } from './scene.js';
import { log } from './logger.js';

let cameraBtns = [];

export function initCameraUI() {
  const panel = document.getElementById('camera-panel');
  if (!panel) return;

  panel.innerHTML = '';

  const title = document.createElement('div');
  title.className = 'camera-title';
  title.textContent = 'Camera';
  panel.appendChild(title);

  const row = document.createElement('div');
  row.className = 'camera-btn-row';
  panel.appendChild(row);

  const globalBtn = document.createElement('button');
  globalBtn.className = 'camera-btn' + (getFollowAgent() === null ? ' active' : '');
  globalBtn.textContent = '全局';
  globalBtn.addEventListener('click', () => {
    setFollowAgent(null);
    updateHighlight(null);
    log.info('camera', '切换至全局视角');
  });
  row.appendChild(globalBtn);

  rebuildDroneButtons(row);
}

export function rebuildCameraUI() {
  if (!App.data || !App.data.agents) return;
  const panel = document.getElementById('camera-panel');
  if (!panel) return;

  const row = panel.querySelector('.camera-btn-row');
  if (!row) return;

  const oldBtns = row.querySelectorAll('.camera-drone-btn');
  oldBtns.forEach(b => b.remove());

  rebuildDroneButtons(row);
  updateHighlight(getFollowAgent());
}

function rebuildDroneButtons(row) {
  if (!App.data || !App.data.agents) return;

  App.data.agents.forEach((a, i) => {
    const btn = document.createElement('button');
    btn.className = 'camera-btn camera-drone-btn' + (getFollowAgent() === a.id ? ' active' : '');
    btn.textContent = '#' + a.id;
    btn.dataset.agentId = a.id;
    btn.addEventListener('click', () => {
      setFollowAgent(a.id);
      updateHighlight(a.id);
      log.info('camera', '切换至跟随视角', { agentID: a.id });
    });
    row.appendChild(btn);
  });
}

function updateHighlight(agentID) {
  const panel = document.getElementById('camera-panel');
  if (!panel) return;
  const btns = panel.querySelectorAll('.camera-btn');
  btns.forEach(btn => {
    if (btn.classList.contains('camera-drone-btn')) {
      const id = parseInt(btn.dataset.agentId, 10);
      btn.classList.toggle('active', id === agentID);
    } else {
      btn.classList.toggle('active', agentID === null);
    }
  });
}
