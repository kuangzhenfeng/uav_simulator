// 前端日志模块
// 功能：分级日志 + 批量上报到后端 POST /api/log + localStorage 缓冲 + console 同步输出
// 使用: import { log } from './logger.js';
//       log.info('app', 'initialized', { key: val });
//       log.error('sse', 'parse failed', { data: str.slice(0, 100) });

const LEVELS = { debug: 0, info: 1, warn: 2, error: 3 };
const LEVEL_NAMES = ['debug', 'info', 'warn', 'error'];

const Config = {
  level: LEVELS.debug,        // 当前输出级别（console）
  reportIntervalMs: 5000,     // 上报间隔(ms)
  batchSize: 50,              // 批量大小
  maxBuffer: 500,             // localStorage 最大缓冲条数
  enabled: true,              // 总开关
};

// 从 localStorage 读取覆盖配置
try {
  const saved = localStorage.getItem('vis_log_config');
  if (saved) Object.assign(Config, JSON.parse(saved));
} catch (e) {}

function saveConfig() {
  try { localStorage.setItem('vis_log_config', JSON.stringify(Config)); } catch (e) {}
}

const buffer = [];
let flushTimer = null;

function now() {
  return new Date().toISOString();
}

function makeEntry(levelNum, module, msg, extra) {
  const entry = {
    ts: now(),
    level: LEVEL_NAMES[levelNum] || 'info',
    module: module || '',
    msg: typeof msg === 'object' ? JSON.stringify(msg) : String(msg),
  };
  if (extra) {
    try {
      entry.extra = typeof extra === 'object' ? JSON.stringify(extra).slice(0, 500) : String(extra).slice(0, 500);
    } catch (e) {
      entry.extra = '[非序列化]';
    }
  }
  return entry;
}

function enqueue(entry) {
  buffer.push(entry);
  // localStorage 持久化（页面刷新不丢）
  if (buffer.length <= Config.maxBuffer) {
    try {
      const stored = JSON.parse(localStorage.getItem('vis_log_buffer') || '[]');
      stored.push(entry);
      if (stored.length > Config.maxBuffer) stored.splice(0, stored.length - Config.maxBuffer);
      localStorage.setItem('vis_log_buffer', JSON.stringify(stored));
    } catch (e) {}
  }
}

function toConsole(entry) {
  const prefix = `[${entry.level.toUpperCase()}] [${entry.module}]`;
  const extraStr = entry.extra ? ' | ' + entry.extra : '';
  const full = `${prefix} ${entry.msg}${extraStr}`;
  if (entry.level === 'error') console.error(full);
  else if (entry.level === 'warn') console.warn(full);
  else if (entry.level === 'debug') console.debug(full);
  else console.info(full);
}

async function flush() {
  if (!buffer.length) return;
  const batch = buffer.splice(0, Config.batchSize);
  try {
    // navigator.sendBeacon 保证页面卸载时也能发出
    const payload = JSON.stringify(batch);
    if (navigator.sendBeacon) {
      navigator.sendBeacon('/api/log', new Blob([payload], { type: 'application/json' }));
    } else {
      await fetch('/api/log', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: payload,
        keepalive: true,
      });
    }
  } catch (e) {
    // 上报失败不丢缓冲，放回队首
    buffer.unshift(...batch);
  }
}

// 启动定时上报
if (Config.enabled) {
  flushTimer = setInterval(flush, Config.reportIntervalMs);
  // 页面卸载时 flush
  window.addEventListener('beforeunload', flush);
  // 页面可见性变化时 flush
  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'hidden') flush();
  });
}

// 启动时先 flush 一次之前的缓冲
(async () => {
  try {
    const stored = JSON.parse(localStorage.getItem('vis_log_buffer') || '[]');
    if (stored.length) {
      buffer.push(...stored.slice(0, Config.maxBuffer));
      localStorage.removeItem('vis_log_buffer');
      await flush();
    }
  } catch (e) {}
})();

function emit(levelNum, module, msg, extra) {
  if (!Config.enabled) return;
  if (levelNum < Config.level) return;
  const entry = makeEntry(levelNum, module, msg, extra);
  toConsole(entry);
  enqueue(entry);
  // 达到批大小立即上报
  if (buffer.length >= Config.batchSize) flush();
}

export const log = {
  debug: (module, msg, extra) => emit(LEVELS.debug, module, msg, extra),
  info:  (module, msg, extra) => emit(LEVELS.info, module, msg, extra),
  warn:  (module, msg, extra) => emit(LEVELS.warn, module, msg, extra),
  error: (module, msg, extra) => emit(LEVELS.error, module, msg, extra),

  // 运行时配置接口
  setLevel(levelName) {
    Config.level = LEVELS[levelName] ?? LEVELS.debug;
    saveConfig();
  },
  getLevel() { return LEVEL_NAMES[Config.level]; },
  setEnabled(v) { Config.enabled = !!v; saveConfig(); },
  flush() { return flush(); },
  getStats() { return { buffered: buffer.length, config: { ...Config } }; },

  // 清除本地缓冲
  clearBuffer() {
    buffer.length = 0;
    try { localStorage.removeItem('vis_log_buffer'); } catch (e) {}
  },
};

// 暴露到 window 供 DevTools 调试
if (typeof window !== 'undefined') {
  window.__logger = log;
}
