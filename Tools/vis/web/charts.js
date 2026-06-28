// charts.js — uPlot time-series charts for UAV vis panel
// Uses window.uPlot from IIFE build (vendor/uPlot.iife.min.js)

const DARK = {
  muted: '#8b95b8',
  border: '#2c3552',
  bg: '#0f1320',
};

const CHART_DEFS = [
  { id: 'chart-speed',     key: 'speedMs',    title: '速度 SPEED (M/S)' },
  { id: 'chart-alt',       key: 'altM',       title: '高度 ALT (M)' },
  { id: 'chart-clearance', key: 'clearanceM',  title: '净空 CLEARANCE (M)' },
  { id: 'chart-wind',      key: 'wind',        title: '风速 WIND (M/S)' },
];

// Module state
const charts = {};      // { speed: { uplot, container, agentCount } }

// -- Data transformation helpers --

function seriesToUPlot(seriesArray) {
  if (!seriesArray?.length) return null;
  const cleaned = seriesArray
    .map(s => ({ ...s, points: sortUniquePoints(s.points) }))
    .filter(s => s.points.length);
  if (!cleaned.length) return null;
  const x = sortUniquePoints(cleaned.flatMap(s => s.points)).map(p => p[0]);
  return [x, ...cleaned.map(s => {
    const values = new Map(s.points.map(p => [p[0], p[1]]));
    return x.map(t => values.get(t) ?? null);
  })];
}

function windToUPlot(wind) {
  const points = sortUniquePoints(wind?.samples?.map(s => [s.t, s.speedMs]));
  if (!points.length) return null;
  const xs = points.map(p => p[0]);
  const ys = points.map(p => p[1]);
  return [xs, ys];
}

function sortUniquePoints(points) {
  if (!points?.length) return [];
  const sorted = [...points]
    .filter(p => Number.isFinite(p?.[0]) && Number.isFinite(p?.[1]))
    .sort((a, b) => a[0] - b[0]);
  const out = [];
  for (const point of sorted) {
    const prev = out[out.length - 1];
    if (!prev || point[0] > prev[0]) {
      out.push(point);
    } else {
      out[out.length - 1] = point;
    }
  }
  return out;
}

// -- uPlot axis config (shared across all 4 charts) --

function makeAxes() {
  return [
    {
      stroke: DARK.muted,
      grid: { stroke: DARK.border, width: 1 },
      ticks: { show: false },
      scale: 'x',
      values: (u, splits) => splits.map(v => v.toFixed(0)),
    },
    {
      stroke: DARK.muted,
      grid: { stroke: DARK.border, width: 1 },
      ticks: { show: false },
      scale: 'y',
      size: 36,
      values: (u, splits) => splits.map(v => v.toFixed(1)),
    },
  ];
}

function makeBaseOpts(container) {
  return {
    width: container.clientWidth || 300,
    height: 110,
    cursor: { drag: { x: true, y: false }, points: { show: false } },
    legend: { show: false },
    padding: [0, 0, 0, 0],
    axes: makeAxes(),
    series: [{}], // x-axis series only; y-series added on create
  };
}

// -- Create / recreate a single chart --

function createChart(def, agentCount, seriesColors) {
  const container = document.getElementById(def.id);
  if (!container) return null;

  // Destroy previous instance if exists
  if (charts[def.key]?.uplot) {
    try { charts[def.key].uplot.destroy(); } catch {}
    charts[def.key].uplot = null;
  }

  const opts = makeBaseOpts(container);

  if (def.key === 'wind') {
    // Single series for wind
    opts.series.push({
      label: 'Wind',
      stroke: '#4f8cff',
      width: 1.5,
      paths: window.uPlot.paths.linear(),
    });
  } else {
    // Multi-agent series
    for (let i = 0; i < agentCount; i++) {
      opts.series.push({
        label: `Agent ${i}`,
        stroke: seriesColors[i] || '#4f8cff',
        width: 1.5,
        paths: window.uPlot.paths.linear(),
      });
    }
  }

  // Minimal initial data so uPlot doesn't error.
  const seriesCount = def.key === 'wind' ? 1 : agentCount;
  const initData = [[0], ...Array.from({ length: seriesCount }, () => [0])];

  const uplot = new window.uPlot(opts, initData, container);

  charts[def.key] = { uplot, container, agentCount };
  return charts[def.key];
}

// -- Public API --

function initCharts() {
  if (!window.uPlot) {
    console.warn('[charts] uPlot not loaded — skipping chart init');
    return;
  }
  // Defer to next frame so DOM containers have layout width
  requestAnimationFrame(() => {
    for (const def of CHART_DEFS) {
      createChart(def, 1, ['#4f8cff']);
    }
  });
}

function updateCharts(data) {
  if (!data || !window.uPlot) return;

  const series = data.series;

  // --- Speed / Alt / Clearance charts ---
  for (const def of CHART_DEFS) {
    if (def.key === 'wind') continue;

    const chartState = charts[def.key];
    if (!chartState?.uplot) continue;

    const seriesArr = series?.[def.key];
    if (!seriesArr?.length) continue;

    const agentCount = seriesArr.length;
    const colors = seriesArr.map(s => s.color || '#4f8cff');

    // Recreate if agent count changed
    if (chartState.agentCount !== agentCount) {
      createChart(def, agentCount, colors);
    }

    const uData = seriesToUPlot(seriesArr);
    if (uData) {
      const cur = charts[def.key];
      // resetScales=true:让 uPlot 按新数据自动 fit 坐标轴范围。
      // 初始用空数组创建,scale 未确定;后续帧 x/y 范围也会随仿真推进变化,
      // 必须重置否则新数据落在旧 scale 外不可见。
      if (cur?.uplot) cur.uplot.setData(uData, true);
    }
  }

  // --- Wind chart ---
  {
    const def = CHART_DEFS.find(d => d.key === 'wind');
    const chartState = charts.wind;
    if (def && chartState?.uplot) {
      const uData = windToUPlot(data.wind);
      if (uData) chartState.uplot.setData(uData, true);
    }
  }
}

export { initCharts, updateCharts };
