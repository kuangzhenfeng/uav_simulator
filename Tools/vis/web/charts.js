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
  const first = seriesArray[0];
  if (!first?.points?.length) return null;
  const x = first.points.map(p => p[0]);
  return [x, ...seriesArray.map(s => s.points.map(p => p[1]))];
}

function windToUPlot(wind) {
  if (!wind?.samples?.length) return null;
  const xs = wind.samples.map(s => s.t);
  const ys = wind.samples.map(s => s.speedMs);
  return [xs, ys];
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

  // Minimal initial data so uPlot doesn't error
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
      if (cur?.uplot) cur.uplot.setData(uData, false);
    }
  }

  // --- Wind chart ---
  {
    const def = CHART_DEFS.find(d => d.key === 'wind');
    const chartState = charts.wind;
    if (def && chartState?.uplot) {
      const uData = windToUPlot(data.wind);
      if (uData) chartState.uplot.setData(uData, false);
    }
  }
}

export { initCharts, updateCharts };
