// UE 左手系(X前/Y右/Z上,cm) <-> web 右手系(Y上,m)
// 映射: web_x=Y_ue(右), web_y=Z_ue(上), web_z=-X_ue(远)
// 逆: X_ue=-web_z, Y_ue=web_x, Z_ue=web_y
function vecToUE(web) {
  if (!Array.isArray(web) || web.length < 3) return [0, 0, 0];
  return [-web[2] * 100, web[0] * 100, web[1] * 100];
}

function vecToWeb(ue) {
  if (!Array.isArray(ue) || ue.length < 3) return [0, 0, 0];
  return [ue[1] / 100, ue[2] / 100, -ue[0] / 100];
}

function lerp3(a, b, k) { return [a[0]+(b[0]-a[0])*k, a[1]+(b[1]-a[1])*k, a[2]+(b[2]-a[2])*k]; }

function sub3(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function len3(v) { return Math.hypot(v[0] || 0, v[1] || 0, v[2] || 0); }
function norm3(v) {
  const n = len3(v);
  return n > 1e-6 ? [v[0] / n, v[1] / n, v[2] / n] : [0, 0, 0];
}
function angleDiffDeg2D(a, b) {
  const an = norm3([a[0], 0, a[2]]);
  const bn = norm3([b[0], 0, b[2]]);
  if (len3(an) < 1e-6 || len3(bn) < 1e-6) return 0;
  const dot = Math.max(-1, Math.min(1, an[0] * bn[0] + an[2] * bn[2]));
  return Math.acos(dot) * 180 / Math.PI;
}
function round3n(v) { return Math.round(v * 1000) / 1000; }
function fmt3(v) { return Array.isArray(v) ? v.slice(0, 3).map(round3n) : v; }
function radToDeg(v) { return round3n(v * 180 / Math.PI); }

export { angleDiffDeg2D, fmt3, len3, lerp3, radToDeg, round3n, sub3, vecToUE, vecToWeb };
