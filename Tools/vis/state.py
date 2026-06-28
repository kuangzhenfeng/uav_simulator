#!/usr/bin/env python3
import json
import logging
import math
import os
import threading
import time

from coords import dir_to_web, obstacle_type, size_web, to_web
from control_proxy import resolve_control_port

log = logging.getLogger("vis_server")

AGENT_COLORS = [
    "#4f8cff", "#ff6b6b", "#51cf66", "#fcc419",
    "#cc5de8", "#22b8cf", "#ff922b", "#20c997",
]

CLEARANCE_SENTINEL_MAX_CM = 1_000_000.0

MAX_TRACE_POINTS = 400
MAX_SERIES_POINTS = 300
MAX_HUD_POINTS = 150


def _downsample(seq, max_n):
    n = len(seq)
    if max_n <= 0 or n <= max_n:
        return seq
    step = (n - 1) / (max_n - 1)
    return [seq[0]] + [seq[int(round(i * step))] for i in range(1, max_n - 1)] + [seq[-1]]


def _clearance_m(clearance_cm):
    if clearance_cm is None:
        return None
    try:
        value = float(clearance_cm)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value) or value < 0 or value > CLEARANCE_SENTINEL_MAX_CM:
        return None
    return round(value / 100.0, 2)


def _series_pack(src, agents, max_points=0):
    out = []
    for a in agents:
        pts = src.get(a["id"], [])
        if pts:
            out.append({"agent": a["id"], "color": a["color"], "points": _downsample(pts, max_points)})
    return out


# ---------------------------------------------------------------------------
# scenario_result.json 读取(最终判决,覆盖逐秒推算)
# ---------------------------------------------------------------------------

def read_scenario_result(logs_dir):
    path = os.path.join(logs_dir, "scenario_result.json")
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        m = data.get("metrics", {})
        return {
            "final": data.get("verdict", "UNKNOWN"),
            "reached": f"{m.get('waypointsReached', 0)}/{m.get('waypointsTotal', 0)}",
            "clearanceM": _clearance_m(m.get("minClearanceCm")),
            "lateralDevM": round(m.get("maxLateralDevCm", 0) / 100.0, 2),
            "elapsedS": round(m.get("elapsedSec", 0), 2),
            "collided": bool(m.get("collided", False)),
            "failures": data.get("failures", []),
            "seed": data.get("seed"),
        }
    except (json.JSONDecodeError, OSError) as e:
        log.warning("read_scenario_result failed: %s", e)
        return None


# ---------------------------------------------------------------------------
# 目标数据源探测
# ---------------------------------------------------------------------------

def default_ndjson_path(project_root):
    return os.path.join(project_root, "Logs", "telemetry.ndjson")


def resolve_target(args, project_root):
    logs_dir = os.path.join(project_root, "Logs")
    if args.watch:
        return args.watch, logs_dir
    return default_ndjson_path(project_root), logs_dir


# ---------------------------------------------------------------------------
# NdjsonState: 累积式 ndjson 解析状态
# ---------------------------------------------------------------------------

class NdjsonState:
    """累积式 ndjson 解析状态。feed() 喂一行 JSON 对象,snapshot() 产出完整数据模型。"""

    def __init__(self):
        self.meta = None
        self.agent_meta = {}       # id -> {model, color, maxVelMs, collisionRadiusM, initPos}
        self.agent_order = []      # 首次 spawn 顺序,用于稳定配色
        self.traces = {}           # id -> [{t,pos,speed,ctrl}]
        self.speed_series = {}     # id -> [[t,v_ms]]
        self.alt_series = {}       # id -> [[t,alt_m]]
        self.clearance_series = {} # id -> [[t,v_m]]  (每帧最后一个值降噪)
        # 未来轨迹最新快照（覆盖式，区别于历史 traces 的累积式）
        # id -> {"opt":[[x,y,z],...], "plan":[[x,y,z],...], "nmpc":[[x,y,z,cost],...]}
        self.future = {}
        self.obstacles = {}        # id -> entry
        self.dynamic_count = 0
        self.waypoints = {}        # idx -> [x,y,z]
        self.wind_config = None
        self.wind_samples = []     # [{t,speedMs,dirDeg,vecMs}]
        self.verdict_timeline = [] # [{t,passed,reached,total,clearanceM,lateralDevM,elapsedS,final}]
        self.summary = {}          # id -> dict(取最后一次 metrics)
        self.events = []           # [{t,type,agent,detail}]
        self.reload_epoch = 0      # 收到 reload 标记自增，前端据此重置回放游标/清轨迹
        self.traj_first_logged = set()
        self.first_t = None
        self.last_t = None
        self.has_final = False
        # 单调时间轴保护: {(agent_id, epoch) -> last_t}。
        # reload epoch 推进时旧 key 自然失效(新 epoch 的 first frame 总是通过)。
        # 防止竞态/文件轮换误喂的回退帧污染 series -> uPlot 拒绝渲染。
        self._last_frame_t = {}
        # debug 原语: layer -> list of {t, prim(已 to_web 变换), expires_at}
        self.debug_prims = {}
        # 瞬时原语（d<=0 的 HUD 类文本，每帧覆盖）: layer -> (t, [entries])
        # UE 端 Duration=0 表示"本帧瞬时"，新帧到达时旧批整体替换。
        self.debug_transient = {}
        # 稳定性 HUD 历史快照: agent -> {t(float) -> {"lines":[str,str,str], "score":float}}
        # 每帧记录一次(已对多物理子步去重), 供前端回放时按 cursorT 采样,
        # 避免回放期间 HUD 文本停留在最后一帧(transient 只留最新一帧的缺陷)。
        self.hud_history = {}
        # 任务切割: 当前任务的元信息
        self.task_scenario = None
        self.task_started_at = None

    def _touch_t(self, t):
        if t is None:
            return
        if self.first_t is None:
            self.first_t = t
        self.last_t = t

    def _ensure_agent(self, aid):
        if aid not in self.agent_meta:
            self.agent_order.append(aid)
            color = AGENT_COLORS[(len(self.agent_order) - 1) % len(AGENT_COLORS)]
            self.agent_meta[aid] = {
                "model": "UAV", "color": color,
                "maxVelMs": None, "collisionRadiusM": 0.6, "initPos": None,
            }

    def feed(self, obj):
        typ = obj.get("type")
        t = obj.get("t")
        self._touch_t(t)

        handlers = {
            "meta": self._feed_meta,
            "reload": self._feed_reload,
            "wind_config": self._feed_wind_config,
            "spawn": self._feed_spawn,
            "obstacle": self._feed_obstacle,
            "waypoint": self._feed_waypoint,
            "frame": self._feed_frame,
            "metrics": self._feed_metrics,
            "event": self._feed_event,
            "verdict": self._feed_verdict,
            "debug": self._feed_debug,
            "traj_opt": self._feed_traj,
            "traj_plan": self._feed_traj,
            "traj_nmpc": self._feed_traj,
        }
        handler = handlers.get(typ)
        if handler:
            handler(obj, t)
        elif typ:
            log.warning("ndjson_unknown_type: %s", typ)

    def _feed_meta(self, obj, _t):
        self.meta = obj

    def _feed_reload(self, _obj, _t):
        self.reload_epoch += 1

    def _feed_wind_config(self, obj, _t):
        steady = obj.get("steady") or [0, 0, 0]
        self.wind_config = {
            "type": obj.get("windType"),
            "steadyMs": to_web(steady[0], steady[1], steady[2]),
        }

    def _feed_spawn(self, obj, _t):
        aid = obj.get("agent")
        pos = obj.get("pos") or [0, 0, 0]
        self._ensure_agent(aid)
        mv = obj.get("maxVelCm")
        self.agent_meta[aid].update({
            "model": obj.get("model") or "UAV",
            "maxVelMs": round(mv / 100.0, 2) if mv is not None else None,
            "collisionRadiusM": round(obj.get("collisionRadiusCm", 60) / 100.0, 2),
            "initPos": to_web(pos[0], pos[1], pos[2]),
        })

    def _feed_obstacle(self, obj, _t):
        oid = obj.get("id")
        center = obj.get("center") or [0, 0, 0]
        extents = obj.get("extents") or [0, 0, 0]
        entry = {
            "id": oid,
            "type": obstacle_type(obj.get("oType")),
            "center": to_web(center[0], center[1], center[2]),
            "extents": size_web(extents[0], extents[1], extents[2]),
            "actor": obj.get("actor", ""),
        }
        if obj.get("dynamic"):
            if oid not in self.obstacles or not self.obstacles[oid].get("_dyn"):
                self.dynamic_count += 1
                entry["_dyn"] = True
            self.obstacles[oid] = entry
        else:
            self.obstacles[oid] = entry

    def _feed_waypoint(self, obj, _t):
        idx = obj.get("idx")
        pos = obj.get("pos") or [0, 0, 0]
        self.waypoints[idx] = to_web(pos[0], pos[1], pos[2])

    def _feed_frame(self, obj, t):
        for agent in obj.get("agents", []):
            self._feed_frame_agent(agent, t)
        wind = obj.get("wind")
        if wind:
            self._push_wind(t, wind)

    def _feed_frame_agent(self, agent, t):
        aid = agent.get("id")
        self._ensure_agent(aid)
        if t is not None:
            frame_key = (aid, self.reload_epoch)
            last_t = self._last_frame_t.get(frame_key)
            if last_t is not None and t <= last_t:
                return
            self._last_frame_t[frame_key] = t
        pos_ue = agent.get("pos") or [0, 0, 0]
        vel_ue = agent.get("vel") or [0, 0, 0]
        pos = to_web(pos_ue[0], pos_ue[1], pos_ue[2])
        speed_ms = round(math.hypot(math.hypot(vel_ue[0], vel_ue[1]), vel_ue[2]) / 100.0, 2)
        self.traces.setdefault(aid, []).append({"t": t, "pos": pos, "speed": speed_ms, "ctrl": agent.get("ctrl", 0)})
        self.speed_series.setdefault(aid, []).append([t, speed_ms])
        self.alt_series.setdefault(aid, []).append([t, pos[1]])
        clearance = _clearance_m(agent.get("clearance"))
        if clearance is not None:
            self._append_clearance(aid, t, clearance)

    def _append_clearance(self, aid, t, near_m):
        series = self.clearance_series.setdefault(aid, [])
        if series and abs(series[-1][0] - t) < 0.5:
            series[-1][1] = near_m
        else:
            series.append([t, near_m])

    def _feed_metrics(self, obj, _t):
        aid = obj.get("agent")
        self.summary[aid] = {
            "agent": aid,
            "speedRatio": round(obj.get("speedRatio", 0), 3),
            "lowSpeedDur": round(obj.get("lowSpeedDur", 0), 2),
            "maxDev": round(obj.get("maxDev", 0), 2),
            "maxRoll": round(obj.get("maxRoll", 0), 2),
            "maxPitch": round(obj.get("maxPitch", 0), 2),
            "instabTime": round(obj.get("instabTime", 0), 2),
            "stuck": int(obj.get("stuck", 0)),
            "forceComplete": int(obj.get("forceComplete", 0)),
        }

    def _feed_event(self, obj, t):
        pos = obj.get("pos") or [0, 0, 0]
        web_pos = to_web(pos[0], pos[1], pos[2])
        self.events.append({
            "t": t,
            "type": obj.get("event", "Event"),
            "agent": obj.get("agent"),
            "detail": f"pos=({web_pos[0]:.1f},{web_pos[1]:.1f},{web_pos[2]:.1f}) crashed={bool(obj.get('crashed'))}",
        })

    def _feed_verdict(self, obj, t):
        clearance_cm = obj.get("clearanceCm")
        lateral_cm = obj.get("lateralDevCm")
        rec = {
            "t": t,
            "passed": bool(obj.get("passed")),
            "reached": obj.get("reached", 0),
            "total": obj.get("total", 0),
            "clearanceM": _clearance_m(clearance_cm),
            "lateralDevM": round(lateral_cm / 100.0, 2) if lateral_cm is not None else None,
            "elapsedS": round(obj.get("elapsedSec", 0), 2),
            "final": bool(obj.get("final")),
            "collided": bool(obj.get("collided", False)),
            "failures": obj.get("failures", []),
        }
        self.verdict_timeline.append(rec)
        if rec["final"]:
            self.has_final = True

    def _feed_debug(self, obj, t):
        agent = obj.get("agent", -1)
        hud_lines = {}  # line_id -> {"text":..., "score":...}  (本帧多子步去重,取最后子步)
        hud_has_data = False
        for prim in obj.get("prims", []):
            converted = self._convert_debug_prim(prim)
            if converted is None:
                continue
            layer = prim.get("layer", "default")
            d = prim.get("d", -1)
            if layer == "hud_stability" and converted.get("type") == "text":
                line_id = self._hud_line_id(converted.get("text", ""))
                if line_id is not None:
                    hud_has_data = True
                    hud_lines[line_id] = {
                        "text": converted["text"],
                        "score": self._hud_score(converted["text"]),
                    }
            entry = {
                "t": t,
                "agent": agent,
                "prim": converted,
                "expires_at": (t + d) if d is not None and d > 0 else None,
            }
            if d is not None and d <= 0:
                # 瞬时原语：该层仅保留最新一帧的批次
                bucket = self.debug_transient.setdefault(layer, {"t": None, "entries": []})
                if bucket["t"] != t:
                    bucket["t"] = t
                    bucket["entries"] = []
                bucket["entries"].append(entry)
            else:
                self.debug_prims.setdefault(layer, []).append(entry)

        if hud_has_data and agent is not None and agent >= 0 and t is not None:
            lines = [
                hud_lines.get(0, {"text": ""})["text"],
                hud_lines.get(1, {"text": ""})["text"],
                hud_lines.get(2, {"text": ""})["text"],
            ]
            score = hud_lines.get(0, {}).get("score")
            self.hud_history.setdefault(agent, {})[t] = {"lines": lines, "score": score}

    @staticmethod
    def _hud_line_id(text):
        if "综合" in text:
            return 0
        if "飞行" in text:
            return 1
        if "避障" in text:
            return 2
        return None

    @staticmethod
    def _hud_score(text):
        import re as _re
        m = _re.search(r"[-+]?\d+(?:\.\d+)?", text.replace("■", ""))
        return float(m.group()) if m else None

    def _convert_debug_prim(self, prim):
        pt = prim.get("t")
        color = prim.get("c", [255, 255, 255, 255])
        color_hex = "#{:02x}{:02x}{:02x}".format(color[0], color[1], color[2])
        if pt == "sphere":
            p = to_web(prim["p"][0], prim["p"][1], prim["p"][2])
            return {"type": "sphere", "pos": p, "radius": prim.get("r", 10) / 100.0, "color": color_hex}
        if pt == "line":
            a = to_web(prim["a"][0], prim["a"][1], prim["a"][2])
            b = to_web(prim["b"][0], prim["b"][1], prim["b"][2])
            return {"type": "line", "a": a, "b": b, "thickness": prim.get("th", 1), "color": color_hex}
        if pt == "arrow":
            a = to_web(prim["a"][0], prim["a"][1], prim["a"][2])
            b = to_web(prim["b"][0], prim["b"][1], prim["b"][2])
            return {"type": "arrow", "a": a, "b": b, "size": prim.get("sz", 10) / 100.0, "color": color_hex}
        if pt == "box":
            p = to_web(prim["p"][0], prim["p"][1], prim["p"][2])
            return {"type": "box", "pos": p, "extents": [prim["e"][0]/100, prim["e"][1]/100, prim["e"][2]/100],
                    "quat": prim.get("q", [0, 0, 0, 1]), "color": color_hex}
        if pt == "point":
            p = to_web(prim["p"][0], prim["p"][1], prim["p"][2])
            return {"type": "point", "pos": p, "size": prim.get("sz", 5) / 100.0, "color": color_hex}
        if pt == "text":
            p = to_web(prim["p"][0], prim["p"][1], prim["p"][2])
            return {"type": "text", "pos": p, "text": prim.get("s", ""), "color": color_hex}
        return None

    def _feed_traj(self, obj, _t):
        aid = obj.get("agent")
        if aid is None:
            return
        typ = obj.get("type")
        self._ensure_agent(aid)
        raw_pts = obj.get("pts") or []
        if raw_pts:
            log.debug("traj_raw: %s agent=%s first=%s last=%s count=%d", typ, aid, raw_pts[0], raw_pts[-1], len(raw_pts))
        key = {"traj_opt": "opt", "traj_plan": "plan", "traj_nmpc": "nmpc"}[typ]
        out = self._convert_traj_points(typ, raw_pts)
        self.future.setdefault(aid, {})[key] = out
        self._log_first_traj(aid, key, typ, raw_pts, out)
        log.debug("traj: %s agent=%s pts_in=%d pts_out=%d key=%s", typ, aid, len(raw_pts), len(out), key)
        if out:
            log.debug("traj_web: %s agent=%s first=%s last=%s", typ, aid, out[0], out[-1])

    def _convert_traj_points(self, typ, raw_pts):
        out = []
        for point in raw_pts:
            if typ == "traj_nmpc" and len(point) >= 4:
                web_point = to_web(point[0], point[1], point[2])
                out.append([web_point[0], web_point[1], web_point[2], float(point[3])])
            elif len(point) >= 3:
                out.append(to_web(point[0], point[1], point[2]))
        return out

    def _log_first_traj(self, aid, key, typ, raw_pts, out):
        first_key = (aid, key, self.reload_epoch)
        if len(raw_pts) < 2 or len(out) < 2 or first_key in self.traj_first_logged:
            return
        raw_dir = [round(raw_pts[1][index] - raw_pts[0][index], 3) for index in range(3)]
        web_dir = [round(out[1][index] - out[0][index], 3) for index in range(3)]
        log.info("[py-traj-first] type=%s agent=%s epoch=%d raw0=%s raw1=%s rawDir=%s web0=%s web1=%s webDir=%s count=%d",
                 typ, aid, self.reload_epoch, raw_pts[0], raw_pts[1], raw_dir, out[0], out[1], web_dir, len(out))
        self.traj_first_logged.add(first_key)

    def _push_wind(self, t, w_ue):
        speed_ms = round(math.hypot(math.hypot(w_ue[0], w_ue[1]), w_ue[2]) / 100.0, 2)
        vec = to_web(w_ue[0], w_ue[1], w_ue[2])
        # 水平方向角(deg): web 水平面为 X-Z,正北=-Z(对应 UE +X 前)
        dir_deg = (math.degrees(math.atan2(vec[0], -vec[2])) + 360.0) % 360.0
        self.wind_samples.append({"t": t, "speedMs": speed_ms, "dirDeg": round(dir_deg, 1), "vecMs": vec})

    def snapshot(self):
        agents = []
        for aid in sorted(self.agent_meta.keys()):
            m = self.agent_meta[aid]
            fut = self.future.get(aid, {})
            hud = self.hud_history.get(aid, {})
            hud_items = sorted(hud.items())
            hud_series = [
                {"t": kt, "lines": vd["lines"], "score": vd.get("score")}
                for kt, vd in _downsample(hud_items, MAX_HUD_POINTS)
            ]
            agents.append({
                "id": aid, "model": m["model"], "color": m["color"],
                "maxVelMs": m["maxVelMs"], "collisionRadiusM": m["collisionRadiusM"],
                "initPos": m["initPos"], "trace": _downsample(self.traces.get(aid, []), MAX_TRACE_POINTS),
                "futureOpt": fut.get("opt", []),
                "futurePlan": fut.get("plan", []),
                "futureNmpc": fut.get("nmpc", []),
                "hudHistory": hud_series,
            })
        verdict = self._build_verdict()
        duration_ms = 0
        if self.first_t is not None and self.last_t is not None:
            duration_ms = round((self.last_t - self.first_t) * 1000.0)
        waypoints = [self.waypoints[k] for k in sorted(self.waypoints.keys())]
        debug_snapshot = self._build_debug_snapshot()
        return {
            "scenario": self.meta.get("scenario", "") if self.meta else "",
            "t0_ms": None,
            "duration_ms": max(0, duration_ms),
            "reloadEpoch": self.reload_epoch,
            "agents": agents,
            "obstacles": [v for v in self.obstacles.values() if not v.get("_dyn")],
            "dynamicActorCount": self.dynamic_count,
            "waypoints": waypoints,
            "wind": {"config": self.wind_config, "samples": self.wind_samples[-300:]},
            "verdict": verdict,
            "summary": [self.summary[k] for k in sorted(self.summary)],
            "events": self.events,
            "series": {
                "speedMs": _series_pack(self.speed_series, agents, MAX_SERIES_POINTS),
                "altM": _series_pack(self.alt_series, agents, MAX_SERIES_POINTS),
                "clearanceM": _series_pack(self.clearance_series, agents, MAX_SERIES_POINTS),
            },
            "debug": debug_snapshot,
        }

    def _build_debug_snapshot(self):
        now = self.last_t if self.last_t is not None else 0
        out = {}
        for layer, entries in self.debug_prims.items():
            active = [e for e in entries if e["expires_at"] is None or e["expires_at"] > now]
            out[layer] = active
        for layer, bucket in self.debug_transient.items():
            if bucket["entries"]:
                out[layer] = out.get(layer, []) + bucket["entries"]
        return out

    def _build_verdict(self):
        if not self.verdict_timeline:
            return None
        last = self.verdict_timeline[-1]
        return {
            "final": "PASS" if last["passed"] else "FAIL",
            "reached": f"{last['reached']}/{last['total']}",
            "clearanceM": last["clearanceM"],
            "lateralDevM": last["lateralDevM"],
            "elapsedS": last["elapsedS"],
            "passed": last["passed"],
            "collided": last["collided"],
            "failures": last["failures"],
            "timeline": self.verdict_timeline,
        }


# ---------------------------------------------------------------------------
# State: 运行状态 (HTTP 控制端 + ndjson 文件轮询)
# ---------------------------------------------------------------------------

class State:
    def __init__(self, args, target, logs_dir):
        self.args = args
        self.target = target
        self.logs_dir = logs_dir
        self.lock = threading.Lock()
        self.state = NdjsonState()
        self.last_size = 0
        self.last_mtime = 0
        self.finished = False
        self.last_change_time = time.monotonic()
        self._pending_tail = ""
        # 上次已观测到的 reload epoch, 用于检测新场景热重载(同文件、不轮换)。
        # reload epoch 推进表示 UE 端 ScenarioLoader 装配了新场景, 旧的 final
        # verdict 不再代表当前状态 -> 必须 reset finished, 否则第二次仿真 SSE
        # 一连接就会立即收到 done 事件并断流。
        self.last_reload_epoch = 0
        # UE HTTP 控制端端口（None 表示控制端不可用，面板置灰）
        self.control_port = resolve_control_port(args.project)
        self.control_available = False
        self.last_control_refresh = 0.0
        self._initial_load()

    def refresh_control_port(self):
        now = time.monotonic()
        if now - self.last_control_refresh < 1.0:
            return self.control_port
        self.last_control_refresh = now
        port = resolve_control_port(self.args.project)
        if port != self.control_port:
            log.info("control_port_refreshed: %s -> %s", self.control_port, port)
            self.control_port = port
            self.control_available = False
        return self.control_port

    def mark_control_available(self):
        self.control_available = True

    def mark_control_unavailable(self):
        if self.control_available or self.control_port is not None:
            log.info("control_endpoint_unavailable: port=%s", self.control_port)
        self.control_available = False
        self.control_port = None

    def is_control_available(self):
        return self.control_available

    def _feed_text(self, text):
        # Buffer incomplete trailing fragment (no trailing newline) so the next
        # poll reassembles the full line instead of parsing a truncated JSON.
        text = self._pending_tail + text
        self._pending_tail = ""
        nl = text.rfind("\n")
        if nl == -1:
            self._pending_tail = text
            return
        complete = text[:nl]
        self._pending_tail = text[nl + 1:]
        for line in complete.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                self.state.feed(json.loads(line))
            except json.JSONDecodeError as e:
                log.warning("ndjson parse error: %s | line: %.80s", e, line)
                continue

    def _read_tail_lines(self, start_offset):
        """从 start_offset(字节偏移)读取文件, 返回 (text, new_offset)。

        只返回到最后一个 '\\n' 的完整行, new_offset 指向该 '\\n' 之后(行边界)。
        保证 last_size 始终停在行边界, 避免 text-mode seek 切断多字节 UTF-8
        字符(中文 HUD 文本)导致该行 JSON 损坏丢帧。UTF-8 自同步性保证 '\\n'
        (0x0A) 只出现在 ASCII 字节, 不会落进多字节字符的续字节里。
        """
        try:
            with open(self.target, "rb") as f:
                if start_offset > 0:
                    f.seek(start_offset)
                raw = f.read()
        except OSError as e:
            log.warning("_read_tail_lines read error at offset %d: %s", start_offset, e)
            return "", start_offset
        if not raw:
            return "", start_offset
        last_nl = raw.rfind(b"\n")
        if last_nl == -1:
            # 还没有完整行(UE 正在写一行尚未刷出换行), 不推进, 下次重读
            return "", start_offset
        complete = raw[:last_nl + 1]
        return complete.decode("utf-8", errors="replace"), start_offset + last_nl + 1

    def _initial_load(self):
        if not os.path.isfile(self.target):
            return
        try:
            text, consumed = self._read_tail_lines(0)
            self.last_size = consumed
            self.last_mtime = os.path.getmtime(self.target)
            self.state = NdjsonState()
            with self.lock:
                self._feed_text(text)
                # 同步初始 reload epoch, 避免首次 poll_once 误判"epoch 推进"
                self.last_reload_epoch = self.state.reload_epoch
            if self.state.has_final:
                self.finished = True
        except OSError as e:
            log.error("Failed to load initial ndjson file %s: %s", self.target, e)

    def poll_once(self):
        try:
            if not os.path.isfile(self.target):
                return False
            size = os.path.getsize(self.target)
            mtime = os.path.getmtime(self.target)
        except OSError as e:
            log.warning("poll_once stat error: %s", e)
            return False

        if size < self.last_size:
            log.info("ndjson file shrinked, reloading all (%d -> %d bytes)", self.last_size, size)
            return self._reload_all()

        if size > self.last_size and mtime != self.last_mtime:
            text, consumed = self._read_tail_lines(self.last_size)
            if consumed <= self.last_size:
                return False
            log.debug("ndjson tail read: +%d bytes (total %d)", consumed - self.last_size, size)
            with self.lock:
                self._feed_text(text)
                self.last_size = consumed
                self.last_mtime = mtime
                self.last_change_time = time.monotonic()
                self._maybe_reset_finished_on_epoch_advance()
            if self.state.has_final:
                self.finished = True
            return True
        return False

    def _maybe_reset_finished_on_epoch_advance(self):
        """reload epoch 推进 = UE 端装配了新场景, 旧的 final verdict 失效。
        必须同时复位 finished 与 NdjsonState.has_final, 否则后续 poll_once
        的 `if self.state.has_final: self.finished = True` 会立刻把 finished
        拉回 True, 使复位失效。"""
        if self.state.reload_epoch > self.last_reload_epoch:
            self.last_reload_epoch = self.state.reload_epoch
            self.state.has_final = False
            self.finished = False
            log.info("reload_epoch advanced -> finished reset (epoch=%d)",
                     self.state.reload_epoch)

    def _reload_all(self):
        text, consumed = self._read_tail_lines(0)
        with self.lock:
            self.state = NdjsonState()
            self._pending_tail = ""
            # 文件被截断/重写 = 新仿真启动, 旧的 finished 状态必须失效
            self.finished = False
            self._feed_text(text)
            self.last_size = consumed
            try:
                self.last_mtime = os.path.getmtime(self.target)
            except OSError:
                self.last_mtime = 0
            self.last_change_time = time.monotonic()
            self.last_reload_epoch = self.state.reload_epoch
        log.info("ndjson reloaded: %d bytes", self.last_size)
        if self.state.has_final:
            self.finished = True
        return True

    def mark_finished(self):
        with self.lock:
            self.finished = True

    def check_idle_finish(self, idle_timeout):
        with self.lock:
            if not self.finished and (time.monotonic() - self.last_change_time) > idle_timeout:
                if self.state.first_t is not None:
                    self.finished = True

    def snapshot_with_result(self):
        with self.lock:
            data = self.state.snapshot()
        res = read_scenario_result(self.logs_dir)
        if res:
            data["verdictResult"] = res
        data["finished"] = self.finished
        data["logFile"] = os.path.basename(self.target)
        return data


# ---------------------------------------------------------------------------
# 任务切割: 扫描 Logs/tasks/ 目录, 管理历史任务
# ---------------------------------------------------------------------------

def list_tasks(logs_dir):
    tasks_dir = os.path.join(logs_dir, "tasks")
    if not os.path.isdir(tasks_dir):
        return []
    out = []
    for name in sorted(os.listdir(tasks_dir), reverse=True):
        task_dir = os.path.join(tasks_dir, name)
        if not os.path.isdir(task_dir):
            continue
        result_path = os.path.join(task_dir, "result.json")
        ndjson_path = os.path.join(task_dir, "telemetry.ndjson")
        info = {"id": name, "dir": name}
        if os.path.isfile(result_path):
            try:
                with open(result_path, "r", encoding="utf-8") as f:
                    r = json.load(f)
                info["scenario"] = r.get("scenario", name)
                info["verdict"] = r.get("verdict", "UNKNOWN")
                info["seed"] = r.get("seed")
                m = r.get("metrics", {})
                info["elapsedS"] = round(m.get("elapsedSec", 0), 2)
                info["collided"] = bool(m.get("collided", False))
            except (json.JSONDecodeError, OSError):
                info["scenario"] = name
                info["verdict"] = "UNKNOWN"
        else:
            info["scenario"] = name
            info["verdict"] = "UNKNOWN"
        info["hasNdjson"] = os.path.isfile(ndjson_path)
        out.append(info)
    return out


def load_task_ndjson(logs_dir, task_id):
    task_dir = os.path.join(logs_dir, "tasks", task_id)
    ndjson_path = os.path.join(task_dir, "telemetry.ndjson")
    if not os.path.isfile(ndjson_path):
        return None
    state = NdjsonState()
    try:
        with open(ndjson_path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    state.feed(json.loads(line))
                except json.JSONDecodeError:
                    continue
    except OSError as e:
        log.warning("load_task_ndjson error: %s", e)
        return None
    data = state.snapshot()
    # 归档的 result.json 是权威判决（覆盖 ndjson 逐帧推算，兜底 UE crash 未写 final verdict 的场景）
    result_path = os.path.join(task_dir, "result.json")
    if os.path.isfile(result_path):
        try:
            with open(result_path, "r", encoding="utf-8") as f:
                rd = json.load(f)
            m = rd.get("metrics", {})
            data["verdictResult"] = {
                "final": rd.get("verdict", "UNKNOWN"),
                "reached": f"{m.get('waypointsReached', 0)}/{m.get('waypointsTotal', 0)}",
                "clearanceM": _clearance_m(m.get("minClearanceCm")),
                "lateralDevM": round(m.get("maxLateralDevCm", 0) / 100.0, 2),
                "elapsedS": round(m.get("elapsedSec", 0), 2),
                "collided": bool(m.get("collided", False)),
                "failures": rd.get("failures", []),
                "seed": rd.get("seed"),
            }
        except (json.JSONDecodeError, OSError):
            pass
    return data


def delete_task(logs_dir, task_id):
    import shutil
    task_dir = os.path.join(logs_dir, "tasks", task_id)
    if not os.path.isdir(task_dir):
        return False
    shutil.rmtree(task_dir)
    log.info("task deleted: %s", task_id)
    return True
