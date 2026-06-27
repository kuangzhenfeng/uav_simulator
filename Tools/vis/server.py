#!/usr/bin/env python3
# UAV 仿真可视化后端(纯标准库)
# 提供静态托管 + 全量数据接口 + SSE 实时增量推流。
# 数据源: Logs/telemetry.ndjson(UAV TelemetryRecorder 追加写的专用数据流)
#         + scenario_result.json(最终权威判决)。
# ndjson 坐标/单位为 UE 原生(cm, 左手系), 本端做坐标系变换后输出给前端。

import argparse
import json
import math
import os
import sys
import time
import threading
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

# ---------------------------------------------------------------------------
# 坐标 / 单位约定
#   UE : 左手系, X 前 / Y 右 / Z 上, 单位 cm
#   Web: 右手系(three.js), Y 上, 单位 m
#   变换: x_web = x_ue/100, y_web = z_ue/100, z_web = -y_ue/100
# ---------------------------------------------------------------------------

# 固定 agent 配色,按首次 spawn 顺序(== AgentID 升序)分配,避免抖动
AGENT_COLORS = [
    "#4f8cff", "#ff6b6b", "#51cf66", "#fcc419",
    "#cc5de8", "#22b8cf", "#ff922b", "#20c997",
]


def to_web(x_ue, y_ue, z_ue):
    """UE cm 点坐标(左手 Z-up) -> Web m 点坐标(右手 Y-up)。"""
    return [round(x_ue / 100.0, 3), round(z_ue / 100.0, 3), round(-y_ue / 100.0, 3)]


def size_web(x_ue, y_ue, z_ue):
    """UE cm 半尺寸分量 -> Web m。仅单位换算(尺寸恒正)。"""
    return [round(abs(x_ue) / 100.0, 3), round(abs(z_ue) / 100.0, 3), round(abs(y_ue) / 100.0, 3)]


def _obstacle_type(t):
    return {0: "Sphere", 1: "Box", 2: "Cylinder", 3: "Custom"}.get(t, "Box")


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
        self.first_t = None
        self.last_t = None
        self.has_final = False

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

        if typ == "meta":
            self.meta = obj
            return

        if typ == "reload":
            # 场景热重载标记：自增 epoch，快照中带出，前端据此清轨迹/重置游标
            self.reload_epoch += 1
            return

        if typ == "wind_config":
            steady = obj.get("steady") or [0, 0, 0]
            self.wind_config = {
                "type": obj.get("windType"),
                "steadyMs": to_web(steady[0], steady[1], steady[2]),
            }
            return

        if typ == "spawn":
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
            return

        if typ == "obstacle":
            oid = obj.get("id")
            center = obj.get("center") or [0, 0, 0]
            extents = obj.get("extents") or [0, 0, 0]
            entry = {
                "id": oid,
                "type": _obstacle_type(obj.get("oType")),
                "center": to_web(center[0], center[1], center[2]),
                "extents": size_web(extents[0], extents[1], extents[2]),
                "actor": obj.get("actor", ""),
            }
            if obj.get("dynamic"):
                # 动态障碍(其他飞机)单独计数,不计入静态场景
                if oid not in self.obstacles or not self.obstacles[oid].get("_dyn"):
                    self.dynamic_count += 1
                    entry["_dyn"] = True
                self.obstacles[oid] = entry
            else:
                self.obstacles[oid] = entry
            return

        if typ == "waypoint":
            idx = obj.get("idx")
            pos = obj.get("pos") or [0, 0, 0]
            self.waypoints[idx] = to_web(pos[0], pos[1], pos[2])
            return

        if typ == "frame":
            for a in obj.get("agents", []):
                aid = a.get("id")
                self._ensure_agent(aid)
                pos_ue = a.get("pos") or [0, 0, 0]
                vel_ue = a.get("vel") or [0, 0, 0]
                pos = to_web(pos_ue[0], pos_ue[1], pos_ue[2])
                speed_ms = round(math.hypot(math.hypot(vel_ue[0], vel_ue[1]), vel_ue[2]) / 100.0, 2)
                self.traces.setdefault(aid, []).append(
                    {"t": t, "pos": pos, "speed": speed_ms, "ctrl": a.get("ctrl", 0)}
                )
                self.speed_series.setdefault(aid, []).append([t, speed_ms])
                self.alt_series.setdefault(aid, []).append([t, pos[1]])
                clr = a.get("clearance")
                if clr is not None and clr >= 0:
                    near_m = round(clr / 100.0, 2)
                    s = self.clearance_series.setdefault(aid, [])
                    if s and abs(s[-1][0] - t) < 0.5:
                        s[-1][1] = near_m
                    else:
                        s.append([t, near_m])
            w = obj.get("wind")
            if w:
                self._push_wind(t, w)
            return

        if typ == "metrics":
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
            return

        if typ == "event":
            aid = obj.get("agent")
            pos = obj.get("pos") or [0, 0, 0]
            wp = to_web(pos[0], pos[1], pos[2])
            self.events.append({
                "t": t, "type": obj.get("event", "Event"), "agent": aid,
                "detail": f"pos=({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f}) crashed={bool(obj.get('crashed'))}",
            })
            return

        if typ == "verdict":
            clr_cm = obj.get("clearanceCm")
            lat_cm = obj.get("lateralDevCm")
            rec = {
                "t": t,
                "passed": bool(obj.get("passed")),
                "reached": obj.get("reached", 0),
                "total": obj.get("total", 0),
                "clearanceM": round(clr_cm / 100.0, 2) if clr_cm is not None else None,
                "lateralDevM": round(lat_cm / 100.0, 2) if lat_cm is not None else None,
                "elapsedS": round(obj.get("elapsedSec", 0), 2),
                "final": bool(obj.get("final")),
            }
            self.verdict_timeline.append(rec)
            if rec["final"]:
                self.has_final = True
            return

        if typ in ("traj_opt", "traj_plan", "traj_nmpc"):
            aid = obj.get("agent")
            if aid is None:
                return
            self._ensure_agent(aid)
            raw_pts = obj.get("pts") or []
            key = {"traj_opt": "opt", "traj_plan": "plan", "traj_nmpc": "nmpc"}[typ]
            # 坐标 cm->m + 坐标系变换；nmpc 额外保留 cost（第4元素）
            out = []
            for p in raw_pts:
                if typ == "traj_nmpc" and len(p) >= 4:
                    w = to_web(p[0], p[1], p[2])
                    out.append([w[0], w[1], w[2], float(p[3])])
                elif len(p) >= 3:
                    out.append(to_web(p[0], p[1], p[2]))
            # 覆盖式快照：只保留最新一份（区别于历史 trace 的累积式）
            self.future.setdefault(aid, {})[key] = out
            return

    def _push_wind(self, t, w_ue):
        speed_ms = round(math.hypot(math.hypot(w_ue[0], w_ue[1]), w_ue[2]) / 100.0, 2)
        vec = to_web(w_ue[0], w_ue[1], w_ue[2])
        # 水平方向角(deg): 以 web x/z 平面,正北=+z
        dir_deg = (math.degrees(math.atan2(vec[0], vec[2])) + 360.0) % 360.0
        self.wind_samples.append({"t": t, "speedMs": speed_ms, "dirDeg": round(dir_deg, 1), "vecMs": vec})

    def snapshot(self):
        agents = []
        for aid in sorted(self.agent_meta.keys()):
            m = self.agent_meta[aid]
            fut = self.future.get(aid, {})
            agents.append({
                "id": aid, "model": m["model"], "color": m["color"],
                "maxVelMs": m["maxVelMs"], "collisionRadiusM": m["collisionRadiusM"],
                "initPos": m["initPos"], "trace": self.traces.get(aid, []),
                "futureOpt": fut.get("opt", []),
                "futurePlan": fut.get("plan", []),
                "futureNmpc": fut.get("nmpc", []),
            })
        verdict = self._build_verdict()
        duration_ms = 0
        if self.first_t is not None and self.last_t is not None:
            duration_ms = round((self.last_t - self.first_t) * 1000.0)
        waypoints = [self.waypoints[k] for k in sorted(self.waypoints.keys())]
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
                "speedMs": _series_pack(self.speed_series, agents),
                "altM": _series_pack(self.alt_series, agents),
                "clearanceM": _series_pack(self.clearance_series, agents),
            },
        }

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
            "timeline": self.verdict_timeline,
        }


def _series_pack(src, agents):
    out = []
    for a in agents:
        pts = src.get(a["id"], [])
        if pts:
            out.append({"agent": a["id"], "color": a["color"], "points": pts})
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
            "clearanceM": round(m.get("minClearanceCm", 0) / 100.0, 2) if m.get("minClearanceCm") is not None else None,
            "lateralDevM": round(m.get("maxLateralDevCm", 0) / 100.0, 2),
            "elapsedS": round(m.get("elapsedSec", 0), 2),
            "collided": bool(m.get("collided", False)),
            "failures": data.get("failures", []),
            "seed": data.get("seed"),
        }
    except (json.JSONDecodeError, OSError):
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
# UE HTTP 控制端端口发现
# ---------------------------------------------------------------------------

def resolve_control_port(project_root):
    """读 Saved/.uav-ctrl/port.json 获取 UE 控制端端口；读不到返回 None（控制不可用）。"""
    path = os.path.join(project_root, "Saved", ".uav-ctrl", "port.json")
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data.get("port")
    except (OSError, ValueError):
        return None


def forward_to_control(state, sub_path, body_obj=None):
    """把控制命令转发到 UE 控制端 http://127.0.0.1:<port>/control<sub_path>。
    返回 (ok, result_dict)。控制端不可用时返回 (False, {...})。"""
    if not state.control_port:
        return False, {"ok": False, "error": "control endpoint unavailable"}
    url = f"http://127.0.0.1:{state.control_port}/control{sub_path}"
    data = json.dumps(body_obj or {}).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            raw = resp.read().decode("utf-8")
            try:
                return True, json.loads(raw)
            except ValueError:
                return True, {"ok": True, "raw": raw}
    except Exception as e:
        return False, {"ok": False, "error": str(e)}


# ---------------------------------------------------------------------------
# Scenario DTO 基本校验（防 UE 端崩溃的前置门）
# ---------------------------------------------------------------------------

VALID_MODELS = {"Agri_AG20", "Agri_AG60", "Agri_AG100", "Map_SVPro", "Map_SVLiDAR"}
VALID_OBS_TYPES = {"Sphere", "Box", "Cylinder"}
VALID_MOVEMENTS = {"Static", "LinearVelocity", "PatrolLoop", "PatrolPingPong"}
VALID_MODES = {"Once", "Loop", "PingPong"}


def _is_vec3(v):
    return isinstance(v, (list, tuple)) and len(v) == 3 and all(isinstance(x, (int, float)) for x in v)


def validate_scenario_dto(dto):
    """返回 (ok: bool, error: str|None)。仅做边界/枚举校验，不校验语义。"""
    if not isinstance(dto, dict):
        return False, "dto must be an object"
    fleet = dto.get("fleet")
    if not isinstance(fleet, list) or len(fleet) == 0:
        return False, "fleet must be a non-empty array"
    if len(fleet) > 16:
        return False, "fleet too large (max 16)"
    for i, a in enumerate(fleet):
        if not isinstance(a, dict):
            return False, f"fleet[{i}] must be an object"
        if a.get("model") not in VALID_MODELS:
            return False, f"fleet[{i}].model invalid"
        if not _is_vec3(a.get("initPos", [0, 0, 0])):
            return False, f"fleet[{i}].initPos must be [x,y,z]"
        mode = a.get("mode", "Once")
        if mode not in VALID_MODES:
            return False, f"fleet[{i}].mode invalid"
        for j, w in enumerate(a.get("waypoints", [])):
            if not isinstance(w, dict) or not _is_vec3(w.get("pos")):
                return False, f"fleet[{i}].waypoints[{j}].pos invalid"
    obs = dto.get("obstacles", [])
    if isinstance(obs, list):
        if len(obs) > 64:
            return False, "too many obstacles (max 64)"
        for k, o in enumerate(obs):
            if not isinstance(o, dict):
                return False, f"obstacles[{k}] must be an object"
            if o.get("type", "Box") not in VALID_OBS_TYPES:
                return False, f"obstacles[{k}].type invalid"
            if o.get("movement", "Static") not in VALID_MOVEMENTS:
                return False, f"obstacles[{k}].movement invalid"
            if not _is_vec3(o.get("center", [0, 0, 0])):
                return False, f"obstacles[{k}].center invalid"
            speed = o.get("patrolSpeed", 300)
            if not isinstance(speed, (int, float)) or speed < 0:
                return False, f"obstacles[{k}].patrolSpeed must be >= 0"
    sim = dto.get("sim", {})
    if isinstance(sim, dict):
        slomo = sim.get("slomo", 0)
        if not isinstance(slomo, (int, float)) or slomo < 0:
            return False, "sim.slomo must be >= 0"
    return True, None


# ---------------------------------------------------------------------------
# HTTP 服务
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
        # UE HTTP 控制端端口（None 表示控制端不可用，面板置灰）
        self.control_port = resolve_control_port(args.project)
        self._initial_load()

    def _feed_text(self, text):
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                self.state.feed(json.loads(line))
            except json.JSONDecodeError:
                continue

    def _initial_load(self):
        if os.path.isfile(self.target):
            try:
                with open(self.target, "r", encoding="utf-8", errors="replace") as f:
                    text = f.read()
                self.last_size = len(text.encode("utf-8", errors="replace"))
                self.last_mtime = os.path.getmtime(self.target)
                self.state = NdjsonState()
                with self.lock:
                    self._feed_text(text)
                if self.state.has_final:
                    self.finished = True
            except OSError:
                pass

    def poll_once(self):
        try:
            if not os.path.isfile(self.target):
                return False
            size = os.path.getsize(self.target)
            mtime = os.path.getmtime(self.target)
        except OSError:
            return False

        if size < self.last_size:
            return self._reload_all()

        if size > self.last_size and mtime != self.last_mtime:
            try:
                with open(self.target, "r", encoding="utf-8", errors="replace") as f:
                    if self.last_size > 0:
                        f.seek(self.last_size)
                    tail = f.read()
            except OSError:
                return False
            if tail:
                with self.lock:
                    self._feed_text(tail)
                    self.last_size = size
                    self.last_mtime = mtime
                    self.last_change_time = time.monotonic()
                if self.state.has_final:
                    self.finished = True
                return True
        return False

    def _reload_all(self):
        try:
            with open(self.target, "r", encoding="utf-8", errors="replace") as f:
                text = f.read()
        except OSError:
            return False
        with self.lock:
            self.state = NdjsonState()
            self._feed_text(text)
            self.last_size = len(text.encode("utf-8", errors="replace"))
            self.last_mtime = os.path.getmtime(self.target)
            self.last_change_time = time.monotonic()
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


WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
PRESET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "presets")


# ---------------------------------------------------------------------------
# Scenario Schema：前后端共享的枚举/范围契约，前端表单单一数据源
# 与 server.py validate_scenario_dto、UE ScenarioFactory 的 Parse* 保持一致
# ---------------------------------------------------------------------------

SCHEMA = {
    "models": [
        {"id": "Agri_AG20", "label": "农业 AG20"},
        {"id": "Agri_AG60", "label": "农业 AG60"},
        {"id": "Agri_AG100", "label": "农业 AG100"},
        {"id": "Map_SVPro", "label": "测绘 SVPro"},
        {"id": "Map_SVLiDAR", "label": "测绘 SVLiDAR"},
    ],
    "obstacleTypes": ["Sphere", "Box", "Cylinder"],
    "movements": ["Static", "LinearVelocity", "PatrolLoop", "PatrolPingPong"],
    "windTypes": ["None", "Constant", "Gust", "Turbulent"],
    "missionModes": ["Once", "Loop", "PingPong"],
    "controlModes": ["Attitude", "Position", "Trajectory"],
    "mpcTypes": ["Nonlinear", "Linear"],
    # 默认值：前端新建字段时直接取用，保证空配置可合法下发
    "defaults": {
        "name": "WebScenario",
        "fleet": [{
            "model": "Agri_AG20", "initPos": [0, 0, 100],
            "yaw": 0, "isLeader": True, "mode": "Once", "waypoints": [],
        }],
        "obstacles": [],
        "wind": {"type": "Constant", "steady": [300, 0, 0], "enabled": True,
                 "gustAmplitude": 200, "turbulenceIntensity": 100},
        "acceptance": {"requireAllWaypoints": True, "waypointRadiusCm": 300,
                       "minClearanceCm": 0, "maxLateralDeviationCm": 300,
                       "timeoutSec": 120, "energyBudget": 0},
        "sim": {"slomo": 8, "durationSec": 60, "controlMode": "", "mpcType": ""},
    },
}


def _preset_path(name):
    """预设名 -> 文件路径。仅允许字母/数字/下划线/短横，禁止路径分隔。"""
    if not name or not isinstance(name, str):
        return None
    if not all(c.isalnum() or c in "_-" for c in name):
        return None
    return os.path.join(PRESET_DIR, f"{name}.json")


def list_presets():
    if not os.path.isdir(PRESET_DIR):
        return []
    out = []
    for fn in sorted(os.listdir(PRESET_DIR)):
        if fn.endswith(".json"):
            out.append(fn[:-5])
    return out


def read_preset(name):
    fp = _preset_path(name)
    if not fp or not os.path.isfile(fp):
        return None
    try:
        with open(fp, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def write_preset(name, dto):
    fp = _preset_path(name)
    if not fp:
        return False, "invalid preset name"
    os.makedirs(PRESET_DIR, exist_ok=True)
    try:
        with open(fp, "w", encoding="utf-8") as f:
            json.dump(dto, f, ensure_ascii=False, indent=2)
        return True, None
    except OSError as e:
        return False, str(e)


def delete_preset(name):
    fp = _preset_path(name)
    if not fp or not os.path.isfile(fp):
        return False, "not found"
    try:
        os.remove(fp)
        return True, None
    except OSError as e:
        return False, str(e)


class Handler(BaseHTTPRequestHandler):
    state_holder = None

    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            self._serve_file("index.html", "text/html; charset=utf-8")
        elif path == "/api/data":
            data = self.state_holder.snapshot_with_result()
            data["controlAvailable"] = self.state_holder.control_port is not None
            self._send(200, "application/json", json.dumps(data).encode())
        elif path == "/api/schema":
            # 前端表单枚举/默认值的单一数据源，附带当前控制端可用性
            schema = dict(SCHEMA)
            schema["controlAvailable"] = self.state_holder.control_port is not None
            self._send(200, "application/json", json.dumps(schema).encode())
        elif path == "/api/presets":
            self._send(200, "application/json",
                       json.dumps({"presets": list_presets()}).encode())
        elif path.startswith("/api/presets/"):
            name = path[len("/api/presets/"):]
            data = read_preset(name)
            if data is None:
                self._send(404, "application/json", b'{"ok":false,"error":"not found"}')
            else:
                self._send(200, "application/json", json.dumps(data).encode())
        elif path == "/api/control/status":
            ok, result = forward_to_control(self.state_holder, "/status", None)
            result["controlAvailable"] = self.state_holder.control_port is not None
            self._send(200 if ok else 502, "application/json", json.dumps(result).encode())
        elif path == "/api/finish":
            self.state_holder.mark_finished()
            self._send(200, "application/json", b'{"ok":true}')
        elif path == "/api/stream":
            self._handle_sse()
        elif path.startswith("/web/"):
            self._serve_file(path[len("/web/"):], _guess_mime(path))
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 预设库：保存/覆盖
        if path.startswith("/api/presets/"):
            name = path[len("/api/presets/"):]
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                dto = json.loads(raw.decode("utf-8")) if raw else {}
            except ValueError:
                self._send(400, "application/json", b'{"ok":false,"error":"invalid json"}')
                return
            ok_msg, err = validate_scenario_dto(dto)
            if not ok_msg:
                self._send(400, "application/json",
                           json.dumps({"ok": False, "error": err}).encode())
                return
            ok, werr = write_preset(name, dto)
            code = 200 if ok else 400
            payload = {"ok": ok} if ok else {"ok": False, "error": werr}
            self._send(code, "application/json", json.dumps(payload).encode())
            return

        # 统一校验：仅接受已知控制路由
        valid = ("/api/control/reload", "/api/control/slomo",
                 "/api/control/wind", "/api/control/params")
        if path not in valid:
            self._send(404, "application/json", b'{"ok":false,"error":"not found"}')
            return

        length = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            body = json.loads(raw.decode("utf-8")) if raw else {}
        except ValueError:
            self._send(400, "application/json", b'{"ok":false,"error":"invalid json"}')
            return

        # 基本校验，防 UE 端崩溃
        if path == "/api/control/reload":
            ok_msg, err = validate_scenario_dto(body)
            if not ok_msg:
                self._send(400, "application/json",
                           json.dumps({"ok": False, "error": err}).encode())
                return

        sub_path = path[len("/api/control"):]  # -> /reload, /slomo, ...
        ok, result = forward_to_control(self.state_holder, sub_path, body)
        code = 200 if ok else 502
        self._send(code, "application/json", json.dumps(result).encode())

    def do_DELETE(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path.startswith("/api/presets/"):
            name = path[len("/api/presets/"):]
            ok, err = delete_preset(name)
            code = 200 if ok else 404
            payload = {"ok": True} if ok else {"ok": False, "error": err}
            self._send(code, "application/json", json.dumps(payload).encode())
            return
        self._send(404, "application/json", b'{"ok":false,"error":"not found"}')

    def _serve_file(self, rel, mime):
        fp = os.path.join(WEB_DIR, rel)
        if not os.path.isfile(fp):
            self._send(404, "text/plain", b"not found")
            return
        with open(fp, "rb") as f:
            body = f.read()
        self._send(200, mime, body)

    def _handle_sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            self._sse_push("snapshot")
            idle_timeout = self.state_holder.args.idle_timeout
            tick = 0
            while True:
                time.sleep(self.state_holder.args.poll_interval)
                changed = self.state_holder.poll_once()
                self.state_holder.check_idle_finish(idle_timeout)
                tick += 1
                if changed or self.state_holder.finished or tick % 4 == 0:
                    self._sse_push("snapshot")
                if self.state_holder.finished:
                    self.wfile.write(b"event: done\ndata: {}\n\n")
                    self.wfile.flush()
                    break
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _sse_push(self, _evt):
        data = self.state_holder.snapshot_with_result()
        payload = "data: " + json.dumps(data) + "\n\n"
        self.wfile.write(payload.encode())
        self.wfile.flush()


def _guess_mime(path):
    if path.endswith(".js"):
        return "text/javascript; charset=utf-8"
    if path.endswith(".css"):
        return "text/css; charset=utf-8"
    if path.endswith(".html"):
        return "text/html; charset=utf-8"
    if path.endswith(".json"):
        return "application/json"
    if path.endswith(".svg"):
        return "image/svg+xml"
    return "application/octet-stream"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    # __file__ = <PROJECT_ROOT>/Tools/vis/server.py → 项目根向上两级。
    default_project_root = os.path.dirname(os.path.dirname(here))
    p = argparse.ArgumentParser(description="UAV 仿真可视化后端")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--watch", default=None, help="实时监听的 ndjson 文件(默认 Logs/telemetry.ndjson)")
    p.add_argument("--project", default=default_project_root, help="项目根目录(定位 Logs/)")
    p.add_argument("--poll-interval", type=float, default=0.5, help="实时轮询间隔(秒)")
    p.add_argument("--idle-timeout", type=float, default=5.0, help="文件无增长多久后判定仿真结束(秒)")
    args = p.parse_args()

    target, logs_dir = resolve_target(args, args.project)
    state = State(args, target, logs_dir)
    Handler.state_holder = state

    def _bg_poll():
        while True:
            time.sleep(args.poll_interval)
            try:
                state.poll_once()
                state.check_idle_finish(args.idle_timeout)
            except Exception:
                pass
    threading.Thread(target=_bg_poll, daemon=True).start()

    print(f"[VIS] 数据源: {target}")
    print(f"[VIS] Logs 目录: {logs_dir}")
    print(f"[VIS] 访问地址: http://{args.host}:{args.port}")
    sys.stdout.flush()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
