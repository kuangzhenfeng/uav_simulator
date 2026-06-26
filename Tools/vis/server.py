#!/usr/bin/env python3
# UAV 仿真日志可视化后端(纯标准库)
# 提供静态托管 + 全量数据接口 + SSE 实时增量推流。
# 解析 Logs/uav.log(或 UE 运行时实时日志)+ scenario_result.json。

import argparse
import json
import os
import re
import sys
import time
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

# ---------------------------------------------------------------------------
# 坐标 / 单位约定
#   UE : 左手系, X 前 / Y 右 / Z 上, 单位 cm
#   Web: 右手系(three.js), Y 上, 单位 m
#   变换: x_web = x_ue/100, y_web = z_ue/100, z_web = -y_ue/100
# ---------------------------------------------------------------------------

# FLT_MAX 哨兵阈值,超过视为 "无数据"
FLT_MAX_GUARD = 1e30

# 固定 agent 配色,避免随帧抖动
AGENT_COLORS = [
    "#4f8cff", "#ff6b6b", "#51cf66", "#fcc419",
    "#cc5de8", "#22b8cf", "#ff922b", "#20c997",
]

# 视觉示意半径(m),日志未直接输出机体半径
DEFAULT_COLLISION_RADIUS_M = 0.6

# 时间戳正则: [2026.06.26-14.15.27:719][ 81]
TS_RE = re.compile(r"\[(\d{4})\.(\d{2})\.(\d{2})-(\d{2})\.(\d{2})\.(\d{2}):(\d{3})\]\[\s*(\d+)\]")

# 各 TAG 正则
MODEL_RE = re.compile(r"\[Model\]\s+(\S+)\s+\|.*?MaxVel=([\d.]+)cm/s")
REGISTER_AGENT_RE = re.compile(r"Registered as Agent\s+(\d+)")
SPAWN_RE = re.compile(r"Pos=X=([-\d.]+)\s+Y=([-\d.]+)\s+Z=([-\d.]+)")
PERF_RE = re.compile(
    r"\[PERF_SUMMARY\]\s+Agent=(\d+)\s+FrameTime=([\d.]+)ms\s+Speed=([\d.]+)\s+"
    r"Pos=\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+ControlMode=(\d+)"
)
OBSTACLE_RE = re.compile(
    r"Obstacle registered:\s+ID=(\d+),\s+Type=(\d+),\s+"
    r"Center=X=([-\d.]+)\s+Y=([-\d.]+)\s+Z=([-\d.]+),\s+"
    r"Extents=X=([-\d.]+)\s+Y=([-\d.]+)\s+Z=([-\d.]+),\s+Actor=([^,]+)"
)
WAYPOINT_RE = re.compile(r"Waypoint\[(\d+)\]:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)")
RESULT_RE = re.compile(
    r"\[Scenario\]\s+RESULT=(PASS|FAIL)\s+\|\s+Reached=(\d+)/(\d+)\s+\|\s+"
    r"Clearance=([\d.]+)cm\s+\|\s+LateralDev=([\d.]+)cm\s+\|\s+Elapsed=([\d.]+)s"
)
ANEMO_RE = re.compile(
    r"\[Anemometer\]\s+Speed=([\d.]+)\s+Direction=([\d.]+)\s+Wind=\(([-\d.]+),([-\d.]+),([-\d.]+)\)"
)
CBF_DETAIL_RE = re.compile(r"\[CBF_DETAIL\]\s+Agent=(\d+)\s+ObsCount=(\d+)\s+NearDist=([\d.]+)")
SIM_SUMMARY_RE = re.compile(
    r"\[SIM_SUMMARY\]\s+Agent=(\d+)\s+SpeedRatio=([\d.]+)\s+LowSpeedDur=([\d.]+)\s+"
    r"MaxDev=([\d.]+)\s+MaxRoll=([\d.]+)\s+MaxPitch=([\d.]+)\s+"
    r"InstabTime=([\d.]+)\s+Stuck=(\d+)\s+ForceComplete=(\d+)"
)
CRASH_RE = re.compile(r"\[Crash\]|Obstacle penetration|Agent-Agent collision", re.IGNORECASE)
SIM_RESULT_RE = re.compile(r"\[SIM_RESULT\]\s*(.*)")
SCENARIO_LOADED_RE = re.compile(r"\[Scenario\]\s+Loaded from.*?:\s*(\S+)")
WIND_INIT_RE = re.compile(r"\[WindField\]\s+Initialized:\s+type=(\d+)\s+steady=\(([-\d,]+)\)")


def ts_to_ms(m):
    """时间戳匹配组 -> 当天内的绝对毫秒数(用于排序与 t0 基准)。"""
    hh, mm, ss, ms = int(m.group(4)), int(m.group(5)), int(m.group(6)), int(m.group(7))
    return ((hh * 60 + mm) * 60 + ss) * 1000 + ms


def to_web(x_ue, y_ue, z_ue):
    """UE cm 点坐标(左手 Z-up) -> Web m 点坐标(右手 Y-up)。"""
    return [round(x_ue / 100.0, 3), round(z_ue / 100.0, 3), round(-y_ue / 100.0, 3)]


def size_web(x_ue, y_ue, z_ue):
    """UE cm 半尺寸/标量分量 -> Web m。仅单位换算,不做坐标系取负(尺寸恒正)。"""
    return [round(abs(x_ue) / 100.0, 3), round(abs(z_ue) / 100.0, 3), round(abs(y_ue) / 100.0, 3)]


def safe_float(v, guard=FLT_MAX_GUARD):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if abs(f) > guard else f


class LogState:
    """累积式日志解析状态。feed() 增量喂行,snapshot() 产出完整数据模型。"""

    def __init__(self):
        self.scenario = ""
        self.t0_ms = None
        self.last_ms = None
        # agent 信息
        self.agent_model = {}        # id -> model
        self.agent_maxvel = {}       # id -> m/s
        self.agent_init = {}         # id -> [x,y,z] web
        self._pending_model = None    # 最近一次 [Model] 行,配对给下一个 Registered agent
        self._pending_maxvel = None   # 同上,对应 MaxVel
        self._pending_register_id = None  # 等待配对的 SpawnSafety 目标 agent
        # 时序
        self.traces = {}             # id -> list[{t,pos,speed,ctrl}]
        self.speed_series = {}       # id -> list[[t,v_ms]]
        self.alt_series = {}         # id -> list[[t,alt_m]]
        self.clearance_series = {}   # id -> list[[t,v_m]]  (NearDist)
        self.wind_samples = []       # [{t,speedMs,dirDeg,vecMs}]
        self.verdict_timeline = []   # [{t,reached,total,clearanceM,...,passed}]
        self.summary = {}            # id -> dict(取每个 agent 最后一次 SIM_SUMMARY)
        self.events = []             # [{t,type,agent,detail}]
        # 场景静态
        self.obstacles = {}          # actor -> obstacle dict(静态 Actor_*)
        self.dynamic_actors = set()  # BP_UAVPawn* 动态障碍(其他飞机)的 actor 名
        self.waypoints = []          # [[x,y,z], ...]
        self.wind_config = None      # {type, steadyMs}

    def _t(self, ms):
        if ms is None:
            return None
        if self.t0_ms is None:
            self.t0_ms = ms
        return round((ms - self.t0_ms) / 1000.0, 3)

    def feed(self, line):
        tsm = TS_RE.search(line)
        ms = ts_to_ms(tsm) if tsm else None
        t = self._t(ms) if ms is not None else None
        if ms is not None:
            self.last_ms = ms

        # Model 行(配对给下一个注册的 agent)
        m = MODEL_RE.search(line)
        if m:
            self._pending_model = m.group(1)
            self._pending_maxvel = safe_float(m.group(2))
            return

        # Registered as Agent N —— 配对最近一次 [Model];spawn 在随后 SpawnSafety 行补上
        m = REGISTER_AGENT_RE.search(line)
        if m:
            aid = int(m.group(1))
            if aid not in self.agent_model:
                self.agent_model[aid] = self._pending_model or "UAV"
                self.agent_maxvel[aid] = self._pending_maxvel
                self._pending_register_id = aid  # 等待 SpawnSafety 配对
            self._pending_model = None
            self._pending_maxvel = None
            return

        # SpawnSafety —— 配给最近一次注册、尚未拿到 spawn 位置的 agent
        if "SpawnSafety" in line:
            sm = SPAWN_RE.search(line)
            if sm and self._pending_register_id is not None:
                aid = self._pending_register_id
                self.agent_init[aid] = to_web(
                    float(sm.group(1)), float(sm.group(2)), float(sm.group(3)))
                self._pending_register_id = None
            return

        # PERF_SUMMARY —— 轨迹主源
        m = PERF_RE.search(line)
        if m:
            aid = int(m.group(1))
            speed_cm = float(m.group(3))
            pos = to_web(float(m.group(4)), float(m.group(5)), float(m.group(6)))
            ctrl = int(m.group(7))
            self.traces.setdefault(aid, []).append(
                {"t": t, "pos": pos, "speed": round(speed_cm / 100.0, 2), "ctrl": ctrl}
            )
            self.speed_series.setdefault(aid, []).append([t, round(speed_cm / 100.0, 2)])
            self.alt_series.setdefault(aid, []).append([t, pos[1]])
            return

        # 障碍物注册
        m = OBSTACLE_RE.search(line)
        if m:
            oid, otype = int(m.group(1)), int(m.group(2))
            center = to_web(float(m.group(3)), float(m.group(4)), float(m.group(5)))
            extents = size_web(float(m.group(6)), float(m.group(7)), float(m.group(8)))
            actor = m.group(9)
            entry = {"id": oid, "type": _obstacle_type(otype),
                     "center": center, "extents": extents, "actor": actor}
            if actor.startswith("BP_UAVPawn"):
                # 动态障碍(其他飞机),单独标记不进静态场景
                self.dynamic_actors.add(actor)
            else:
                # 静态场景障碍按 actor 去重(ID 会重复分配)
                self.obstacles[actor] = entry
            return

        # 航点
        m = WAYPOINT_RE.search(line)
        if m:
            idx = int(m.group(1))
            wp = to_web(float(m.group(2)), float(m.group(3)), float(m.group(4)))
            while len(self.waypoints) <= idx:
                self.waypoints.append(None)
            self.waypoints[idx] = wp
            return

        # 逐秒判决
        m = RESULT_RE.search(line)
        if m:
            passed = m.group(1) == "PASS"
            reached, total = int(m.group(2)), int(m.group(3))
            clearance_m = safe_float(m.group(4))
            clearance_m = round(clearance_m / 100.0, 2) if clearance_m is not None else None
            lat_m = safe_float(m.group(5))
            lat_m = round(lat_m / 100.0, 2) if lat_m is not None else None
            self.verdict_timeline.append({
                "t": t, "passed": passed, "reached": reached, "total": total,
                "clearanceM": clearance_m, "lateralDevM": lat_m,
                "elapsedS": round(float(m.group(6)), 2),
            })
            return

        # 风场实时
        m = ANEMO_RE.search(line)
        if m:
            speed_cm = safe_float(m.group(1))
            vec = [safe_float(m.group(3)), safe_float(m.group(4)), safe_float(m.group(5))]
            if speed_cm is not None and None not in vec:
                self.wind_samples.append({
                    "t": t,
                    "speedMs": round(speed_cm / 100.0, 2),
                    "dirDeg": round(float(m.group(2)), 1),
                    "vecMs": to_web(vec[0], vec[1], vec[2]),
                })
            return

        # 避障净空(每帧,取每秒最后一个值以降噪)
        m = CBF_DETAIL_RE.search(line)
        if m:
            aid = int(m.group(1))
            near = safe_float(m.group(3))
            if near is not None:
                near_m = round(near / 100.0, 2)
                s = self.clearance_series.setdefault(aid, [])
                # 同一秒内只保留最后一个采样
                if s and abs(s[-1][0] - t) < 0.5:
                    s[-1][1] = near_m
                else:
                    s.append([t, near_m])
            return

        # 指标汇总(每 agent 取最后一次)
        m = SIM_SUMMARY_RE.search(line)
        if m:
            aid = int(m.group(1))
            self.summary[aid] = {
                "agent": aid,
                "speedRatio": round(float(m.group(2)), 3),
                "lowSpeedDur": round(float(m.group(3)), 2),
                "maxDev": round(float(m.group(4)), 2),
                "maxRoll": round(float(m.group(5)), 2),
                "maxPitch": round(float(m.group(6)), 2),
                "instabTime": round(float(m.group(7)), 2),
                "stuck": int(m.group(8)),
                "forceComplete": int(m.group(9)),
            }
            return

        # 硬失败事件
        if CRASH_RE.search(line):
            self.events.append({"t": t, "type": "Crash/Collision", "agent": None, "detail": _clean(line)})
            return
        m = SIM_RESULT_RE.search(line)
        if m:
            self.events.append({"t": t, "type": "SimResult", "agent": None, "detail": m.group(1).strip()})
            return

        # 场景名 / 风场配置
        m = SCENARIO_LOADED_RE.search(line)
        if m and not self.scenario:
            self.scenario = m.group(1)
            return
        m = WIND_INIT_RE.search(line)
        if m and self.wind_config is None:
            parts = m.group(2).split(",")
            if len(parts) == 3:
                self.wind_config = {
                    "type": int(m.group(1)),
                    "steadyMs": to_web(float(parts[0]), float(parts[1]), float(parts[2])),
                }
            return

    def snapshot(self):
        agents = []
        all_ids = sorted(set(list(self.traces.keys()) +
                             list(self.agent_model.keys()) +
                             list(self.agent_init.keys())))
        for i, aid in enumerate(all_ids):
            agents.append({
                "id": aid,
                "model": self.agent_model.get(aid, "UAV"),
                "color": AGENT_COLORS[i % len(AGENT_COLORS)],
                "maxVelMs": self.agent_maxvel.get(aid),
                "collisionRadiusM": DEFAULT_COLLISION_RADIUS_M,
                "initPos": self.agent_init.get(aid),
                "trace": self.traces.get(aid, []),
            })
        verdict = _build_verdict(self.verdict_timeline)
        duration_ms = 0
        if self.t0_ms is not None and self.last_ms is not None:
            duration_ms = self.last_ms - self.t0_ms
        return {
            "scenario": self.scenario,
            "t0_ms": self.t0_ms,
            "duration_ms": max(0, duration_ms),
            "agents": agents,
            "obstacles": list(self.obstacles.values()),
            "dynamicActorCount": len(self.dynamic_actors),
            "waypoints": [w for w in self.waypoints if w is not None],
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


def _obstacle_type(t):
    return {0: "Box", 1: "Sphere", 2: "Cylinder", 3: "Custom"}.get(t, "Box")


def _series_pack(src, agents):
    out = []
    for a in agents:
        pts = src.get(a["id"], [])
        if pts:
            out.append({"agent": a["id"], "color": a["color"], "points": pts})
    return out


def _build_verdict(timeline):
    if not timeline:
        return None
    last = timeline[-1]
    return {
        "final": "PASS" if last["passed"] else "FAIL",
        "reached": f"{last['reached']}/{last['total']}",
        "clearanceM": last["clearanceM"],
        "lateralDevM": last["lateralDevM"],
        "elapsedS": last["elapsedS"],
        "passed": last["passed"],
        "timeline": timeline,
    }


def _clean(line):
    """去掉行首时间戳/category,保留可读正文。"""
    s = re.sub(r"^\[[^\]]*\]\[\s*\d+\]", "", line)
    s = re.sub(r"^Log\w+:\s*", "", s)
    return s.strip()[:200]


def parse_text(text):
    state = LogState()
    for line in text.splitlines():
        state.feed(line)
    return state.snapshot()


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
# 目标日志文件探测
# ---------------------------------------------------------------------------

def default_ue_log_path(project_root):
    if sys.platform == "darwin":
        return os.path.expanduser("~/Library/Logs/uav_simulator/uav_simulator.log")
    # Windows / Linux
    return os.path.join(project_root, "Saved", "Logs", "uav_simulator.log")


def resolve_target(args, project_root):
    """优先级: --watch > UE 实时日志(若存在) > Logs/uav.log。"""
    logs_dir = os.path.join(project_root, "Logs")
    if args.watch:
        return args.watch, logs_dir
    ue_log = default_ue_log_path(project_root)
    if os.path.isfile(ue_log):
        return ue_log, logs_dir
    return os.path.join(logs_dir, "uav.log"), logs_dir


# ---------------------------------------------------------------------------
# HTTP 服务
# ---------------------------------------------------------------------------

class State:
    def __init__(self, args, target, logs_dir):
        self.args = args
        self.target = target
        self.logs_dir = logs_dir
        self.lock = threading.Lock()
        self.state = LogState()
        self.last_size = 0
        self.last_mtime = 0
        self.finished = False
        self.last_change_time = time.monotonic()
        self._initial_load()

    def _initial_load(self):
        if os.path.isfile(self.target):
            try:
                with open(self.target, "r", encoding="utf-8", errors="replace") as f:
                    text = f.read()
                self.last_size = len(text.encode("utf-8", errors="replace"))
                self.last_mtime = os.path.getmtime(self.target)
                self.state = LogState()
                for line in text.splitlines():
                    self.state.feed(line)
            except OSError:
                pass

    def poll_once(self):
        """轮询文件增量,返回是否有更新。"""
        try:
            if not os.path.isfile(self.target):
                return False
            size = os.path.getsize(self.target)
            mtime = os.path.getmtime(self.target)
        except OSError:
            return False

        # 文件被重写(truncate 或缩小):全量重读重建
        if size < self.last_size:
            return self._reload_all()

        # 普通追加(mtime 变且文件变大):只解析新增尾部,避免全量重建开销
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
                    # 首次加载(t0 未定)走全量,以建立基准时间戳
                    if self.state.t0_ms is None:
                        self.state = LogState()
                        for line in tail.splitlines():
                            self.state.feed(line)
                    else:
                        for line in tail.splitlines():
                            self.state.feed(line)
                    self.last_size = size
                    self.last_mtime = mtime
                    self.last_change_time = time.monotonic()
                return True
        return False

    def _reload_all(self):
        try:
            with open(self.target, "r", encoding="utf-8", errors="replace") as f:
                text = f.read()
        except OSError:
            return False
        with self.lock:
            self.state = LogState()
            for line in text.splitlines():
                self.state.feed(line)
            self.last_size = len(text.encode("utf-8", errors="replace"))
            self.last_mtime = os.path.getmtime(self.target)
            self.last_change_time = time.monotonic()
        return True

    def mark_finished(self):
        with self.lock:
            self.finished = True

    def check_idle_finish(self, idle_timeout):
        """文件 idle 超时则标记结束(启发式)。"""
        with self.lock:
            if not self.finished and (time.monotonic() - self.last_change_time) > idle_timeout:
                # 仅在已有数据时才判定结束,避免空文件立即 done
                if self.state.t0_ms is not None:
                    self.finished = True

    def snapshot_with_result(self):
        with self.lock:
            data = self.state.snapshot()
        res = read_scenario_result(self.logs_dir)
        if res:
            # JSON 判决覆盖逐秒推算的 final 字段(更权威)
            data["verdictResult"] = res
        data["finished"] = self.finished
        data["logFile"] = os.path.basename(self.target)
        return data


WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


class Handler(BaseHTTPRequestHandler):
    state_holder = None  # State 实例,由 main 注入

    def log_message(self, *a):
        pass  # 静默访问日志

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
            self._send(200, "application/json", json.dumps(data).encode())
        elif path == "/api/finish":
            self.state_holder.mark_finished()
            self._send(200, "application/json", b'{"ok":true}')
        elif path == "/api/stream":
            self._handle_sse()
        elif path.startswith("/web/"):
            self._serve_file(path[len("/web/"):], _guess_mime(path))
        else:
            self._send(404, "text/plain", b"not found")

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
            # 先推一帧当前快照
            self._sse_push("snapshot")
            idle_timeout = self.state_holder.args.idle_timeout
            tick = 0
            while True:
                time.sleep(self.state_holder.args.poll_interval)
                changed = self.state_holder.poll_once()
                self.state_holder.check_idle_finish(idle_timeout)
                tick += 1
                if changed or self.state_holder.finished or tick % 4 == 0:
                    # 有更新 / 已结束 / 或每 4 拍发一次心跳保活
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
    return "application/octet-stream"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(here))
    p = argparse.ArgumentParser(description="UAV 仿真日志可视化后端")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--watch", default=None, help="实时监听的日志文件(默认探测 UE 运行时日志)")
    p.add_argument("--project", default=project_root, help="项目根目录(定位 Logs/)")
    p.add_argument("--poll-interval", type=float, default=0.5, help="实时轮询间隔(秒)")
    p.add_argument("--idle-timeout", type=float, default=5.0, help="文件无增长多久后判定仿真结束(秒)")
    args = p.parse_args()

    target, logs_dir = resolve_target(args, args.project)
    state = State(args, target, logs_dir)
    Handler.state_holder = state

    # 后台轮询线程:独立于 HTTP/SSE 连接持续跟踪日志增量,
    # 保证 /api/data 与 /api/stream 都能拿到最新数据(刷新页面或短暂断连不丢实时性)。
    def _bg_poll():
        while True:
            time.sleep(args.poll_interval)
            try:
                state.poll_once()
                state.check_idle_finish(args.idle_timeout)
            except Exception:
                pass
    threading.Thread(target=_bg_poll, daemon=True).start()

    print(f"[VIS] 日志源: {target}")
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
