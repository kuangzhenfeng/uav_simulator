#!/usr/bin/env python3
# UAV 仿真可视化后端(纯标准库)
# 提供静态托管 + 全量数据接口 + SSE 实时增量推流。
# 数据源: Logs/telemetry.ndjson(UAV TelemetryRecorder 追加写的专用数据流)
#         + scenario_result.json(最终权威判决)。
# ndjson 坐标/单位为 UE 原生(cm, 左手系), 本端做坐标系变换后输出给前端。

import argparse
import json
import logging
import os
import sys
import time
import threading
from logging.handlers import RotatingFileHandler
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

from coords import dir_to_web, to_web
from control_proxy import forward_to_control, resolve_control_port
import presets as preset_store
from sim_manager import SimManager
from state import NdjsonState, State, delete_task, list_tasks, load_task_ndjson, read_scenario_result, resolve_target
from validation import (
    VALID_MODELS,
    VALID_MODES,
    VALID_MOVEMENTS,
    VALID_OBS_TYPES,
    validate_scenario_dto,
)

log = logging.getLogger("vis_server")

# Re-exports for backward compatibility (tests import these via vis_server.*)
to_web = to_web
NdjsonState = NdjsonState
State = State
forward_to_control = forward_to_control
read_scenario_result = read_scenario_result
resolve_target = resolve_target
resolve_control_port = resolve_control_port
validate_scenario_dto = validate_scenario_dto

# ---------------------------------------------------------------------------
# 坐标 / 单位约定
#   UE : 左手系, X 前 / Y 右 / Z 上, 单位 cm
#   Web: 右手系(three.js), Y 上, 单位 m
#   变换: x_web = y_ue/100, y_web = z_ue/100, z_web = -x_ue/100
#   俯视时 web 的"上方"(−Z)对应 UE 的"前方"(+X),与 UE 编辑器俯视一致。
# ---------------------------------------------------------------------------


WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
PRESET_DIR = preset_store.PRESET_DIR


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
    # 默认值使用前端展示单位(m/m/s)，提交时转 UE cm。
    "defaults": {
        "name": "WebScenario",
        "fleet": [{
            "model": "Agri_AG20", "initPos": [0, 0, 5],
            "yaw": 0, "isLeader": True, "mode": "Once",
            "waypoints": [{"pos": [50, 0, 5]}],
        }],
        "obstacles": [
            {"type": "Box", "center": [8, -4, 5], "extents": [2, 2, 2], "movement": "Static", "safetyMargin": 0.5},
            {"type": "Box", "center": [8, 4, 5], "extents": [2, 2, 2], "movement": "Static", "safetyMargin": 0.5},
            {"type": "Box", "center": [16, 0, 5], "extents": [3, 3, 3], "movement": "Static", "safetyMargin": 0.5},
            {"type": "Sphere", "center": [24, -3, 6], "extents": [1.5, 1.5, 1.5], "movement": "Static", "safetyMargin": 0.5},
            {"type": "Sphere", "center": [24, 3, 4], "extents": [1.5, 1.5, 1.5], "movement": "Static", "safetyMargin": 0.5},
            {"type": "Box", "center": [20, 0, 5], "extents": [1.5, 1.5, 1.5], "movement": "LinearVelocity", "velocity": [0, 3, 0], "safetyMargin": 0.5},
            {"type": "Cylinder", "center": [32, 0, 0], "extents": [1, 1, 5], "movement": "Static", "safetyMargin": 0.5},
            {"type": "Box", "center": [40, -2, 5], "extents": [2, 2, 2], "movement": "Static", "safetyMargin": 0.5},
        ],
        "wind": {"type": "Constant", "steady": [3, 0, 0], "enabled": True,
                 "gustAmplitude": 2, "turbulenceIntensity": 1},
        "acceptance": {"requireAllWaypoints": True, "waypointRadiusCm": 300,
                       "minClearanceCm": 0, "maxLateralDeviationCm": 300,
                       "timeoutSec": 120, "energyBudget": 0},
        "sim": {"slomo": 8, "durationSec": 60, "controlMode": "", "mpcType": ""},
    },
}


def _sync_preset_dir():
    preset_store.PRESET_DIR = PRESET_DIR


def _preset_path(name):
    _sync_preset_dir()
    return preset_store.preset_path(name)


def list_presets():
    _sync_preset_dir()
    return preset_store.list_presets()


def read_preset(name):
    _sync_preset_dir()
    return preset_store.read_preset(name)


def write_preset(name, dto):
    _sync_preset_dir()
    return preset_store.write_preset(name, dto)


def delete_preset(name):
    _sync_preset_dir()
    return preset_store.delete_preset(name)


class Handler(BaseHTTPRequestHandler):
    state_holder = None
    sim_manager = None

    def log_message(self, *a):
        log.debug("HTTP %s %s from %s", self.command, self.path, self.client_address[0])

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
        elif path == "/favicon.ico":
            self._serve_file("favicon.ico", "image/x-icon")
        elif path == "/api/data":
            data = self.state_holder.snapshot_with_result()
            data["controlAvailable"] = self.state_holder.is_control_available()
            data["simStatus"] = self.sim_manager.status() if self.sim_manager else {"running": False}
            self._send(200, "application/json", json.dumps(data).encode())
        elif path == "/api/tasks":
            tasks = list_tasks(self.state_holder.logs_dir)
            self._send(200, "application/json", json.dumps(tasks).encode())
        elif path.startswith("/api/task/"):
            task_id = path[len("/api/task/"):]
            data = load_task_ndjson(self.state_holder.logs_dir, task_id)
            if data is None:
                self._send(404, "application/json", b'{"ok":false,"error":"task not found"}')
            else:
                data["finished"] = True
                self._send(200, "application/json", json.dumps(data).encode())
        elif path == "/api/schema":
            # 前端表单枚举/默认值的单一数据源，附带当前控制端可用性
            schema = dict(SCHEMA)
            schema["controlAvailable"] = self.state_holder.is_control_available()
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
            result["controlAvailable"] = self.state_holder.is_control_available()
            result["simStatus"] = self.sim_manager.status() if self.sim_manager else {"running": False}
            log.debug("control status check: available=%s port=%s", self.state_holder.is_control_available(), self.state_holder.control_port)
            self._send(200, "application/json", json.dumps(result).encode())
        elif path == "/api/sim/status":
            result = self.sim_manager.status() if self.sim_manager else {"running": False}
            self._send(200, "application/json", json.dumps(result).encode())
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

        # 前端日志上报
        if path == "/api/log":
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length > 0 else b"[]"
            try:
                entries = json.loads(raw.decode("utf-8")) if raw else []
            except ValueError:
                self._send(400, "application/json", b'{"ok":false,"error":"invalid json"}')
                return
            fe_log = logging.getLogger("vis_frontend")
            if isinstance(entries, list):
                for entry in entries:
                    level = entry.get("level", "info").upper()
                    ts = entry.get("ts", "")
                    mod = entry.get("module", "")
                    msg = entry.get("msg", "")
                    numeric_level = getattr(logging, level, logging.INFO)
                    extra = entry.get("extra", "")
                    fe_log.log(numeric_level, "[%s] [%s] %s | %s", ts, mod, msg, extra if extra else "-")
            self._send(200, "application/json", b'{"ok":true}')
            return

        if path == "/api/sim/start":
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                body = json.loads(raw.decode("utf-8")) if raw else {}
            except ValueError:
                self._send(400, "application/json", b'{"ok":false,"error":"invalid json"}')
                return

            # 接收完整场景配置（与原 /api/control/reload 的 DTO 一致）
            ok_msg, err = validate_scenario_dto(body)
            if not ok_msg:
                self._send(400, "application/json",
                           json.dumps({"ok": False, "error": err}).encode())
                return

            sim_dto = body.get("sim", {}) or {}
            duration = max(1, min(int(sim_dto.get("durationSec", 60)), 3600))
            slomo = max(0.1, min(float(sim_dto.get("slomo", 8)), 100))

            # 1. 确保进程在跑（已在跑就复用，否则拉起）
            ensure_result = self.sim_manager.ensure_running(
                duration=duration, slomo=slomo, scenario=None)
            if not ensure_result.get("ok"):
                self._send(400, "application/json",
                           json.dumps(ensure_result).encode())
                return

            # 2. 若是新拉起的进程，等控制端 HTTP 上线
            if ensure_result.get("started"):
                deadline = time.monotonic() + 60
                while time.monotonic() < deadline:
                    ok, _ = forward_to_control(self.state_holder, "/status", None)
                    if ok:
                        break
                    time.sleep(1.0)

            # 3. 推送场景配置
            ok, result = forward_to_control(self.state_holder, "/reload", body)
            if ok:
                result["pid"] = ensure_result.get("pid")
                self._send(200, "application/json", json.dumps(result).encode())
            else:
                result["pid"] = ensure_result.get("pid")
                self._send(502, "application/json", json.dumps(result).encode())
            return

        if path == "/api/sim/stop":
            result = self.sim_manager.stop() if self.sim_manager else {"ok": True, "killed": False}
            self._send(200, "application/json", json.dumps(result).encode())
            return

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
                 "/api/control/wind", "/api/control/params",
                 "/api/control/stop", "/api/control/pause",
                 "/api/control/exit")
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
                log.warning("reload_dto_invalid: %s", err)
                self._send(400, "application/json",
                           json.dumps({"ok": False, "error": err}).encode())
                return
            fleet = body.get("fleet", [])
            log.info("reload_dto_ok: fleet=%d obstacles=%d scene=%s",
                     len(fleet), len(body.get("obstacles", [])), body.get("name", "?"))

        sub_path = path[len("/api/control"):]  # -> /reload, /slomo, ...
        log.info("POST %s from=%s body_size=%d", path, self.client_address[0], length)
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
        if path.startswith("/api/task/"):
            task_id = path[len("/api/task/"):]
            ok = delete_task(self.state_holder.logs_dir, task_id)
            code = 200 if ok else 404
            payload = {"ok": True} if ok else {"ok": False, "error": "task not found"}
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
        try:
            self._send(200, mime, body)
        except (BrokenPipeError, ConnectionResetError):
            log.info("static client disconnected: %s", rel)

    def _handle_sse(self):
        log.info("New SSE client connected")
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
            log.info("SSE client disconnected")

    def _sse_push(self, _evt):
        data = self.state_holder.snapshot_with_result()
        # SSE 必须携带 simStatus, 前端 isLiveMode 以 "UE 进程在跑" 为唯一权威信号;
        # 否则前端只能凭 data.finished 判断, vis 重启后旧 telemetry 会被误判为 LIVE。
        data["simStatus"] = self.sim_manager.status() if self.sim_manager else {"running": False}
        data["controlAvailable"] = self.state_holder.is_control_available()
        payload = "data: " + json.dumps(data) + "\n\n"
        log.debug("sse_push: payload=%d bytes agents=%d obstacles=%d",
                  len(payload), len(data.get("agents", [])), len(data.get("obstacles", [])))
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
    if path.endswith(".ico"):
        return "image/x-icon"
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
    p.add_argument("--poll-interval", type=float, default=0.15, help="实时轮询间隔(秒), 越小越流畅")
    p.add_argument("--idle-timeout", type=float, default=5.0, help="文件无增长多久后判定仿真结束(秒)")
    args = p.parse_args()

    target, logs_dir = resolve_target(args, args.project)

    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    log_file = os.path.join(logs_dir, "vis_server.log")
    os.makedirs(logs_dir, exist_ok=True)
    fh = RotatingFileHandler(log_file, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8")
    fh.setFormatter(fmt)
    fh.setLevel(logging.DEBUG)

    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    sh.setLevel(logging.INFO)

    log.addHandler(fh)
    log.addHandler(sh)
    log.setLevel(logging.DEBUG)

    fe_handler = logging.FileHandler(os.path.join(logs_dir, "vis_frontend.log"), encoding="utf-8")
    fe_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
    fe_handler.setLevel(logging.DEBUG)
    fe_logger = logging.getLogger("vis_frontend")
    fe_logger.addHandler(fe_handler)
    fe_logger.setLevel(logging.DEBUG)

    state = State(args, target, logs_dir)
    Handler.state_holder = state
    Handler.sim_manager = SimManager(args.project, logs_dir)

    log.info("[VIS] Starting - port=%d host=%s data=%s logs=%s", args.port, args.host, target, logs_dir)

    def _bg_poll():
        while True:
            time.sleep(args.poll_interval)
            try:
                state.poll_once()
                state.check_idle_finish(args.idle_timeout)
            except Exception:
                log.exception("bg_poll error")
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
