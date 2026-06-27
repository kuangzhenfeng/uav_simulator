#!/usr/bin/env python3
import json
import logging
import os
import time
import urllib.request

log = logging.getLogger("vis_server")


def resolve_control_port(project_root):
    """读 Saved/.uav-ctrl/port.json 获取 UE 控制端端口；读不到返回 None（控制不可用）。"""
    path = os.path.join(project_root, "Saved", ".uav-ctrl", "port.json")
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        port = data.get("port")
        log.info("control_port_discovered: %s", port)
        return port
    except (OSError, ValueError) as e:
        log.debug("control_port_not_found: %s", e)
        return None


def forward_to_control(state, sub_path, body_obj=None):
    """把控制命令转发到 UE 控制端 http://127.0.0.1:<port>/control<sub_path>。
    返回 (ok, result_dict)。控制端不可用时返回 (False, {...})。"""
    state.refresh_control_port()
    if not state.control_port:
        state.mark_control_unavailable()
        return False, {"ok": False, "error": "control endpoint unavailable"}
    url = f"http://127.0.0.1:{state.control_port}/control{sub_path}"
    data = json.dumps(body_obj or {}).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
    started = time.monotonic()
    if sub_path == "/reload":
        log.info("[py-reload-fwd] forwarding reload endpoint=%s body_bytes=%d", url, len(data))
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            raw = resp.read().decode("utf-8")
            latency_ms = round((time.monotonic() - started) * 1000.0)
            try:
                body_obj = json.loads(raw)
                if sub_path == "/reload":
                    log.info("[py-reload-fwd] reload response status=%s latency_ms=%d body_keys=%s body=%.300s",
                             resp.status, latency_ms, list((body_obj or {}).keys()), raw)
                state.mark_control_available()
                log.info("forward_to_control(%s) -> ok (body_keys=%s)", sub_path, list((body_obj or {}).keys()))
                return True, body_obj
            except ValueError:
                if sub_path == "/reload":
                    log.warning("[py-reload-fwd] reload non_json status=%s latency_ms=%d body=%.300s", resp.status, latency_ms, raw)
                state.mark_control_available()
                log.warning("forward_to_control(%s) non-JSON response: %.200s", sub_path, raw)
                return True, {"ok": True, "raw": raw}
    except Exception as e:
        if sub_path == "/reload":
            latency_ms = round((time.monotonic() - started) * 1000.0)
            log.error("[py-reload-fwd] reload failed endpoint=%s latency_ms=%d error=%s", url, latency_ms, e)
        log.error("forward_to_control(%s) failed: %s", sub_path, e)
        state.mark_control_unavailable()
        return False, {"ok": False, "error": str(e)}


def _coerce_positive_number(value, default):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if number > 0 else default


def replay_reload_when_control_ready(state, body_obj, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        ok, _ = forward_to_control(state, "/status", None)
        if ok:
            reload_ok, result = forward_to_control(state, "/reload", body_obj)
            if reload_ok and result.get("ok", True):
                log.info("reload_after_control_start_ok: pid=%s", getattr(state.control_process, "pid", None))
            else:
                log.error("reload_after_control_start_failed: result=%s", result)
            return
        time.sleep(1.0)
    log.error("reload_after_control_start_timeout: timeout_sec=%s", timeout_sec)
