#!/usr/bin/env python3
import logging
import os
import signal
import subprocess
import sys
import threading
import time

log = logging.getLogger("vis_server")


class SimManager:
    def __init__(self, project_root, logs_dir):
        self.project_root = project_root
        self.logs_dir = logs_dir
        self._process = None
        self._lock = threading.Lock()
        self._start_time = None
        self._duration = None
        self._slomo = None

    def start(self, duration=60, slomo=8, scenario=None):
        with self._lock:
            self._stop_unlocked()
            return self._start_unlocked(duration, slomo, scenario)

    def ensure_running(self, duration=60, slomo=8, scenario=None):
        """幂等启动：进程已在跑则复用，否则拉起。返回 started 标记区分两种情况。"""
        with self._lock:
            if self._process and self._process.poll() is None:
                return {"ok": True, "already_running": True, "pid": self._process.pid}
            return self._start_unlocked(duration, slomo, scenario)

    def _start_unlocked(self, duration=60, slomo=8, scenario=None):
        if sys.platform == "win32":
            script = os.path.join(self.project_root, "Script", "sim.bat")
            cmd = ["cmd.exe", "/c", script, str(duration), str(slomo)]
        else:
            script = os.path.join(self.project_root, "Script", "sim.sh")
            cmd = ["bash", script, str(duration), str(slomo)]

        if scenario:
            cmd.append(scenario)

        if not os.path.isfile(script):
            return {"ok": False, "error": f"simulation script not found: {script}"}

        os.makedirs(self.logs_dir, exist_ok=True)
        log_path = os.path.join(self.logs_dir, "sim_output.log")
        log.info("sim_start: cmd=%s log=%s", cmd, log_path)
        try:
            with open(log_path, "ab") as stream:
                self._process = subprocess.Popen(
                    cmd,
                    cwd=self.project_root,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=(sys.platform != "win32"),
                )
        except OSError as exc:
            log.error("sim_start_failed: %s", exc)
            return {"ok": False, "error": str(exc)}

        self._start_time = time.time()
        self._duration = duration
        self._slomo = slomo
        log.info("sim_started: pid=%s duration=%d slomo=%s", self._process.pid, duration, slomo)
        return {"ok": True, "started": True, "pid": self._process.pid, "duration": duration, "slomo": slomo}

    def stop(self):
        with self._lock:
            return self._stop_unlocked()

    def _stop_unlocked(self):
        if not self._process:
            return {"ok": True, "killed": False}
        if self._process.poll() is not None:
            pid = self._process.pid
            self._process = None
            self._start_time = None
            return {"ok": True, "killed": False, "already_dead": True, "pid": pid}

        pid = self._process.pid
        log.info("sim_stop: pid=%s", pid)
        try:
            if sys.platform != "win32":
                os.killpg(os.getpgid(pid), signal.SIGTERM)
            else:
                self._process.terminate()
            self._process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            log.warning("sim_stop_timeout, force killing pid=%s", pid)
            if sys.platform != "win32":
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            else:
                self._process.kill()
        except ProcessLookupError:
            pass

        self._process = None
        self._start_time = None
        log.info("sim_stopped: pid=%s", pid)
        return {"ok": True, "killed": True, "pid": pid}

    def status(self):
        with self._lock:
            if self._process and self._process.poll() is None:
                elapsed = time.time() - self._start_time if self._start_time else 0
                return {
                    "running": True,
                    "pid": self._process.pid,
                    "elapsedSec": round(elapsed, 1),
                    "duration": self._duration,
                    "slomo": self._slomo,
                }
            return {"running": False}
