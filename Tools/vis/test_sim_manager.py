#!/usr/bin/env python3
# Tools/vis/sim_manager.py - SimManager 进程管理测试
#
# 验证 ensure_running 的幂等性:
#   - 进程不存在时拉起并返回 started=True
#   - 进程已在跑时复用并返回 already_running=True (不重启)
#   - 进程已死时重新拉起
#
# 运行: cd Tools/vis && python3 test_sim_manager.py

import importlib.util
import os
import sys
import tempfile
import shutil
from pathlib import Path
from unittest.mock import patch, MagicMock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

_spec = importlib.util.spec_from_file_location("sim_manager", os.path.join(HERE, "sim_manager.py"))
sim_manager_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim_manager_mod)

SimManager = sim_manager_mod.SimManager


def _make_manager(tmp_path):
    # SimManager._start_unlocked 会检查 sim.sh 是否存在
    script_dir = tmp_path / "Script"
    script_dir.mkdir(exist_ok=True)
    (script_dir / "sim.sh").write_text("#!/bin/bash\n")
    (script_dir / "sim.bat").write_text("@echo off\n")
    return SimManager(str(tmp_path), str(tmp_path))


def test_ensure_running_starts_when_no_process(tmp_path):
    """无进程时 ensure_running 应拉起并返回 started=True。"""
    mgr = _make_manager(tmp_path)
    fake_proc = MagicMock()
    fake_proc.pid = 12345
    fake_proc.poll.return_value = None

    with patch("subprocess.Popen", return_value=fake_proc):
        result = mgr.ensure_running(duration=30, slomo=4)

    assert result["ok"] is True
    assert result["started"] is True
    assert result["pid"] == 12345


def test_ensure_running_reuses_when_already_running(tmp_path):
    """进程已在跑时 ensure_running 应复用, 返回 already_running=True, 不重启。"""
    mgr = _make_manager(tmp_path)
    fake_proc = MagicMock()
    fake_proc.pid = 999
    fake_proc.poll.return_value = None

    with patch("subprocess.Popen", return_value=fake_proc):
        mgr.ensure_running(duration=30, slomo=4)

    with patch("subprocess.Popen") as mock_popen:
        result = mgr.ensure_running(duration=60, slomo=8)
        assert mock_popen.call_count == 0, "进程已在跑时不应再次 Popen"

    assert result["ok"] is True
    assert result["already_running"] is True
    assert result["pid"] == 999


def test_ensure_running_restarts_when_process_dead(tmp_path):
    """进程已死(poll 返回非 None)时 ensure_running 应重新拉起。"""
    mgr = _make_manager(tmp_path)

    dead_proc = MagicMock()
    dead_proc.pid = 111
    dead_proc.poll.return_value = 0  # 已退出

    new_proc = MagicMock()
    new_proc.pid = 222
    new_proc.poll.return_value = None

    with patch("subprocess.Popen", return_value=dead_proc):
        mgr.ensure_running(duration=30, slomo=4)

    mgr._process.poll.return_value = 0  # 模拟进程死亡

    with patch("subprocess.Popen", return_value=new_proc):
        result = mgr.ensure_running(duration=60, slomo=8)

    assert result["ok"] is True
    assert result["started"] is True
    assert result["pid"] == 222


def test_start_always_kills_and_restarts(tmp_path):
    """start (非 ensure_running) 应总是杀旧进程再重启, 保持向后兼容。"""
    mgr = _make_manager(tmp_path)

    old_proc = MagicMock()
    old_proc.pid = 111
    old_proc.poll.return_value = None

    new_proc = MagicMock()
    new_proc.pid = 222
    new_proc.poll.return_value = None

    with patch("subprocess.Popen", return_value=old_proc):
        mgr.start(duration=30, slomo=4)

    with patch("os.killpg"):  # 拦截真实 kill 系统调用
        with patch("subprocess.Popen", return_value=new_proc):
            result = mgr.start(duration=60, slomo=8)

    assert result["ok"] is True
    assert result["pid"] == 222, "start 应杀旧进程(pid=111)并替换为新进程(pid=222)"


if __name__ == "__main__":
    passed = 0
    total = 0
    for test in [
        test_ensure_running_starts_when_no_process,
        test_ensure_running_reuses_when_already_running,
        test_ensure_running_restarts_when_process_dead,
        test_start_always_kills_and_restarts,
    ]:
        total += 1
        tmp_path = Path(tempfile.mkdtemp(prefix="uav_sim_mgr_test_"))
        try:
            test(tmp_path)
            print(f"  PASS  {test.__name__}")
            passed += 1
        except Exception as exc:
            import traceback
            print(f"  FAIL  {test.__name__}: {exc}")
            traceback.print_exc()
        finally:
            shutil.rmtree(tmp_path, ignore_errors=True)
    print(f"\n{passed}/{total} passed")
    sys.exit(0 if passed == total else 1)
