#!/usr/bin/env python3
# Tools/vis/control_proxy.py - 端口保活策略测试
#
# 验证 forward_to_control 在不同失败模式下的 control_port 处理:
#   - 瞬态请求失败(端口已知 + port.json 仍在) -> 保留端口, 允许后续重试
#   - 真实离线(port.json 消失) -> 清空端口, 标记不可用
#
# 运行: cd Tools/vis && python3 test_control_proxy.py

import importlib.util
import json
import os
import sys
import tempfile
import shutil
import types
import urllib.error
from pathlib import Path
from unittest.mock import patch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

_spec = importlib.util.spec_from_file_location("vis_server", os.path.join(HERE, "server.py"))
vis_server = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(vis_server)

forward_to_control = vis_server.forward_to_control
State = vis_server.State


def _make_state(project_root, port):
    """构造最小 State mock: control_port 已知, control_available=True。"""
    args = types.SimpleNamespace()
    args.project = str(project_root)
    args.idle_timeout = 5.0
    args.poll_interval = 0.5

    ndjson = project_root / "telemetry.ndjson"
    ndjson.touch()
    state = State(args, str(ndjson), str(project_root))
    state.control_port = port
    state.control_available = True
    state.last_control_refresh = 0.0
    return state


def test_transient_failure_keeps_port(tmp_path):
    """端口已知 + port.json 仍在 -> ConnectionError 不应清空 control_port。"""
    project = Path(tmp_path)
    port_file_dir = project / "Saved" / ".uav-ctrl"
    port_file_dir.mkdir(parents=True, exist_ok=True)
    port = 8770
    with open(port_file_dir / "port.json", "w") as f:
        json.dump({"port": port}, f)

    state = _make_state(project, port)
    assert state.control_port == port

    def fake_urlopen(req, timeout):
        raise urllib.error.URLError("transient connection refused")

    with patch("urllib.request.urlopen", side_effect=fake_urlopen):
        ok, result = forward_to_control(state, "/status", None)

    assert ok is False, "请求失败应返回 ok=False"
    assert state.control_port == port, (
        f"瞬态失败不应清空端口, 期望 {port}, 实际 {state.control_port}"
    )


def test_port_file_gone_clears_port(tmp_path):
    """port.json 消失 -> 真实离线, 应清空 control_port。"""
    project = Path(tmp_path)
    port = 8770
    # 不创建 port.json, resolve_control_port 会返回 None

    state = _make_state(project, port)
    assert state.control_port == port

    def fake_urlopen(req, timeout):
        raise urllib.error.URLError("connection refused")

    with patch("urllib.request.urlopen", side_effect=fake_urlopen):
        ok, result = forward_to_control(state, "/status", None)

    assert ok is False
    assert state.control_port is None, (
        "port.json 消失应清空端口, 标记 UE 真实离线"
    )


if __name__ == "__main__":
    passed = 0
    total = 0
    for test in [test_transient_failure_keeps_port, test_port_file_gone_clears_port]:
        total += 1
        tmp_path = Path(tempfile.mkdtemp(prefix="uav_vis_ctrl_test_"))
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
