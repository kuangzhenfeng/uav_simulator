#!/usr/bin/env python3
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_spec = importlib.util.spec_from_file_location("vis_server", os.path.join(HERE, "server.py"))
vis_server = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(vis_server)

NdjsonState = vis_server.NdjsonState
to_web = vis_server.to_web
dir_to_web = vis_server.dir_to_web


def test_dir_to_web_no_round():
    assert dir_to_web(1, 2, 3) == [0.02, 0.03, -0.01]


def test_feed_debug_sphere_to_web():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": [
        {"t": "sphere", "p": [100, 200, 300], "r": 25.0, "c": [255, 0, 0, 255], "d": -1, "layer": "waypoint"}
    ]})
    debug = state.snapshot()["debug"]
    assert "waypoint" in debug
    entry = debug["waypoint"][0]
    assert entry["prim"]["type"] == "sphere"
    assert entry["prim"]["pos"] == to_web(100, 200, 300)
    assert entry["prim"]["radius"] == 0.25
    assert entry["prim"]["color"] == "#ff0000"
    assert entry["expires_at"] is None


def test_feed_debug_line_and_arrow():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": [
        {"t": "line", "a": [0, 0, 0], "b": [100, 0, 0], "c": [0, 255, 0, 255], "th": 3.0, "d": -1, "layer": "path"},
        {"t": "arrow", "a": [0, 0, 0], "b": [50, 0, 0], "sz": 20.0, "c": [0, 255, 255, 255], "d": -1, "layer": "velocity"}
    ]})
    debug = state.snapshot()["debug"]
    assert debug["path"][0]["prim"]["a"] == to_web(0, 0, 0)
    assert debug["path"][0]["prim"]["b"] == to_web(100, 0, 0)
    assert debug["velocity"][0]["prim"]["type"] == "arrow"


def test_feed_debug_box_and_point_and_text():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": [
        {"t": "box", "p": [0, 0, 0], "e": [50, 50, 100], "q": [0, 0, 0, 1], "c": [255, 0, 0, 255], "d": -1, "layer": "obstacle"},
        {"t": "point", "p": [10, 10, 10], "sz": 8.0, "c": [255, 255, 0, 255], "d": -1, "layer": "waypoint"},
        {"t": "text", "p": [10, 10, 10], "s": "Error: 42.1", "c": [255, 255, 255, 255], "d": 0.5, "layer": "label"}
    ]})
    debug = state.snapshot()["debug"]
    assert debug["obstacle"][0]["prim"]["type"] == "box"
    assert debug["obstacle"][0]["prim"]["extents"] == [0.5, 0.5, 1.0]
    assert debug["waypoint"][0]["prim"]["type"] == "point"
    assert debug["label"][0]["prim"]["text"] == "Error: 42.1"


def test_transient_prim_expires():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": [
        {"t": "sphere", "p": [0, 0, 0], "r": 10, "c": [255, 0, 0, 255], "d": 1.0, "layer": "transient_test"}
    ]})
    state._touch_t(1.5)
    assert len(state.snapshot()["debug"]["transient_test"]) == 1
    state._touch_t(2.5)
    assert len(state.snapshot()["debug"]["transient_test"]) == 0


def test_persistent_prim_survives():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": [
        {"t": "sphere", "p": [0, 0, 0], "r": 10, "c": [255, 0, 0, 255], "d": -1, "layer": "persist_test"}
    ]})
    state._touch_t(100.0)
    assert len(state.snapshot()["debug"]["persist_test"]) == 1


def test_transient_zero_duration_overwrites_previous_frame():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": [
        {"t": "text", "p": [0, 0, 0], "s": "综合: 80", "c": [255, 255, 255, 255], "d": 0.0, "layer": "hud_stability"}
    ]})
    state._touch_t(1.1)
    snap1 = state.snapshot()["debug"]["hud_stability"]
    assert len(snap1) == 1
    assert snap1[0]["prim"]["text"] == "综合: 80"

    state.feed({"type": "debug", "t": 1.2, "agent": 0, "prims": [
        {"t": "text", "p": [0, 0, 0], "s": "综合: 85", "c": [255, 255, 255, 255], "d": 0.0, "layer": "hud_stability"}
    ]})
    state._touch_t(1.2)
    snap2 = state.snapshot()["debug"]["hud_stability"]
    assert len(snap2) == 1, f"瞬时原语应覆盖旧帧，实际累积了 {len(snap2)} 条"
    assert snap2[0]["prim"]["text"] == "综合: 85"


def test_transient_zero_duration_does_not_expire_with_time():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": [
        {"t": "text", "p": [0, 0, 0], "s": "综合: 80", "c": [255, 255, 255, 255], "d": 0.0, "layer": "hud_stability"}
    ]})
    state._touch_t(100.0)
    snap = state.snapshot()["debug"]["hud_stability"]
    assert len(snap) == 1, "瞬时原语在无新帧到来时应保持显示，不应因时间过期"


def test_debug_layer_grouping():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": [
        {"t": "sphere", "p": [0, 0, 0], "r": 10, "c": [255, 0, 0, 255], "d": -1, "layer": "nmpc"},
        {"t": "sphere", "p": [100, 0, 0], "r": 10, "c": [0, 255, 0, 255], "d": -1, "layer": "nmpc"},
        {"t": "sphere", "p": [0, 100, 0], "r": 10, "c": [0, 0, 255, 255], "d": -1, "layer": "path"}
    ]})
    debug = state.snapshot()["debug"]
    assert len(debug["nmpc"]) == 2
    assert len(debug["path"]) == 1


def test_debug_in_snapshot_schema():
    state = NdjsonState()
    state.feed({"type": "debug", "t": 1.0, "agent": 0, "prims": []})
    snapshot = state.snapshot()
    assert "debug" in snapshot


def test_list_tasks_empty(tmp_path=None):
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        result = vis_server.list_tasks(td)
        assert result == []


def test_list_tasks_with_dirs():
    import tempfile, json
    with tempfile.TemporaryDirectory() as td:
        tasks_dir = os.path.join(td, "tasks")
        os.makedirs(os.path.join(tasks_dir, "20260628_120000_Demo"))
        with open(os.path.join(tasks_dir, "20260628_120000_Demo", "result.json"), "w") as f:
            json.dump({"scenario": "Demo", "verdict": "PASS", "seed": 42, "metrics": {"elapsedSec": 10.0}}, f)
        with open(os.path.join(tasks_dir, "20260628_120000_Demo", "telemetry.ndjson"), "w") as f:
            f.write('{"type":"meta","t":0,"version":2,"scenario":"Demo"}\n')
        result = vis_server.list_tasks(td)
        assert len(result) == 1
        assert result[0]["scenario"] == "Demo"
        assert result[0]["verdict"] == "PASS"
        assert result[0]["hasNdjson"] is True


if __name__ == "__main__":
    tests = [value for key, value in sorted(globals().items()) if key.startswith("test_")]
    passed = 0
    for test in tests:
        try:
            test()
            print(f"  PASS  {test.__name__}")
            passed += 1
        except Exception as exc:
            print(f"  FAIL  {test.__name__}: {exc}")
    print(f"\n{passed}/{len(tests)} passed")
    sys.exit(0 if passed == len(tests) else 1)
