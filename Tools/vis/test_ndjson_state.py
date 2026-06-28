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


def test_feed_meta_sets_scenario():
    state = NdjsonState()

    state.feed({"type": "meta", "t": 0, "scenario": "Demo"})

    assert state.snapshot()["scenario"] == "Demo"


def test_feed_reload_increments_epoch():
    state = NdjsonState()

    state.feed({"type": "reload", "t": 0})
    state.feed({"type": "reload", "t": 1})

    assert state.snapshot()["reloadEpoch"] == 2


def test_feed_wind_config_and_frame_wind_sample():
    state = NdjsonState()

    state.feed({"type": "wind_config", "t": 0, "windType": "Constant", "steady": [300, 0, 0]})
    state.feed({"type": "frame", "t": 1, "agents": [], "wind": [200, 50, 0]})

    wind = state.snapshot()["wind"]
    assert wind["config"]["type"] == "Constant"
    assert wind["config"]["steadyMs"] == [0.0, 0.0, -3.0]
    assert len(wind["samples"]) == 1


def test_feed_spawn_creates_agent_meta():
    state = NdjsonState()

    state.feed({"type": "spawn", "t": 0, "agent": 0, "model": "AG-20", "maxVelCm": 1200, "collisionRadiusCm": 100, "pos": [0, 0, 200]})

    agent = state.snapshot()["agents"][0]
    assert agent["id"] == 0
    assert agent["model"] == "AG-20"
    assert agent["maxVelMs"] == 12.0
    assert agent["collisionRadiusM"] == 1.0
    assert agent["initPos"] == [0.0, 2.0, 0.0]


def test_feed_static_and_dynamic_obstacles():
    state = NdjsonState()

    state.feed({"type": "obstacle", "id": "box", "oType": 1, "center": [100, 200, 300], "extents": [50, 100, 150], "actor": "BoxActor", "dynamic": False})
    state.feed({"type": "obstacle", "id": "dyn", "oType": 0, "center": [0, 0, 0], "extents": [25, 25, 25], "dynamic": True})

    snapshot = state.snapshot()
    assert snapshot["dynamicActorCount"] == 1
    assert len(snapshot["obstacles"]) == 1
    assert snapshot["obstacles"][0]["type"] == "Box"
    assert snapshot["obstacles"][0]["center"] == to_web(100, 200, 300)


def test_feed_waypoint_orders_by_index():
    state = NdjsonState()

    state.feed({"type": "waypoint", "idx": 1, "pos": [100, 0, 0]})
    state.feed({"type": "waypoint", "idx": 0, "pos": [0, 0, 0]})

    assert state.snapshot()["waypoints"] == [to_web(0, 0, 0), to_web(100, 0, 0)]


def test_feed_frame_appends_trace_and_series():
    state = NdjsonState()

    state.feed({"type": "frame", "t": 1.0, "agents": [{"id": 0, "pos": [100, 200, 300], "vel": [300, 400, 0], "ctrl": 2, "clearance": 150}]})

    snapshot = state.snapshot()
    agent = snapshot["agents"][0]
    assert agent["trace"] == [{"t": 1.0, "pos": to_web(100, 200, 300), "speed": 5.0, "ctrl": 2}]
    assert snapshot["series"]["speedMs"][0]["points"] == [[1.0, 5.0]]
    assert snapshot["series"]["clearanceM"][0]["points"] == [[1.0, 1.5]]


def test_feed_frame_ignores_clearance_sentinel():
    state = NdjsonState()

    state.feed({"type": "frame", "t": 1.0, "agents": [{"id": 0, "pos": [0, 0, 0], "vel": [0, 0, 0], "clearance": 3.4028234663852886e38}]})

    assert state.snapshot()["series"]["clearanceM"] == []


def test_feed_metrics_populates_summary():
    state = NdjsonState()

    state.feed({"type": "metrics", "agent": 0, "speedRatio": 0.8765, "lowSpeedDur": 2.123, "maxDev": 10.456, "maxRoll": 25.4, "maxPitch": 15.2, "instabTime": 1.234, "stuck": 1, "forceComplete": 0})

    summary = state.snapshot()["summary"][0]
    assert summary["speedRatio"] == 0.876
    assert summary["lowSpeedDur"] == 2.12
    assert summary["stuck"] == 1


def test_feed_event_appends_event_detail():
    state = NdjsonState()

    state.feed({"type": "event", "t": 5.0, "agent": 0, "event": "Crash", "pos": [100, 200, 300], "crashed": True})

    event = state.snapshot()["events"][0]
    assert event["type"] == "Crash"
    assert event["agent"] == 0
    assert "crashed=True" in event["detail"]


def test_feed_verdict_sets_final_and_timeline():
    state = NdjsonState()

    state.feed({"type": "verdict", "t": 10.0, "passed": True, "reached": 3, "total": 3, "clearanceCm": 250, "lateralDevCm": 100, "elapsedSec": 10.5, "final": True, "collided": False, "failures": []})

    verdict = state.snapshot()["verdict"]
    assert verdict["final"] == "PASS"
    assert verdict["reached"] == "3/3"
    assert verdict["clearanceM"] == 2.5
    assert state.has_final


def test_feed_verdict_ignores_clearance_sentinel():
    state = NdjsonState()

    state.feed({"type": "verdict", "t": 10.0, "passed": True, "reached": 1, "total": 1, "clearanceCm": 3.4028234663852886e38, "lateralDevCm": 0, "elapsedSec": 1.0, "final": True})

    assert state.snapshot()["verdict"]["clearanceM"] is None


def test_feed_traj_types_populate_future_fields():
    state = NdjsonState()

    state.feed({"type": "traj_opt", "t": 1, "agent": 0, "pts": [[0, 0, 0], [100, 0, 0]]})
    state.feed({"type": "traj_plan", "t": 1, "agent": 0, "pts": [[0, 0, 0], [0, 100, 0]]})
    state.feed({"type": "traj_nmpc", "t": 1, "agent": 0, "pts": [[0, 0, 0, 0.5], [0, 0, 100, 1.5]]})

    agent = state.snapshot()["agents"][0]
    assert agent["futureOpt"] == [to_web(0, 0, 0), to_web(100, 0, 0)]
    assert agent["futurePlan"] == [to_web(0, 0, 0), to_web(0, 100, 0)]
    assert agent["futureNmpc"] == [[0.0, 0.0, 0.0, 0.5], [0.0, 1.0, 0.0, 1.5]]


def test_missing_agent_traj_is_skipped():
    state = NdjsonState()

    state.feed({"type": "traj_opt", "t": 1, "pts": [[0, 0, 0], [100, 0, 0]]})

    assert state.snapshot()["agents"] == []


def test_unknown_type_does_not_change_snapshot_schema():
    state = NdjsonState()

    state.feed({"type": "unknown", "t": 1, "value": 123})

    snapshot = state.snapshot()
    assert {"scenario", "duration_ms", "reloadEpoch", "agents", "obstacles", "dynamicActorCount", "waypoints", "wind", "verdict", "summary", "events", "series"}.issubset(snapshot)


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
