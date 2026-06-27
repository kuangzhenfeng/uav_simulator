#!/usr/bin/env python3
import sys

import validation


def _agent(model="Agri_AG20"):
    return {"model": model, "initPos": [0, 0, 100], "mode": "Once", "waypoints": []}


def test_valid_minimal_dto():
    ok, err = validation.validate_scenario_dto({"fleet": [_agent()], "obstacles": []})

    assert ok, err


def test_rejects_empty_fleet():
    ok, _ = validation.validate_scenario_dto({"fleet": []})

    assert not ok


def test_rejects_17_agents():
    ok, _ = validation.validate_scenario_dto({"fleet": [_agent()] * 17})

    assert not ok


def test_rejects_invalid_model():
    ok, _ = validation.validate_scenario_dto({"fleet": [_agent("UnknownModel")]})

    assert not ok


def test_rejects_invalid_mission_mode():
    agent = _agent()
    agent["mode"] = "InvalidMode"
    ok, _ = validation.validate_scenario_dto({"fleet": [agent]})

    assert not ok


def test_rejects_65_obstacles():
    obstacle = {"type": "Box", "center": [0, 0, 0], "movement": "Static"}
    ok, _ = validation.validate_scenario_dto({"fleet": [_agent()], "obstacles": [obstacle] * 65})

    assert not ok


def test_rejects_negative_slomo():
    ok, _ = validation.validate_scenario_dto({"fleet": [_agent()], "sim": {"slomo": -1}})

    assert not ok


def test_accepts_all_valid_models():
    for model in validation.VALID_MODELS:
        ok, err = validation.validate_scenario_dto({"fleet": [_agent(model)], "obstacles": []})
        assert ok, f"{model}: {err}"


def test_accepts_all_obstacle_types():
    for obstacle_type in validation.VALID_OBS_TYPES:
        ok, err = validation.validate_scenario_dto({
            "fleet": [_agent()],
            "obstacles": [{"type": obstacle_type, "center": [0, 0, 0], "movement": "Static"}],
        })
        assert ok, f"{obstacle_type}: {err}"


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
