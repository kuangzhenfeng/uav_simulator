#!/usr/bin/env python3
# Tools/vis/server.py 解析逻辑的单元测试。
# 验证三类未来轨迹行的解析、坐标变换、覆盖式快照、向后兼容。
# 运行: cd Tools/vis && python -m pytest test_server.py -q   (或 python test_server.py)

import os
import sys
import json
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# server.py 作为模块导入
import importlib.util
_spec = importlib.util.spec_from_file_location("vis_server", os.path.join(os.path.dirname(__file__), "server.py"))
vis_server = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(vis_server)

NdjsonState = vis_server.NdjsonState
to_web = vis_server.to_web
validate_scenario_dto = vis_server.validate_scenario_dto
SCHEMA = vis_server.SCHEMA
_default_random_obstacles = vis_server._default_random_obstacles

def _feed_lines(state, lines):
    """逐行喂 JSON 对象给 state.feed()"""
    for ln in lines:
        state.feed(json.loads(ln))


def _t(opt=None, plan=None, nmpc=None, agent=0, t=1.0):
    """构造未来轨迹行"""
    out = []
    if opt is not None:
        out.append({"type": "traj_opt", "t": t, "agent": agent, "valid": True, "pts": opt})
    if plan is not None:
        out.append({"type": "traj_plan", "t": t, "agent": agent, "pts": plan})
    if nmpc is not None:
        out.append({"type": "traj_nmpc", "t": t, "agent": agent, "pts": nmpc})
    return [json.dumps(o) for o in out]


def test_traj_opt_parsed_and_transformed():
    """traj_opt 行应解析为 futureOpt，坐标经 to_web 变换（cm->m + 坐标系翻转）"""
    s = NdjsonState()
    _feed_lines(s, [
        json.dumps({"type": "spawn", "t": 0.0, "agent": 0, "model": "AG-20",
                    "maxVelCm": 1200, "collisionRadiusCm": 100, "pos": [0, 0, 200]}),
    ])
    _feed_lines(s, _t(opt=[[0, 0, 200], [1000, 0, 200]], agent=0))

    snap = s.snapshot()
    a = snap["agents"][0]
    assert "futureOpt" in a, "agent 缺 futureOpt 字段"
    assert len(a["futureOpt"]) == 2

    # 坐标变换: UE (x=0,y=0,z=200) -> web。to_web: x/100, z/100, -y/100
    p0 = a["futureOpt"][0]
    expected = to_web(0, 0, 200)
    assert abs(p0[0] - expected[0]) < 1e-6, f"x 变换错 {p0} vs {expected}"
    assert abs(p0[1] - expected[1]) < 1e-6, f"y 变换错 {p0} vs {expected}"
    assert abs(p0[2] - expected[2]) < 1e-6, f"z 变换错 {p0} vs {expected}"


def test_traj_plan_parsed():
    """traj_plan 行应解析为 futurePlan"""
    s = NdjsonState()
    _feed_lines(s, [json.dumps({"type": "spawn", "t": 0.0, "agent": 0,
                                "model": "M", "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]})])
    _feed_lines(s, _t(plan=[[0, 0, 0], [500, 0, 0], [1000, 0, 0]], agent=0))

    a = s.snapshot()["agents"][0]
    assert len(a["futurePlan"]) == 3


def test_traj_nmpc_with_cost():
    """traj_nmpc 行应解析为 futureNmpc，保留 cost（第4元素）"""
    s = NdjsonState()
    _feed_lines(s, [json.dumps({"type": "spawn", "t": 0.0, "agent": 0,
                                "model": "M", "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]})])
    _feed_lines(s, _t(nmpc=[[0, 0, 0, 0.5], [1000, 0, 0, 5.0]], agent=0))

    a = s.snapshot()["agents"][0]
    assert len(a["futureNmpc"]) == 2
    assert len(a["futureNmpc"][0]) == 4, "nmpc 点应含 cost"
    assert abs(a["futureNmpc"][0][3] - 0.5) < 1e-6, "cost 值应保留"
    assert abs(a["futureNmpc"][1][3] - 5.0) < 1e-6


def test_snapshot_overwrite():
    """新未来轨迹行覆盖旧快照（区别于历史 trace 的累积式）"""
    s = NdjsonState()
    _feed_lines(s, [json.dumps({"type": "spawn", "t": 0.0, "agent": 0,
                                "model": "M", "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]})])
    _feed_lines(s, _t(opt=[[0, 0, 0], [1000, 0, 0]], agent=0, t=1.0))
    _feed_lines(s, _t(opt=[[2000, 0, 0], [3000, 0, 0]], agent=0, t=2.0))

    a = s.snapshot()["agents"][0]
    # 覆盖式：只保留最新一份
    assert len(a["futureOpt"]) == 2
    assert abs(a["futureOpt"][0][0] - to_web(2000, 0, 0)[0]) < 1e-6, "应被最新行覆盖"


def test_unknown_type_ignored():
    """未知 type 应安全忽略，不抛异常、不影响已知字段"""
    s = NdjsonState()
    _feed_lines(s, [json.dumps({"type": "spawn", "t": 0.0, "agent": 0,
                                "model": "M", "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]})])
    _feed_lines(s, _t(opt=[[0, 0, 0], [1000, 0, 0]], agent=0))
    # 喂入未知 type
    s.feed({"type": "some_future_type", "t": 3.0, "data": "whatever"})

    snap = s.snapshot()
    a = snap["agents"][0]
    assert len(a["futureOpt"]) == 2, "未知 type 不应破坏已知字段"


def test_multi_agent_isolation():
    """多 agent 的未来轨迹按 id 独立关联，互不错乱"""
    s = NdjsonState()
    _feed_lines(s, [
        json.dumps({"type": "spawn", "t": 0.0, "agent": 0, "model": "M",
                    "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]}),
        json.dumps({"type": "spawn", "t": 0.0, "agent": 1, "model": "M",
                    "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [500, 500, 0]}),
    ])
    _feed_lines(s, _t(opt=[[0, 0, 0], [1000, 0, 0]], agent=0))
    _feed_lines(s, _t(opt=[[500, 500, 0], [1500, 500, 0]], agent=1))

    agents = {a["id"]: a for a in s.snapshot()["agents"]}
    assert len(agents[0]["futureOpt"]) == 2
    assert len(agents[1]["futureOpt"]) == 2
    # agent0 起点应为 0,0,0 变换后
    assert abs(agents[0]["futureOpt"][0][0]) < 1e-6
    # agent1 起点应为 500,500,0 变换后
    exp1 = to_web(500, 500, 0)
    assert abs(agents[1]["futureOpt"][0][0] - exp1[0]) < 1e-6


def test_missing_agent_field_skipped():
    """缺 agent 字段的未来轨迹行应被跳过，不崩溃"""
    s = NdjsonState()
    s.feed({"type": "traj_opt", "t": 1.0, "pts": [[0, 0, 0], [1, 1, 1]]})
    # 不应抛异常，且不应产生任何 agent
    assert s.snapshot()["agents"] == []


# ==================== schema / 预设库 ====================

def test_schema_enums_consistent():
    """schema 枚举应与 validate_scenario_dto 的合法集合一致（单一数据源契约）"""
    # 模型
    schema_models = {m["id"] for m in SCHEMA["models"]}
    assert schema_models == vis_server.VALID_MODELS, "模型集合不一致"
    assert set(SCHEMA["obstacleTypes"]) == vis_server.VALID_OBS_TYPES, "障碍类型不一致"
    assert set(SCHEMA["movements"]) == set(vis_server.VALID_MOVEMENTS), "运动类型不一致"
    assert set(SCHEMA["missionModes"]) == set(vis_server.VALID_MODES), "任务模式不一致"
    # 默认配置应通过 DTO 校验
    ok, err = validate_scenario_dto(SCHEMA["defaults"])
    assert ok, f"defaults 未通过校验: {err}"


def test_schema_default_scenario_contract():
    defaults = SCHEMA["defaults"]
    lead = defaults["fleet"][0]

    assert lead["initPos"][2] >= 5, "默认无人机起飞高度必须不低于 5m"
    for index, waypoint in enumerate(lead.get("waypoints", [])):
        assert waypoint["pos"][2] >= 5, f"默认航点 {index} 高度必须不低于 5m"

    obstacles = defaults.get("obstacles", [])
    assert len(obstacles) >= 10, "默认场景必须包含不少于 10 个随机障碍物"
    centers = [tuple(obstacle["center"]) for obstacle in obstacles]
    assert len(set(centers)) == len(centers), "默认随机障碍物中心点不能重复"


def test_default_random_obstacles_deterministic():
    a = _default_random_obstacles(seed=42)
    b = _default_random_obstacles(seed=42)
    assert a == b, "相同 seed 必须产生相同障碍布局"


def test_default_random_obstacles_different_seed_differs():
    a = _default_random_obstacles(seed=1)
    b = _default_random_obstacles(seed=2)
    assert a != b, "不同 seed 应产生不同障碍布局"


def test_serve_file_handles_client_disconnect():
    """静态 JS 传输时客户端断开连接不应让 handler 线程抛出异常。"""
    class BrokenPipeWriter:
        def __init__(self, exc):
            self.exc = exc

        def write(self, _body):
            raise self.exc("client disconnected")

    for exc in (BrokenPipeError, ConnectionResetError):
        handler = vis_server.Handler.__new__(vis_server.Handler)
        handler.wfile = BrokenPipeWriter(exc)
        handler.send_response = lambda _code: None
        handler.send_header = lambda _name, _value: None
        handler.end_headers = lambda: None

        handler._serve_file("index.html", "text/html; charset=utf-8")


if __name__ == "__main__":
    # 简单自跑（无 pytest 也能执行）。

    # schema 一致性（不依赖 fixture）
    try:
        test_schema_enums_consistent()
        print("  PASS  test_schema_enums_consistent")
        passed_schema = True
    except Exception as e:
        print(f"  FAIL  test_schema_enums_consistent: {e}")
        passed_schema = False

    passed = 1 if passed_schema else 0
    total = 1

    # 其余无 fixture 测试
    simple_tests = [v for k, v in sorted(globals().items())
                    if k.startswith("test_") and callable(v)
                    and k not in ("test_schema_enums_consistent",)]
    for t in simple_tests:
        total += 1
        try:
            t()
            print(f"  PASS  {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {t.__name__}: {e}")
    print(f"\n{passed}/{total} passed")
    sys.exit(0 if passed == total else 1)
