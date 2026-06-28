#!/usr/bin/env python3
# Tools/vis/state.py - State 类生命周期测试
#
# 重点覆盖 finished 状态机:
#   - 文件轮换(shrink)后必须复位 finished
#   - 同文件内 reload epoch 推进(新场景)必须复位 finished
#   - 一次仿真结束后, 第二次仿真不应被立即判 finished
#
# 运行: cd Tools/vis && python3 test_state_lifecycle.py

import importlib.util
import os
import sys
import tempfile
import shutil
import types
import json
import threading
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

_spec = importlib.util.spec_from_file_location("vis_server", os.path.join(HERE, "server.py"))
vis_server = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(vis_server)

State = vis_server.State


def _make_args(project_root):
    """构造最小 args mock, 仅满足 State.__init__ 需要的字段。"""
    args = types.SimpleNamespace()
    args.project = project_root
    args.idle_timeout = 5.0
    args.poll_interval = 0.5
    return args


def _write_ndjson(path, lines):
    """覆盖式写入 ndjson (每行一个 JSON 对象)。"""
    with open(path, "w", encoding="utf-8") as f:
        for obj in lines:
            f.write(json.dumps(obj) + "\n")


def _append_ndjson(path, lines):
    """追加写入 ndjson。"""
    with open(path, "a", encoding="utf-8") as f:
        for obj in lines:
            f.write(json.dumps(obj) + "\n")


# ==================== finished 状态机 ====================

def test_finished_resets_on_file_shrink(tmp_path):
    """文件被截断(新仿真覆盖写)后, State.finished 必须复位为 False。

    场景: 第一次仿真跑完写 final verdict -> finished=True;
          第二次仿真以新文件启动(覆盖写) -> finished 必须 False。
    """
    ndjson = tmp_path / "telemetry.ndjson"
    logs_dir = str(tmp_path)

    # 第一次仿真: 写入 spawn + final verdict
    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "Run1"},
        {"type": "spawn", "t": 0, "agent": 0, "model": "M",
         "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]},
        {"type": "verdict", "t": 10, "passed": True, "reached": 1, "total": 1,
         "clearanceCm": 200, "lateralDevCm": 50, "elapsedSec": 10,
         "final": True, "collided": False, "failures": []},
    ])

    state = State(_make_args(str(tmp_path)), str(ndjson), logs_dir)
    assert state.finished is True, "第一次仿真结束应 finished=True"

    # 第二次仿真: 文件被覆盖(更小)
    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "Run2"},
        {"type": "spawn", "t": 0, "agent": 0, "model": "M",
         "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]},
        # 注意: 没有 verdict, has_final 应为 False
    ])

    changed = state.poll_once()
    assert changed is True, "poll_once 应检测到文件变化"
    assert state.finished is False, (
        "文件轮换(新仿真)后 finished 必须复位为 False, 否则第二次仿真 SSE 立即 done"
    )


def test_finished_resets_on_reload_epoch_advance(tmp_path):
    """同文件内 reload epoch 推进(热重载新场景)必须复位 finished。

    场景: 第一次仿真跑完, 文件追加 verdict -> finished=True;
          UE 热重载新场景, 写入 reload 行(无文件轮换) -> finished 必须 False。
    """
    ndjson = tmp_path / "telemetry.ndjson"
    logs_dir = str(tmp_path)

    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "Run1"},
        {"type": "spawn", "t": 0, "agent": 0, "model": "M",
         "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]},
        {"type": "verdict", "t": 10, "passed": True, "reached": 1, "total": 1,
         "clearanceCm": 200, "lateralDevCm": 50, "elapsedSec": 10,
         "final": True, "collided": False, "failures": []},
    ])

    state = State(_make_args(str(tmp_path)), str(ndjson), logs_dir)
    assert state.finished is True

    # 热重载: 同文件追加 reload + 新 meta, 但不轮换
    _append_ndjson(str(ndjson), [
        {"type": "reload", "t": 11},
        {"type": "meta", "t": 11, "scenario": "Run2"},
        {"type": "frame", "t": 12, "agents": []},
    ])

    state.poll_once()
    assert state.state.reload_epoch == 1, "reload 行应推进 epoch"
    assert state.finished is False, (
        "reload epoch 推进(新场景热重载)后 finished 必须复位"
    )


def test_finished_stays_true_when_idle_after_verdict(tmp_path):
    """正常结束态: verdict 写入后 idle 超时也应保持 finished=True (回归保护)。"""
    ndjson = tmp_path / "telemetry.ndjson"
    logs_dir = str(tmp_path)

    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "Run1"},
        {"type": "verdict", "t": 10, "passed": True, "reached": 1, "total": 1,
         "clearanceCm": 200, "lateralDevCm": 50, "elapsedSec": 10,
         "final": True, "collided": False, "failures": []},
    ])

    state = State(_make_args(str(tmp_path)), str(ndjson), logs_dir)
    assert state.finished is True
    state.check_idle_finish(0.0)  # idle_timeout=0 立即触发
    assert state.finished is True, "已结束态不应被 idle 检查复位"


def test_snapshot_reports_finished_after_resets(tmp_path):
    """端到端: 两次仿真, snapshot_with_result 第二次必须返回 finished=False。"""
    ndjson = tmp_path / "telemetry.ndjson"
    logs_dir = str(tmp_path)

    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "Run1"},
        {"type": "verdict", "t": 10, "passed": True, "reached": 1, "total": 1,
         "clearanceCm": 200, "lateralDevCm": 50, "elapsedSec": 10,
         "final": True, "collided": False, "failures": []},
    ])

    state = State(_make_args(str(tmp_path)), str(ndjson), logs_dir)
    assert state.snapshot_with_result()["finished"] is True

    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "Run2"},
        {"type": "spawn", "t": 0, "agent": 0, "model": "M",
         "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]},
    ])
    state.poll_once()
    assert state.snapshot_with_result()["finished"] is False


# ==================== 行边界读取(字节切断多字节字符防护) ====================

def test_partial_line_deferred_until_newline(tmp_path):
    """文件末尾半行(无 \\n)不应被解析; 追加换行后才 feed。
    锁住 last_size 始终停在 \\n 行边界, 避免 seek 切断多字节 UTF-8 字符。"""
    ndjson = tmp_path / "telemetry.ndjson"
    line1 = json.dumps({"type": "spawn", "t": 0, "agent": 0, "model": "M",
                        "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]})
    line2 = json.dumps({"type": "spawn", "t": 1, "agent": 1, "model": "M",
                        "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]})
    with open(ndjson, "wb") as f:
        f.write((line1 + "\n").encode("utf-8"))
        f.write(line2.encode("utf-8"))  # 半行, 无换行

    state = State(_make_args(str(tmp_path)), str(ndjson), str(tmp_path))
    assert 0 in state.state.agent_meta
    assert 1 not in state.state.agent_meta
    assert state.last_size == len((line1 + "\n").encode("utf-8"))

    with open(ndjson, "ab") as f:
        f.write(b"\n")
    assert state.poll_once() is True
    assert 1 in state.state.agent_meta
    assert state.last_size == len((line1 + "\n" + line2 + "\n").encode("utf-8"))


def test_multibyte_lines_parse_without_corruption(tmp_path):
    """含中文(多字节 UTF-8)的多行 ndjson 增量 poll 时, 行边界推进保证
    每行都被正确解析, 不因 seek 落在字符中间而损坏丢帧。"""
    ndjson = tmp_path / "telemetry.ndjson"
    objs = [
        {"type": "debug", "t": float(i), "agent": 0, "prims": [
            {"t": "text", "p": [0, 0, 0], "s": "帧%d综合评分: %d" % (i, i * 10),
             "layer": "hud_stability"}]}
        for i in range(5)
    ]
    full_text = "".join(json.dumps(o, ensure_ascii=False) + "\n" for o in objs)
    with open(ndjson, "wb") as f:
        f.write(full_text.encode("utf-8"))

    state = State(_make_args(str(tmp_path)), str(ndjson), str(tmp_path))
    assert state.state.last_t == 4.0


# ==================== 并发安全: poll_once 不得重复喂同一批帧 ====================

def test_poll_once_concurrent_no_duplicate_feed(tmp_path):
    """回归: 并发 poll_once() 在文件增长后, 不得把同一段 tail 喂两次。

    根因: 原实现把 size/last_size 的读取与判定放在 self.lock 外, 仅在
    _feed_text 阶段进锁。两个 HTTP 线程可同时读到陈旧的 last_size, 都走
    "size > last_size" 分支并都 _read_tail_lines(last_size), 再依次进锁
    feed 同一份文本 -> 同一帧被 append 两次 -> speed_series/alt_series/
    clearance_series 时间轴非单调 -> uPlot 拒绝渲染(图表空白或错乱)。

    本测试用 threading.Barrier 强制两线程在 _read_tail_lines 内同时挂起,
    再让它们各自完成 feed。修复前: 帧数翻倍; 修复后: 只增 1 帧。
    """
    ndjson = tmp_path / "telemetry.ndjson"
    logs_dir = str(tmp_path)

    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "RaceTest"},
        {"type": "spawn", "t": 0, "agent": 0, "model": "M",
         "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]},
        {"type": "frame", "t": 1.0,
         "agents": [{"id": 0, "pos": [0, 0, 0], "vel": [0, 0, 0]}]},
    ])
    state = State(_make_args(str(tmp_path)), str(ndjson), logs_dir)
    initial = len(state.state.traces[0])

    # 追加一帧, 触发"size > last_size"路径
    _append_ndjson(str(ndjson), [
        {"type": "frame", "t": 2.0,
         "agents": [{"id": 0, "pos": [100, 0, 0], "vel": [100, 0, 0]}]},
    ])

    # 强制竞态: 两线程在 _read_tail_lines 内 barrier 同步, 都基于同一
    # 陈旧 last_size 读 tail, 再依次进锁 feed。
    barrier = threading.Barrier(2)
    original_read_tail = state._read_tail_lines

    def synced_read_tail(start):
        barrier.wait(timeout=2.0)
        return original_read_tail(start)

    state._read_tail_lines = synced_read_tail

    errors = []
    def worker():
        try:
            state.poll_once()
        except Exception as e:  # noqa: BLE001 - 测试线程异常捕获
            errors.append(e)

    threads = [threading.Thread(target=worker) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=5.0)

    assert not errors, f"poll_once 抛异常: {errors}"

    final = len(state.state.traces[0])
    assert final == initial + 1, (
        f"并发 poll_once 重复喂入: 期望 {initial + 1} 帧, 实际 {final} 帧 "
        f"(多了 {final - initial - 1} 份重复 -> series 时间轴会非单调)"
    )


def test_poll_once_series_time_strictly_monotonic(tmp_path):
    """端到端契约: 多次 poll_once(包括并发)后, snapshot 的 series 每条
    agent 的 points 时间戳必须严格单调递增。uPlot 要求 x 严格单调,
    任何回跳都会导致图表渲染失败。"""
    ndjson = tmp_path / "telemetry.ndjson"
    logs_dir = str(tmp_path)

    frames = [
        {"type": "frame", "t": float(i),
         "agents": [{"id": 0, "pos": [i * 100, 0, 0], "vel": [100, 0, 0]}]}
        for i in range(1, 6)
    ]
    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "MonoTest"},
        {"type": "spawn", "t": 0, "agent": 0, "model": "M",
         "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]},
    ] + frames[:2])

    state = State(_make_args(str(tmp_path)), str(ndjson), logs_dir)

    # 分批追加剩余帧, 每批后 poll; 模拟边跑边读
    for batch_start in range(2, len(frames)):
        _append_ndjson(str(ndjson), [frames[batch_start]])
        state.poll_once()

    snap = state.snapshot_with_result()
    for key in ("speedMs", "altM", "clearanceM"):
        series = snap["series"].get(key, [])
        for s in series:
            xs = [p[0] for p in s["points"]]
            for i in range(1, len(xs)):
                assert xs[i] > xs[i - 1], (
                    f"series.{key} agent={s['agent']} 时间轴非单调: "
                    f"xs[{i-1}]={xs[i-1]} >= xs[{i}]={xs[i]}"
                )


def test_non_monotonic_frames_rejected(tmp_path):
    """回归: 即使上游因竞态/文件轮换误喂了时间回退的帧, NdjsonState 必须拒绝
    把它们追加到 series/trace, 保证图表时间轴严格单调。

    场景: 正常喂 t=1,2,3 后, 人为插入 t=2(回退)和 t=1(更老)。
    修复前: series 变成 [1,2,3,2,1] 非单调 -> uPlot 图表空白/错乱。
    修复后: 回退帧被丢弃, series 保持 [1,2,3]。
    """
    ndjson = tmp_path / "telemetry.ndjson"
    logs_dir = str(tmp_path)

    _write_ndjson(str(ndjson), [
        {"type": "meta", "t": 0, "scenario": "NonMono"},
        {"type": "spawn", "t": 0, "agent": 0, "model": "M",
         "maxVelCm": 1000, "collisionRadiusCm": 100, "pos": [0, 0, 0]},
    ])
    state = State(_make_args(str(tmp_path)), str(ndjson), logs_dir)

    s = state.state
    # 正常 3 帧
    for t in (1.0, 2.0, 3.0):
        s.feed({"type": "frame", "t": t,
                "agents": [{"id": 0, "pos": [int(t*100), 0, 0], "vel": [100, 0, 0]}]})
    assert len(s.speed_series[0]) == 3

    # 人为喂入回退帧 (模拟竞态/文件轮换导致的重复喂)
    s.feed({"type": "frame", "t": 2.0,
            "agents": [{"id": 0, "pos": [200, 0, 0], "vel": [100, 0, 0]}]})
    s.feed({"type": "frame", "t": 1.0,
            "agents": [{"id": 0, "pos": [100, 0, 0], "vel": [100, 0, 0]}]})

    # 必须仍然只有 3 个点
    assert len(s.speed_series[0]) == 3, (
        f"回退帧未被拒绝: speed_series 有 {len(s.speed_series[0])} 点 (期望 3)"
    )
    xs = [p[0] for p in s.speed_series[0]]
    assert xs == [1.0, 2.0, 3.0], f"series 被污染: {xs}"

    # traces 同理
    trace_ts = [e["t"] for e in s.traces[0]]
    assert trace_ts == [1.0, 2.0, 3.0], f"trace 被污染: {trace_ts}"


if __name__ == "__main__":
    tmp_path = Path(tempfile.mkdtemp(prefix="uav_vis_state_test_"))
    try:
        tests = [
            test_finished_resets_on_file_shrink,
            test_finished_resets_on_reload_epoch_advance,
            test_finished_stays_true_when_idle_after_verdict,
            test_snapshot_reports_finished_after_resets,
            test_partial_line_deferred_until_newline,
            test_multibyte_lines_parse_without_corruption,
            test_poll_once_concurrent_no_duplicate_feed,
            test_poll_once_series_time_strictly_monotonic,
            test_non_monotonic_frames_rejected,
        ]
        passed = 0
        for test in tests:
            try:
                test(tmp_path)
                print(f"  PASS  {test.__name__}")
                passed += 1
            except Exception as exc:
                import traceback
                print(f"  FAIL  {test.__name__}: {exc}")
                traceback.print_exc()
        print(f"\n{passed}/{len(tests)} passed")
        sys.exit(0 if passed == len(tests) else 1)
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)
