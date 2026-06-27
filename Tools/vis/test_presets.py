#!/usr/bin/env python3
import shutil
import sys
import tempfile

import presets


def _dto():
    return {"fleet": [{"model": "Agri_AG20", "initPos": [0, 0, 100], "waypoints": []}]}


def test_preset_name_validation():
    assert presets.preset_path("../escape") is None
    assert presets.preset_path("has space") is None
    assert presets.preset_path("dot.json") is None
    assert presets.preset_path("valid-name") is not None


def test_preset_list_empty_dir():
    tmp_path = tempfile.mkdtemp(prefix="uav_vis_presets_")
    try:
        presets.PRESET_DIR = tmp_path
        assert presets.list_presets() == []
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_preset_write_read_delete_roundtrip():
    tmp_path = tempfile.mkdtemp(prefix="uav_vis_presets_")
    try:
        presets.PRESET_DIR = tmp_path
        ok, err = presets.write_preset("roundtrip", _dto())
        assert ok, err
        assert "roundtrip" in presets.list_presets()
        loaded = presets.read_preset("roundtrip")
        assert loaded is not None
        assert loaded["fleet"][0]["model"] == "Agri_AG20"
        deleted, _ = presets.delete_preset("roundtrip")
        assert deleted
        assert "roundtrip" not in presets.list_presets()
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


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
