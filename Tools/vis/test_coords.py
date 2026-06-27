#!/usr/bin/env python3
import sys

import coords


def test_to_web_identity_zero():
    assert coords.to_web(0, 0, 0) == [0.0, 0.0, 0.0]


def test_to_web_cm_to_m():
    # UE [100,0,200] = 前1m,上2m -> web: x=右(Y_ue)=0, y=上(Z_ue)=2, z=-前(X_ue)=-1
    assert coords.to_web(100, 0, 200) == [0.0, 2.0, -1.0]


def test_to_web_axis_flip():
    # UE [0,500,0] = 右5m -> web: x=5(右), y=0, z=0
    assert coords.to_web(0, 500, 0) == [5.0, 0.0, 0.0]


def test_size_web_positive_only():
    # size_web 取绝对值,轴序与新 to_web 一致: [|Y|, |Z|, |X|]
    assert coords.size_web(-100, 200, -300) == [2.0, 3.0, 1.0]


def test_obstacle_type_mapping():
    assert coords.obstacle_type(0) == "Sphere"
    assert coords.obstacle_type(1) == "Box"
    assert coords.obstacle_type(2) == "Cylinder"
    assert coords.obstacle_type(3) == "Custom"
    assert coords.obstacle_type(99) == "Box"


def test_roundtrip_to_web_and_web_to_ue_formula():
    ue = [300, -200, 150]
    web = coords.to_web(*ue)
    # 逆映射: X_ue=-web_z*100, Y_ue=web_x*100, Z_ue=web_y*100
    back = [-web[2] * 100, web[0] * 100, web[1] * 100]

    for index, value in enumerate(back):
        assert abs(value - ue[index]) < 0.01


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
