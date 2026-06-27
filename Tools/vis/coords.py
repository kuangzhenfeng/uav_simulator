#!/usr/bin/env python3


def to_web(x_ue, y_ue, z_ue):
    # UE 左手系(X前/Y右/Z上,cm) -> web 右手系(Y上,m)
    # 映射: web_x = Y_ue(右), web_y = Z_ue(上), web_z = -X_ue(远)
    # 俯视时 web 的"上=远=-X_ue=UE 前",与 UE 编辑器俯视一致。
    return [round(y_ue / 100.0, 3), round(z_ue / 100.0, 3), round(-x_ue / 100.0, 3)]


def size_web(x_ue, y_ue, z_ue):
    return [round(abs(y_ue) / 100.0, 3), round(abs(z_ue) / 100.0, 3), round(abs(x_ue) / 100.0, 3)]


def dir_to_web(dx_ue, dy_ue, dz_ue):
    # 方向向量变换: 与 to_web 同轴映射, 不 round (方向需精度)
    return [dy_ue / 100.0, dz_ue / 100.0, -dx_ue / 100.0]


def obstacle_type(value):
    return {0: "Sphere", 1: "Box", 2: "Cylinder", 3: "Custom"}.get(value, "Box")
