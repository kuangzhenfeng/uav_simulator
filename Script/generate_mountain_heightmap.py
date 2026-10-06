"""生成可复现的山地 Landscape 高度图（16 位 PNG，单位为米）。"""
from pathlib import Path
import json
import numpy as np
from PIL import Image


def generate():
    size = 1009
    spacing = 4.0
    axis = (np.arange(size) - (size - 1) / 2) * spacing
    x, y = np.meshgrid(axis, axis)
    rng = np.random.default_rng(20261005)
    detail = np.zeros_like(x)
    for wavelength, amplitude in [(900, 38), (430, 22), (190, 10), (85, 4), (36, 1.4)]:
        for _ in range(3):
            angle, phase = rng.uniform(0, 2 * np.pi, 2)
            detail += amplitude / 3 * np.sin((x * np.cos(angle) + y * np.sin(angle)) * 2 * np.pi / wavelength + phase)
    west = 1050 + 210 * np.sin(y / 650) + 80 * np.sin(y / 250)
    east = 1000 + 240 * np.sin(y / 720 + 1.8)
    ridge_w = np.exp(-((x + west) / 470) ** 2) * (340 + 190 * (0.5 + 0.5 * np.sin(y / 430)))
    ridge_e = np.exp(-((x - east) / 520) ** 2) * (380 + 220 * (0.5 + 0.5 * np.cos(y / 510)))
    foothills = 65 * (np.sin(x / 510 + y / 680) ** 2)
    height = np.maximum(0, ridge_w + ridge_e + foothills + detail + 18)
    # 谷道连接起降区，使用平滑过渡消除阶梯和突变。
    valley = 170 * np.sin(y / 760)
    height *= 1 - 0.86 * np.exp(-((x - valley) / 260) ** 2)
    radius = np.sqrt(x * x + y * y)
    blend = np.clip((radius - 100) / 180, 0, 1)
    height *= blend * blend * (3 - 2 * blend)
    height *= 650 / height.max()
    # UE: (value - 32768) * ZScale / 128 厘米，ZScale=300。
    encoded = np.rint(32768 + height * 100 * 128 / 300).astype(np.uint16)
    output = Path(__file__).resolve().parents[1] / 'Content/Environment/Terrain/Source'
    output.mkdir(parents=True, exist_ok=True)
    Image.fromarray(encoded).save(output / 'Mountain_1009.png')
    metadata = dict(seed=20261005, resolution=size, spacing_m=spacing, extent_m=4032,
                    height_range_m=[0, 650], landscape_scale=[400, 400, 300],
                    landscape_location_cm=[-201600, -201600, 0], launch_radius_m=100)
    (output / 'Mountain_1009.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    print(json.dumps(metadata))


if __name__ == '__main__':
    generate()
