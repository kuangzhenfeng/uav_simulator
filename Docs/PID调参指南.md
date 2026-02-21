# PID 调参指南

## 控制架构

```
目标位置 → [位置PID] → 期望速度 → [速度PID] → 期望加速度 → [姿态PID] → 电机推力
```

级联结构：外环（位置/速度）输出作为内环（姿态）的输入，调参顺序**从内到外**。

---

## 控制台命令

仿真运行时按 `~` 打开控制台：

| 命令 | 说明 |
|------|------|
| `pid.set <param> <value>` | 实时修改参数 |
| `pid.step [magnitude]` | 触发 Z 轴阶跃测试（默认 200cm） |
| `pid.autotune` | **一键自动调参**（见下文） |
| `pid.save` | 保存当前参数到文件 |
| `pid.reset` | 恢复默认值 |

**参数名对照：**

```
pos.kp / pos.ki / pos.kd     # 位置环
vel.kp / vel.ki / vel.kd     # 速度环
roll.kp / roll.ki / roll.kd  # Roll 姿态环
pitch.kp / pitch.ki / pitch.kd
yaw.kp / yaw.ki / yaw.kd
```

---

## 阶跃测试

```
pid.step        # 上升 200cm
pid.step 300    # 上升 300cm
```

测试完成后 HUD 右侧显示三项指标：

- **Settle Time**：调节时间（误差收敛至 5% 幅值所需秒数）
- **Overshoot**：超调量（>20% 标红）
- **Steady Err**：稳态误差（cm）

日志同步输出：`Logs/uav.log` 搜索 `[PIDTuner]`

---

## 默认参数

| 参数 | 默认值 |
|------|--------|
| `pos.kp` | 1.0 |
| `pos.ki` | 0.0 |
| `pos.kd` | 0.5 |
| `vel.kp` | 2.0 |
| `vel.ki` | 0.1 |
| `vel.kd` | 0.1 |
| `roll.kp / pitch.kp` | 0.05 |
| `roll.ki / pitch.ki` | 0.005 |
| `roll.kd / pitch.kd` | 0.05 |
| `yaw.kp` | 0.04 |
| `yaw.ki` | 0.002 |
| `yaw.kd` | 0.04 |

---

## 调参流程

### 第一步：调姿态内环（Roll/Pitch）

姿态环响应快，先调好再调外环。

```
pid.set roll.kp 0.008
pid.set pitch.kp 0.008
```

**目标**：无振荡，响应迅速。观察 HUD 的 Roll/Pitch 值是否平稳。

- Kp 过大 → 振荡
- Kd 过小 → 响应慢、超调
- Kd 过大 → 高频抖动

### 第二步：调速度环

```
pid.set vel.kp 2.0
pid.set vel.kd 0.1
```

**目标**：速度跟踪平滑，无抖动。

### 第三步：调位置环 + 阶跃测试

```
pid.set pos.kp 1.0
pid.set pos.kd 0.5
pid.step 200
```

观察 HUD 结果：

| 现象 | 调整方向 |
|------|----------|
| 超调 > 20% | 减小 `pos.kp` 或增大 `pos.kd` |
| 调节时间 > 5s | 增大 `pos.kp` |
| 稳态误差 > 10cm | 小幅增加 `pos.ki`（从 0.01 开始） |
| 持续振荡 | 减小 `pos.kp`，增大 `pos.kd` |

### 第四步：保存

```
pid.save
```

参数保存至 `Saved/Config/PIDParams.ini`。

---

## 典型问题排查

**无人机起飞后持续振荡**
→ 姿态环 Kp 过大，先 `pid.reset` 再从小值重调

**悬停时缓慢漂移**
→ 位置环 Ki 为 0 时正常，可小幅增加 `pos.ki`

**到达目标后超调严重**
→ 增大 `pos.kd` 或减小 `vel.kp`

**阶跃测试 30s 超时**
→ 位置环增益太小，增大 `pos.kp`

---

## 自动调参

```
pid.autotune
```

使用 **IMC/Lambda** 方法，共触发 2 次阶跃测试完成调参：

```
VelLoop（1次）→ PosLoop（1次）→ Done
```

**算法**：每次阶跃测试从响应曲线实时提取 FOPDT 模型参数（θ 死区时间、τ 时间常数），用公式直接计算增益：

```
λ = max(θ, τ/3)
Kp = τ / (λ + θ)
Kd = Kp × θ / 2
```

**进度跟踪**：`Logs/uav.log` 搜索 `[AutoTune]`

```
[AutoTune] Starting IMC tuning...
[AutoTune] Phase=1 theta=0.050s tau=0.820s lambda=0.273s -> Kp=2.981 Kd=0.075
[AutoTune] Phase=2 theta=0.080s tau=1.200s lambda=0.400s -> Kp=2.500 Kd=0.100
[AutoTune] Done! vel.kp=2.981 vel.kd=0.075 pos.kp=2.500 pos.kd=0.100
```

**完成后**：参数自动保存至 `Saved/Config/PIDParams_AutoTune.ini`，可用 `pid.step` 验证效果。

> 注意：自动调参仅优化位置/速度环，姿态环（roll/pitch/yaw）需手动调节。
