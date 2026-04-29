# air_lj velocity_env_cfg.py 参数调整日志

追踪配置：`velocity_env_cfg.py`（Unitree-LJ-Velocity 任务）

---

## 2026-04-14 调整 #1：降低 bad_orientation 终止率 + 平滑动作 + 增强鲁棒性

**本次修改前的训练结果（50k 迭代）：**
- 平均奖励：30.15，回合长度：983.47
- 终止分布：time_out=95.28%, bad_orientation=4.48%, base_height=0.25%
- 主要惩罚项：action_rate=-0.4724, joint_vel_waist=-0.1328, joint_deviation_arms=-0.1223
- 跟踪误差：vel_xy=0.46, vel_yaw=0.42

**参数变更：**

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `RewardsCfg.action_rate.weight` | -0.05 | **-0.15** | 平滑动作，减少抖动（此项惩罚最大，达 -0.47） |
| `RewardsCfg.flat_orientation_l2.weight` | -5.0 | **-8.0** | 增强姿态惩罚，促使策略主动保持平衡 |
| `TerminationsCfg.bad_orientation.limit_angle` | 0.8 | **1.0** | 放宽阈值（45° → 57°），给机器人更多恢复空间 |
| `EventCfg.push_robot.velocity_range.x` | (-0.5, 0.5) | **(-0.8, 0.8)** | 增大推力扰动范围，提升抗干扰能力 |

**目标：** 将 bad_orientation 终止率从 4.48% 降低，动作更平滑，增强鲁棒性。

**状态：** 训练完成 — sim2sim 中可以 2.0 m/s 稳定奔跑，但左右步态不对称（右脚迈步大、左脚小；左手摆幅大、右手小）

---

## 2026-04-15 调整 #2：添加左右对称性奖励，改善步态和手臂摆动的左右不均

**本次修改前的训练结果（调整 #1 训练完成）：**
- sim2sim 中可以 2.0 m/s 稳定奔跑
- 主要问题：右脚迈步幅度 > 左脚，左手摆臂幅度 > 右手，左右步态不对称

**参数变更：**

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `RewardsCfg.joint_symmetry_legs` | 不存在 | **新增，weight=-0.5，joint_mirror 约束 hip_pitch/hip_yaw/knee/ankle_pitch 四对关节** | 直接惩罚左右腿关节位置差异，改善右脚迈步大、左脚小的问题 |
| `RewardsCfg.joint_symmetry_arms` | 不存在 | **新增，weight=-0.3，joint_mirror 约束 shoulder_pitch/shoulder_yaw/elbow 三对关节** | 直接惩罚左右臂关节位置差异，改善左手摆幅大、右手小的问题 |

**备注：** `joint_mirror` 计算左右关节位置的平方差作为惩罚。排除了默认值符号相反的关节（hip_roll、shoulder_roll、wrist_roll），这些关节已由 `joint_deviation_l1` 单独约束。若对称性改善不明显，可将权重逐步增大到 -1.0。

**目标：** 消除左右步态和摆臂的幅度差异，实现对称行走/奔跑。

**状态：** 训练完成 — isaacsim 中可以行走，但 mujoco 中无法行走（存在明显 sim2sim gap）
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
---

## 2026-04-16 调整 #3：扩大域随机化 + 大幅提高电机 Kp/Kd，解决 sim2sim gap

**本次修改前的训练结果（调整 #2 训练完成）：**
- isaacsim 中可以正常行走
- mujoco 中无法行走 → 存在 sim2sim gap，策略对物理参数敏感，需要更强的域随机化和更硬的电机增益

**参数变更：**

### velocity_env_cfg.py — 地形多样化

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `COBBLESTONE_ROAD_CFG.sub_terrains` | 仅 `flat`(0.5) | **`flat`(0.4) + `rough`(0.4) + `slope`(0.1) + `slope_inv`(0.1)** | 增加地形多样性：2~6cm 随机起伏 + 上下坡金字塔（~8.5°），提升策略鲁棒性 |

### velocity_env_cfg.py — EventCfg 域随机化

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `physics_material.static_friction_range` | (0.3, 1.0) | **(0.1, 1.0)** | 扩大摩擦下限至 0.1，覆盖 mujoco 可能出现的低摩擦情况 |
| `physics_material.dynamic_friction_range` | (0.3, 1.0) | **(0.1, 1.0)** | 同上 |
| `physics_material.restitution_range` | (0.0, 0.0) | **(0.0, 0.1)** | 增加反弹随机化 |
| `add_base_mass.mass_distribution_params` | (-1.0, 3.0) | **(-2.0, 4.0)** | pelvis 质量扰动范围扩大，覆盖真实质量误差 |
| `base_external_force_torque.force_range` | (0.0, 0.0) | **(30.0, 30.0)** | 新增 reset 时 30N 外力扰动 |
| `base_external_force_torque.torque_range` | (0.0, 0.0) | **(-20.0, 20.0)** | 新增 reset 时 ±20Nm 外力矩扰动 |
| `reset_base.velocity_range.x` | (0.0, 0.0) | **(-0.5, 0.5)** | 初始线速度随机化 |
| `reset_base.velocity_range.y` | (0.0, 0.0) | **(-0.3, 0.3)** | 初始线速度随机化 |
| `push_robot.interval_range_s` | (5.0, 5.0) | **(3.0, 7.0)** | 推力间隔随机化 |
| `push_robot.velocity_range.x` | (-0.8, 0.8) | **(-1.5, 1.5)** | 推力幅度增大 |
| `push_robot.velocity_range.y` | (-0.5, 0.5) | **(-1.0, 1.0)** | 推力幅度增大 |

### velocity_env_cfg.py — ObservationsCfg 噪声

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `PolicyCfg.base_ang_vel.noise` | ±0.2 | **±0.3** | 增大 IMU 角速度噪声，模拟真机/mujoco 传感器差异 |
| `PolicyCfg.joint_pos_rel.noise` | ±0.01 | **±0.03** | 增大关节位置噪声，模拟编码器精度误差 |

### unitree.py — 电机刚度 Kp

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `STIFFNESS_5020` (踝/肩) | 30 | **100** | 大幅提高刚度以更接近真实电机表现，缩小 sim2sim gap |
| `STIFFNESS_7520_14` (腰/髋 pitch/yaw) | 80 | **200** | 同上 |
| `STIFFNESS_7520_22` (髋 roll/膝) | 200 | **400** | 同上，膝/髋 roll 承重关节刚度翻倍 |
| `STIFFNESS_4010` (手腕) | 34 | **80** | 同上 |

### unitree.py — 电机阻尼 Kd

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `DAMPING_5020` | 2 | **5** | 配合 Kp 提升同步增大阻尼，保持 Kp/Kd 比例在合理范围（20:1） |
| `DAMPING_7520_14` | 5 | **10** | 同上 |
| `DAMPING_7520_22` | 5 | **15** | 同上，抑制膝/髋 roll 振荡（之前比例 40:1 偏高） |
| `DAMPING_4010` | 2 | **5** | 同上 |

**目标：** 通过扩大域随机化（地形 + 扰动 + 噪声）和提高电机增益，缩小 isaacsim → mujoco 的 sim2sim gap，使策略在 mujoco 中也能稳定行走。

**注意事项：**
- Kp 大幅提高可能导致训练初期不稳定，若早期奖励异常低或发散，可先降一档（如 STIFFNESS_7520_22 从 400 → 300）
- Kp/Kd 比例：5020=20:1, 7520_14=20:1, 7520_22=26:1, 4010=16:1，整体合理
- 地形难度提高后，curriculum 前期可能进展变慢，观察 terrain_levels 是否能顺利爬升

**状态：** 训练完成 — 存在三个问题：① 脚步不稳（步态摇晃、落脚不扎实）；② 给出前进指令时机器人斜着走（系统性侧向偏移）；③ 部分电机 Kd 超过硬件规格上限（规格要求 Kd ≤ 5，本次 STIFFNESS_5020/4010 的 Damping 为 5 刚好顶格）。根因分析：斜走由 `base_external_force_torque.force_range=(30.0, 30.0)` 在 `sample_uniform` 下退化为恒定力向量 (30,30,30) N、合力约 52N 固定偏置 pelvis 所致，策略学会歪走补偿，真机无此偏置便斜行；脚步不稳推测与高 Kp 下踝关节对扰动反应过激、以及恒定侧向力导致步态学习畸形有关

---

## 2026-04-18 调整 #4：修复 force_range 恒定力偏置 + 降低 ankle/wrist/shoulder Kp·Kd 以满足电机规格

**本次修改前的训练结果（调整 #3 训练完成）：**
- 主要问题 ①：**脚步不稳**，落脚不扎实、步态摇晃（推测高 Kp 下踝关节对扰动反应过激 + 恒定侧向力使步态学习畸形）
- 主要问题 ②：**给前进指令 (lin_vel_x > 0) 但机器人斜着走**，存在系统性 y 方向偏移
- 根因定位：`EventCfg.base_external_force_torque.force_range=(30.0, 30.0)` 实际退化为恒定力 (30, 30, 30) N（`sample_uniform(30, 30, ...)` 得三分量全 30），每次 reset 都在 pelvis 施加方向固定、大小约 52N 的力，策略被迫学会补偿 → 部署到真机/mujoco 无此力便表现为斜走；此恒定偏置同时污染了对称步态的学习，加剧脚步不稳
- 主要问题 ③：经硬件核查发现 STIFFNESS_5020（踝/肩）、STIFFNESS_4010（手腕）对应的实际电机规定 Kd ≤ 5，本次 Kd=5 刚好顶格，配合过高 Kp 存在指令超限隐患
- 定量指标（奖励/回合长度/终止率）：本次训练未记录具体数值

**参数变更：**

### velocity_env_cfg.py — EventCfg 修复恒定力偏置

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `EventCfg.base_external_force_torque.force_range` | (30.0, 30.0) | **(-15.0, 15.0)** | 改为以 0 为中心的对称采样，每个分量 ±15N 独立随机；修复恒定力偏置导致前进指令斜走的问题，同时降低幅度避免训练初期不稳定 |

### unitree.py — 电机 Kp/Kd 回调至电机规格范围内

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `STIFFNESS_5020` (踝/肩) | 100 | **50** | 配合 DAMPING_5020 回调，保持 Kp/Kd 比例 25:1 处于合理范围 |
| `STIFFNESS_4010` (手腕) | 80 | **50** | 同上，保持 25:1 比例 |
| `DAMPING_5020` | 5 | **2** | 发现电机规格限制 Kd ≤ 5，改为 2 留足余量，避免真机部署时指令被硬件限幅 |
| `DAMPING_4010` | 5 | **2** | 同上 |

**目标：** ① 消除"前进指令斜走"的系统偏移；② 保证电机 Kd 在所有型号规格允许范围内（踝/肩/腕 Kd 规格上限为 5），避免 sim2real 指令被限幅导致性能退化。

**注意事项：**
- Kp/Kd 比例变化：5020 从 20:1 → 25:1，4010 从 16:1 → 25:1（略偏硬，若出现振荡可把 Kp 再降到 40，或 Kd 提到 2.5）
- STIFFNESS_7520_14（腰/髋 pitch/yaw）、STIFFNESS_7520_22（髋 roll/膝）保持 200/400 不变，此类电机 Kd 规格允许 ≥ 10
- 地形 curriculum 与 #3 保持一致，但电机刚度下降后脚踝/手腕对扰动的恢复力变弱，关注 `bad_orientation` 终止率是否回升
- 若修复 force_range 后仍存在轻微斜走，可进一步检查 `joint_deviation_legs`（只约束 hip_roll/hip_yaw）下的 hip_roll 左右偏置

**状态：** 训练完成 — isaacsim 在 rough 地形失败率较高；mujoco 中 v=2 m/s 摔倒，仅 v=1 m/s 能勉强歪扭行走，姿态不佳。推测额外 reset 扰动过大 + 地形难度偏高，策略未能稳定收敛

---

## 2026-04-19 调整 #5：关闭 reset 扰动 + 简化地形，优先训练稳定基础步态

**本次修改前的训练结果（调整 #4 训练完成）：**
- isaacsim：rough 地形失败率较高
- mujoco：v=2 m/s 摔倒，仅 v=1 m/s 能勉强歪扭行走，姿态不佳
- 判断：reset 时 ±15N 力 + ±20Nm 力矩 + 起伏地形 + slope 金字塔叠加，超出当前策略鲁棒性上限，难以训出稳定基础步态

**参数变更：**

### velocity_env_cfg.py — 地形简化

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `COBBLESTONE_ROAD_CFG.sub_terrains` | flat(0.4) + rough(0.4) + slope(0.1) + slope_inv(0.1) | **flat(0.4) + rough(0.5)** | 移除上下坡金字塔，rough 比例 0.4 → 0.5；降低地形难度上限，先训稳平地/起伏地形上的基础行走 |

### velocity_env_cfg.py — EventCfg 关闭 reset 扰动

| 参数 | 调整前 | 调整后 | 原因 |
|------|--------|--------|------|
| `physics_material.restitution_range` | (0.0, 0.1) | **(0.0, 0.0)** | 关闭反弹随机化，减少足地接触行为的不确定性 |
| `base_external_force_torque.force_range` | (-15.0, 15.0) | **(0, 0)** | 关闭 reset 瞬间对 pelvis 的外力扰动，消除训练初期冲击 |
| `base_external_force_torque.torque_range` | (-20.0, 20.0) | **(0, 0)** | 关闭 reset 瞬间外力矩扰动，配合上项 |

**目标：** 降低训练难度下限（去除 reset 扰动 + 去除 slope 地形），让策略先学稳基础站立与行走，解决 mujoco 高速摔倒问题；后续再逐步加回扰动和复杂地形。

**注意事项：**
- `push_robot`（间歇性速度推力，±1.5 m/s x、±1 m/s y）保持不变，仍提供基本抗扰动训练
- 本轮若训稳，可分级恢复：先加回 slope 地形，再小幅加回 reset 扰动（如 ±5N 起）
- 重点观察 mujoco 下 v=2 m/s 是否仍摔倒；若仍摔倒需排查电机响应、步态周期是否与高速不匹配

**状态：** 训练中
我想要在'/home/hpf/wsm/unitree_rl/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/ai 
  r_lj/adjustment.md' 加入上一次参数调试的训练效果：测试sim2real的时候，发现当v=-0.2的时候，可以站立，并且具有一定 
  的抗扰能力，当向0趋近的时候，抗扰能力减弱 
