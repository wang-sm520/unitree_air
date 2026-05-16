# elf3_29dof velocity_env_cfg.py 参数调整日志

追踪配置：`velocity_env_cfg.py` + `agents_cfg.py`（Unitree-Elf3-29dof-Velocity 任务）

部署目标：bxi `amp.py`（`HumanoidGaitPolicyLite`）—— 接口契约已对齐（见 [bxi 部署契约](../../../../../../scripts/diag_elf3_joint_order.py) 诊断脚本）

---

## 2026-05-15 调整 #0：初始版本（基于 air_lj 移植）

**初始 cfg：**
- Cmd 范围：`lin_vel_x=(-0.1, 0.1) → curriculum → (-0.5, 0.5)`、`lin_vel_y=(-0.1, 0.1) → (-0.3, 0.3)`、`ang_vel_z=(0, 0)`
- `track_lin_vel_xy.weight = 1.0`，`track_ang_vel_z.weight = 0.5`
- `action_rate.weight = -0.15`
- `lin_vel_cmd_levels` curriculum 启用
- PPO 用共享 `BasePPORunnerCfg`（`entropy_coef=0.01`、`init_noise_std=1.0`）

**关键契约项（按 bxi amp.py 要求）：**
- `action_scale`：29 项 dict，含 `l_hip_z=0.154` vs `r_hip_z=0.213` 不对称
- `default_joint_pos`：`l_shoulder_x=+0.2` / `r_shoulder_x=-0.2` 不对称
- `kp/kd`：waist/legs/feet/arms/wrist 5 组（数值见 `assets/robots/unitree.py` 的 `ELF3_29DOF_CFG`）
- Obs 顺序：`ang_vel(3)|grav(3)|cmd(3)|q_rel(29)|qd(29)|last_action(29) = 96`，`history_length=10`，scale 全 1.0

---

## 2026-05-16 调整 #1（用户操作）：扩宽 cmd 范围

| 参数 | 调整前 | 调整后 | 原因 |
|---|---|---|---|
| `CommandsCfg.ranges.lin_vel_x` | (-0.1, 0.1) | **(-0.6, 1.0)** | 让 policy 从一开始就被要求走，不靠 curriculum 慢慢扩 |
| `CommandsCfg.ranges.lin_vel_y` | (-0.1, 0.1) | **(-0.5, 0.5)** | 同上 |
| `CommandsCfg.ranges.ang_vel_z` | (0, 0) | **(-1.0, 1.0)** | 训出转向能力 |
| `CommandsCfg.limit_ranges` | 同 `ranges` 设置 | 同 `ranges`（即 ranges==limit_ranges） | curriculum 实际不起作用了 |

**训练结果（2026-05-15_21-16-07，跑到 iter 73264/208600）：**

| 指标 | 值 | 解读 |
|---|---|---|
| `Mean reward` | ~26-35 | 平稳但低 |
| `track_lin_vel_xy` Episode reward | **0.45**（在 iter 73k 时） | 不到满分的一半；曾在 iter 58k 时为 0.68 |
| `error_vel_xy` | **0.997 m/s** | 远大于 cmd 范围 |
| `Mean action std` | **0.22** | 探索已塌缩 |
| `Mean episode length` | **1000**（满） | 从不摔倒 |
| `time_out` termination | 99.95% | 几乎所有 reset 都是 timeout |
| `Curriculum/lin_vel_cmd_levels` | 1.0（满） | curriculum 顶到天花板但 policy 跟不上 |
| `gait` reward | 0.49 | 步态相位 reward 在 fire |
| `feet_clearance` reward | 0.97 | 脚抬得高 |

**诊断：** Policy 学到的是**抬脚原地踏步**——`gait`/`feet_clearance` 满分，但 `error_vel_xy ≈ 1.0` 说明没有水平位移。`Mean action std=0.22` 表明探索能力消失，policy 已经"自信地"陷入"站住+轻微抬脚"的局部最优。

**部署侧观察：**
- 在 bxi `amp.py` sim 中：因为 obs 略 OOD（sim 跟训练侧动力学有小差异），policy 输出震荡 → 视觉上"乱动"
- 在 IsaacLab play 中：完全静止（policy 在它自己训练的 env 里就是站着不动）

---

## 2026-05-17 调整 #2：破局原地不动局部最优

**目标：** 让 policy 真正学会走路，逃出"站住拿满分"的局部最优。

**参数变更：**

| 文件 | 参数 | 调整前 | 调整后 | 原因 |
|---|---|---|---|---|
| `velocity_env_cfg.py:262` | `track_lin_vel_xy.weight` | 1.0 | **3.0** | 当前 track 0.45 vs `action_rate` -0.43，惩罚几乎抵消跟踪奖励；拉高 3 倍让"动起来"边际收益变正 |
| `velocity_env_cfg.py:266` | `track_ang_vel_z.weight` | 0.5 | **1.5** | 同上（转向也要鼓励） |
| `velocity_env_cfg.py:276` | `action_rate.weight` | -0.15 | **-0.05** | 把惩罚降到 1/3，给动作更大自由度 |
| `velocity_env_cfg.py:434` | `lin_vel_cmd_levels` curriculum | 启用 | **注释掉** | ranges==limit_ranges 后这个 curriculum 已经名存实亡，可能还在内部干扰 |
| `agents_cfg.py`（新建） | `Elf3PPORunnerCfg.entropy_coef` | 0.01 | **0.025** | 之前 `action_std` 在 22k iter 就塌到 0.22；更高的 entropy bonus 防止过早塌缩 |
| `agents_cfg.py` | `init_noise_std` | 1.0 | 1.0（保持） | 不变，靠 entropy_coef 控制收敛速度 |
| `__init__.py:12` | `rsl_rl_cfg_entry_point` | 共享 `BasePPORunnerCfg` | **指向 elf3 专属 `Elf3PPORunnerCfg`** | 不污染 air_lj/g1/h1 等其他机器人的 PPO 配置 |

**改完不 resume，从零开始训。** 旧的 26900 iter checkpoint 已经塌进局部最优，权重恢复不了。

**成功指标（iter 500 / 2000 / 5000 各看一次 tensorboard）：**

- [ ] `Mean action std` 保持 > 0.5（之前一直塌到 0.22）
- [ ] `error_vel_xy` 到 iter 5000 时 < 0.5（开始有意义的跟踪）
- [ ] `Episode_Termination/time_out` < 0.99（机器人开始有摔倒/学步态）
- [ ] `Mean episode length` 不再固定 1000

如果 iter 5000 三项都没出现 → 这套激进 cfg 也没用，下一步要：
- entropy_coef 继续拉到 0.05+
- 去掉更多 penalty（比如 `joint_pos`、`joint_deviation_*`、`flat_orientation_l2` 的权重）
- 或者完全重新设计 reward 结构（参考 bx_lab_amp 的 AMP 路线）

**待填：** 训练完成后回来填实际指标曲线和部署结果。

---
