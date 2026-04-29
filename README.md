# Unitree Air — RL 训练 / 推理 / Sim2Sim

基于 [IsaacLab](https://github.com/isaac-sim/IsaacLab) + [RSL-RL](https://github.com/leggedrobotics/rsl_rl) 的 Unitree 系列机器人强化学习项目，支持 Isaac Sim 中训练与推理，并通过 MuJoCo 完成 sim2sim 部署。

## 目录结构

```
.
├── unitree_rl_lab/        # 训练 / 推理代码（基于 IsaacLab）
│   ├── scripts/rsl_rl/    # train.py / play.py
│   ├── source/.../tasks/  # 各机器人任务环境配置
│   └── unitree_rl_lab.sh  # 命令行入口
├── deploy_mujoco/         # Python 版 MuJoCo sim2sim 部署
│   ├── deploy_mujoco.py
│   └── configs/           # 各机器人部署 YAML
├── unitree_ros/           # 机器人 URDF / mesh / 描述文件
└── IsaacLab/              # 上游 IsaacLab（gitignored，需自行 clone）
```

## 关键文件位置

修改机器人 / 调参时最常打开的三个位置：

| 用途 | 路径 |
|------|------|
| **机器人模型**（URDF / mesh / MJCF XML） | `unitree_ros/robots/robot/` |
| **机器人资产配置**（IsaacLab 中注册资产、关节顺序、PD、`UNITREE_ROS_DIR` 等） | `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py` |
| **机器人训练参数**（环境 cfg、奖励权重、终止条件、域随机化） | `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/air_lj/` |

绝对路径（本机）：

```
/home/hpf/wsm/unitree_rl/unitree_ros/robots/robot
/home/hpf/wsm/unitree_rl/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py
/home/hpf/wsm/unitree_rl/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/air_lj
```

`air_lj/` 目录内主要文件：

- `velocity_env_cfg.py` — 速度跟踪环境配置（观测、奖励、终止、命令采样）
- `wsm_velocity_cfg.py` — WSM 变体速度配置
- `adjustment.md` — 训练参数调整记录

> 修改流程一般是：调机器人模型 → 同步 `unitree.py` 中关节顺序 / PD → 调 `air_lj/` 下的环境 cfg → 重新训练。

## 环境准备

### 1. 训练环境（Isaac Sim + IsaacLab）

参考 [IsaacLab 官方安装指南](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)。

```bash
# 启用 IsaacLab 的 conda 环境
conda activate env_isaaclab

# 安装本项目
cd unitree_rl_lab
./unitree_rl_lab.sh -i
```

机器人描述文件配置：编辑 `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`，把 `UNITREE_ROS_DIR` 指向本仓库的 `unitree_ros/unitree_ros`。

### 2. 部署环境（MuJoCo）

```bash
# 推荐单独的 conda 环境
conda create -n mujoco python=3.10
conda activate mujoco
pip install mujoco onnxruntime pyyaml numpy
```

## 支持的任务（Task）

| Task ID | 机器人 | 说明 |
|---------|------|------|
| `Unitree-G1-29dof-Velocity` | G1 (29dof) | 速度跟踪 |
| `Unitree-Go2-Velocity` | Go2 | 速度跟踪 |
| `Unitree-H1-Velocity` | H1 | 速度跟踪 |
| `Unitree-DT114-Velocity` | DT114 | 速度跟踪 12dof|
| `Unitree-DT114-New-Velocity` | DT114 | 改进版速度环境 12dof|
| `Unitree-DT114-WSM-Velocity` | DT114 | WSM 变体 12dof|
| `Unitree-DT114-XR-Stand` | DT114 | 站立任务 12dof|
| `Unitree-LJ-Velocity` | LJ | 速度跟踪 目前正在用的27dof|
| `Unitree-WSM-Velocity` | WSM | 速度跟踪 废弃的27dof|
| `Unitree-XR-Stand` | XR | 站立任务 |

列出当前所有任务：

```bash
cd unitree_rl_lab
./unitree_rl_lab.sh -l
```

## 训练

以 air 机器人（27dof，对应任务 `Unitree-LJ-Velocity`）为例：

```bash
cd unitree_rl_lab

# 推荐入口（headless，自动激活环境）
./unitree_rl_lab.sh -t --task Unitree-LJ-Velocity

# 等价命令
python scripts/rsl_rl/train.py --headless --task Unitree-LJ-Velocity
```

常用参数：

| 参数 | 说明 |
|------|------|
| `--task <name>` | 任务 ID |
| `--headless` | 不开图形界面，速度更快 |
| `--num_envs <N>` | 并行环境数（默认 4096） |
| `--max_iterations <N>` | 最大训练迭代数 |
| `--seed <N>` | 随机种子 |
| `--resume` | 从最近 checkpoint 恢复 |

训练产物：

```
unitree_rl_lab/logs/rsl_rl/<task_name>/<timestamp>/
├── model_*.pt           # PyTorch checkpoint
├── exported/policy.onnx # 导出的 ONNX，部署时使用
└── params/deploy.yaml   # 部署参数（关节顺序、PD、scale 等）
```

> 训练日志和 checkpoint **不会**进入 git（已被 `.gitignore` 排除）。

## 推理（Isaac Sim 可视化）

以 air 机器人为例：

```bash
cd unitree_rl_lab

./unitree_rl_lab.sh -p --task Unitree-LJ-Velocity

# 等价命令
python scripts/rsl_rl/play.py --task Unitree-LJ-Velocity
```

默认加载该 task 最近一次训练的 checkpoint，并在 Isaac Sim 中渲染。同时会导出 `policy.onnx` 供 sim2sim 使用。

加载指定 checkpoint：

```bash
python scripts/rsl_rl/play.py --task <name> --load_run <timestamp> --checkpoint <iter>
```

## Sim2Sim（MuJoCo）

### 1. 准备配置

`deploy_mujoco/configs/` 下已有的配置：

- `arm_new4.yaml` — **air 机器人（27dof，配套 `Unitree-LJ-Velocity`）**
- `g1_29dof.yaml` — G1 29DOF
- `go2.yaml` — Go2
- `h1.yaml` — H1

每个 YAML 主要字段（以 air 的 `arm_new4.yaml` 为例）：

```yaml
policy_path: ".../exported/policy.onnx"           # 训练导出的 ONNX
xml_path: ".../mujoco/arm_new4.xml"               # MuJoCo 场景
robot_params_path: ".../params/deploy.yaml"       # 训练同步的部署参数

simulation_duration: 60.0
simulation_dt: 0.002
control_decimation: 10                            # 控制降频（policy 频率 = 1/(dt*decimation)）
num_actions: 27                                   # air 机器人 27 个关节
num_obs: 450                                      # 观测维度
cmd_init: [1.0, 0.0, 0.0]                         # [vx, vy, ωz]
gait_period: null                                 # 带相位策略才需要
use_sensor: true
```

### 2. 运行

以 air 机器人为例：

```bash
conda activate mujoco
cd deploy_mujoco

python deploy_mujoco.py arm_new4.yaml          # air 机器人配置
python deploy_mujoco.py /abs/path/to/conf.yaml # 或绝对路径
```

### 3. 添加新机器人

1. **训练完成**后会自动生成 `logs/.../params/deploy.yaml`。
2. 准备一份 MuJoCo XML（`unitree_ros/robots/<robot>/...`），关节 `damping=0`、`armature` 与训练匹配。
3. 复制 `configs/g1_29dof.yaml` 改名后修改三条路径与维度。
4. `python deploy_mujoco.py <robot>.yaml`。

### 关键对齐点（避免 sim2real gap）

- 关节阻尼：MuJoCo XML 里 `damping=0`，PD 控制由部署脚本完成（用 `deploy.yaml` 的 stiffness/damping）。
- `joint_ids_map`：策略的关节顺序与 MuJoCo actuator 顺序若不一致，必须配置；一致则填 `null`。
- `armature`：必须与训练参数一致，否则动力学不一致。
- 观测拼接：`[ang_vel(3), gravity(3), cmd(3), joint_pos(n), joint_vel(n), action(n)] × history_len`，可选 `[sin_phase, cos_phase]`。

更多细节见 [`deploy_mujoco/README.md`](deploy_mujoco/README.md)。


## 常用工作流（以 air 机器人为例）

```bash
# 1. 训练
cd unitree_rl_lab
./unitree_rl_lab.sh -t --task Unitree-LJ-Velocity

# 2. Isaac Sim 中查看效果，导出 ONNX
./unitree_rl_lab.sh -p --task Unitree-LJ-Velocity

# 3. MuJoCo sim2sim 验证
conda activate mujoco
cd ../deploy_mujoco
python deploy_mujoco.py arm_new4.yaml
```

## 致谢

- [IsaacLab](https://github.com/isaac-sim/IsaacLab)
- [RSL-RL](https://github.com/leggedrobotics/rsl_rl)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab)
