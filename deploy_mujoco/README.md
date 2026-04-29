# MuJoCo Sim2Sim 部署

本目录包含将Isaac Lab训练的RL策略部署到MuJoCo仿真的sim2sim部署脚本。

## 演示

https://github.com/user-attachments/assets/6d7d3345-2a57-496b-8e60-ba81f7c499c0

## 环境准备

安装依赖：

```bash
# 使用conda（推荐）
conda activate mujoco

# 或手动安装
pip install mujoco onnxruntime pyyaml numpy
```

## 使用方法

### 基本用法

```bash
# 指定配置文件运行
python deploy/deploy_mujoco/deploy_mujoco.py g1_29dof.yaml

# 或使用绝对路径
python deploy/deploy_mujoco/deploy_mujoco.py /path/to/config.yaml
```

### 配置说明

每个机器人需要在 `configs/` 目录下有一个YAML配置文件，包含：

- **Policy path**: ONNX策略文件路径
- **MuJoCo XML path**: 机器人的MuJoCo XML文件路径
- **Robot deploy parameters path**: 训练时生成的 `deploy.yaml` 路径
- **Simulation parameters**: 仿真时长、时间步长、控制降频
- **Command configuration**: 初始速度指令
- **Policy dimensions**: 动作和观测维度

配置示例（`configs/g1_29dof.yaml`）：

```yaml
# 策略路径
policy_path: "{PROJECT_ROOT}/deploy/robots/g1_29dof/config/policy/velocity/v0/exported/policy.onnx"

# MuJoCo XML模型路径
xml_path: "{PROJECT_ROOT}/source/unitree_model/G1/29dof/mujoco/g1_29dof.xml"

# 机器人部署参数路径（训练时自动生成）
robot_params_path: "{PROJECT_ROOT}/deploy/robots/g1_29dof/config/policy/velocity/v0/params/deploy.yaml"

# 仿真参数
simulation_duration: 60.0
simulation_dt: 0.002
control_decimation: 10

# 策略维度
num_actions: 29
num_obs: 480

# 初始速度指令
cmd_init: [1.0, 0.0, 0.0]

# 步态周期（可选，用于带相位的策略）
gait_period: null

# 使用传感器获取的输入
use_sensor: true
```

## 部署参数 (`deploy.yaml`)

`deploy.yaml` 文件在**训练时自动生成**，包含所有机器人相关参数：

- `joint_ids_map`: 策略关节顺序到MuJoCo执行器顺序的映射
- `stiffness`: PD控制Kp增益
- `damping`: PD控制Kd增益
- `default_joint_pos`: 策略的默认关节位置
- `actions`: 动作缩放和偏移
- `observations`: 观测缩放和历史长度

**重要提示**：不要手动编辑 `deploy.yaml`，它必须与训练配置完全一致。

## MuJoCo XML 配置

为确保sim2sim成功，MuJoCo XML必须与训练物理参数匹配：

### 关节阻尼

移除关节定义中的 `damping`（设为0或省略）。PD控制由部署脚本处理：

```xml
<!-- 正确：阻尼由部署脚本处理 -->
<joint name="hip_pitch_joint" type="hinge" axis="0 1 0" range="-2 2" armature="0.01"/>

<!-- 错误：双重阻尼 -->
<joint name="hip_pitch_joint" type="hinge" axis="0 1 0" range="-2 2" damping="2.56" armature="0.01"/>
```

### 惯量 (Armature)

设置 `armature` 值以匹配训练配置：

| 关节类型 | Armature值 | 来源 |
|---------|-----------|------|
| 髋pitch/yaw | 0.010177520 | ARMATURE_7520_14 |
| 髋roll, 膝盖 | 0.025101925 | ARMATURE_7520_22 |
| 踝关节 | 0.00721945 | 2 × ARMATURE_5020 |
| 肩, 肘, 腕roll | 0.003609725 | ARMATURE_5020 |
| 腕pitch/yaw | 0.00425 | ARMATURE_4010 |

## 关节映射

`joint_ids_map` 处理以下差异：
- **策略顺序**: 由Isaac Lab在训练时确定（基于USD文件）
- **MuJoCo顺序**: 由XML中actuator序列确定

```
joint_ids_map[i] = j  表示：策略关节i → MuJoCo执行器j
```

当策略顺序与MuJoCo顺序一致时，设置 `joint_ids_map: null`。

## 观测结构

观测由历史缓冲区构建：

```
obs = [ang_vel(3), gravity(3), cmd(3), joint_pos(n), joint_vel(n), action(n)] * history_length
```

可选步态相位：
```
obs += [sin_phase, cos_phase]
```

## 添加新机器人

### 1. 训练策略

训练时会自动在日志目录生成 `deploy.yaml`：
```
logs/rsl_rl/{robot_name}/{timestamp}/params/deploy.yaml
```

### 2. 创建MuJoCo XML

创建XML文件时需注意：
- 关节名称与USD匹配
- 关节阻尼设为0
- 使用训练配置中的armature值
- 执行器顺序应与策略一致（或使用 `joint_ids_map`）

### 3. 创建配置文件

创建 `configs/{robot_name}.yaml`：

```yaml
policy_path: "{PROJECT_ROOT}/logs/rsl_rl/{robot_name}/{timestamp}/exported/policy.onnx"
xml_path: "{PROJECT_ROOT}/source/unitree_model/{robot_name}/mujoco/{robot_name}.xml"
robot_params_path: "{PROJECT_ROOT}/logs/rsl_rl/{robot_name}/{timestamp}/params/deploy.yaml"

simulation_duration: 60.0
simulation_dt: 0.002
control_decimation: 10

num_actions: 29
num_obs: 480

cmd_init: [0.0, 0.0, 0.0]
gait_period: null
```

### 4. 运行部署

```bash
python deploy/deploy_mujoco/deploy_mujoco.py {robot_name}.yaml
```

## 文件结构

```
deploy/deploy_mujoco/
├── deploy_mujoco.py      # 主部署脚本
├── README.md             # 本文件
└── configs/
    ├── g1_29dof.yaml     # G1 29DOF配置
    └── arm_new4.yaml     # ArmNew4配置
```
