# `velocity_env_cfg.py` 代码逐定义说明

对应源码：`unitree_rl_lab/tasks/locomotion/robots/arm_new/velocity_env_cfg.py`

## 1. 文件整体作用

该文件是一个 IsaacLab 的强化学习环境配置文件，用于“速度跟踪型 locomotion 任务”。  
核心职责是把环境拆成多个配置块并装配到 `RobotEnvCfg`：

- 场景（地形、机器人、传感器、灯光）
- 事件（启动随机化、重置策略、周期扰动）
- 指令（速度命令采样）
- 动作（关节位置动作定义）
- 观测（policy/critic 输入）
- 奖励（各项正负奖励）
- 终止条件（超时/跌倒/姿态异常）
- 课程学习（难度调度）

## 2. 导入项定义含义

以下是文件顶部每个导入“在本文件中的角色”。

| 导入 | 含义 | 在本文件中的用途 |
|---|---|---|
| `import math` | Python 数学库 | 计算奖励里的 `std=math.sqrt(0.25)` |
| `import isaaclab.sim as sim_utils` | 仿真相关配置构件 | 刚体材质、灯光材质、MDL 材质等 |
| `import isaaclab.terrains as terrain_gen` | 地形生成配置构件 | 创建 `TerrainGeneratorCfg` 与子地形配置 |
| `ArticulationCfg, AssetBaseCfg` | 资产配置类型 | 机器人 articulation、天空光资产 |
| `ManagerBasedRLEnvCfg` | RL 环境总配置基类 | `RobotEnvCfg` 继承它 |
| `CurriculumTermCfg as CurrTerm` | 课程项定义类型 | 定义课程学习项 |
| `EventTermCfg as EventTerm` | 事件项定义类型 | 定义 startup/reset/interval 事件 |
| `ObservationGroupCfg as ObsGroup` | 观测组类型 | `PolicyCfg`、`CriticCfg` 继承 |
| `ObservationTermCfg as ObsTerm` | 单个观测项类型 | 各观测字段都用它定义 |
| `RewardTermCfg as RewTerm` | 单个奖励项类型 | 各奖励字段都用它定义 |
| `SceneEntityCfg` | 场景实体选择器 | 用正则筛选 body/joint/sensor 对象 |
| `TerminationTermCfg as DoneTerm` | 终止项类型 | 定义 done 条件 |
| `InteractiveSceneCfg` | 场景配置基类 | `RobotSceneCfg` 继承 |
| `ContactSensorCfg, RayCasterCfg, patterns` | 传感器构件 | 接触传感器与高度扫描传感器 |
| `TerrainImporterCfg` | 地形导入器配置 | `terrain` 字段定义 |
| `configclass` | IsaacLab 配置类装饰器 | 将类声明为可管理配置对象 |
| `ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR` | 资源路径常量 | 材质/天空纹理路径 |
| `AdditiveUniformNoiseCfg as Unoise` | 均匀噪声配置 | 观测加噪 |
| `ARM_NEW_4_CFG as ROBOT_CFG` | 机器人默认配置 | 作为当前任务机器人模板 |
| `mdp` | 任务函数集合 | 奖励、观测、事件、动作、命令函数来源 |

## 3. 顶层常量定义

### 3.1 `COBBLESTONE_ROAD_CFG`（L24）

定义：`terrain_gen.TerrainGeneratorCfg(...)`。  
作用：地形生成器参数集合。

字段含义：

- `size=(8.0, 8.0)`：单块地形尺寸。
- `border_width=20.0`：地形外围边框宽度。
- `num_rows=9`、`num_cols=21`：地形网格块数量（课程/采样会用到）。
- `horizontal_scale=0.1`、`vertical_scale=0.005`：地形网格水平/垂直缩放。
- `slope_threshold=0.75`：坡度阈值（用于地形生成规则）。
- `difficulty_range=(0.0, 1.0)`：难度区间。
- `use_cache=False`：不使用缓存生成。
- `sub_terrains={"flat": MeshPlaneTerrainCfg(proportion=0.5)}`：仅启用平面子地形配置（当前实际效果偏“平地模式”）。

## 4. `RobotSceneCfg`（L41）

定义：`InteractiveSceneCfg` 子类。  
作用：定义训练场景中的地形、机器人、传感器和光照。

### 字段定义

- `terrain`（L45）
  - 类型：`TerrainImporterCfg`
  - 含义：导入/生成地面并设置其物理材质与视觉材质。
  - 关键点：
    - `terrain_type="plane"` 当前按平面地形使用。
    - `terrain_generator=COBBLESTONE_ROAD_CFG` 预留了生成器参数。
    - `max_init_terrain_level=...num_rows - 1` 允许初始化到最大地形等级。
    - `physics_material` 设置摩擦与恢复系数。
    - `visual_material` 指向 Nucleus 的 MDL 材质。

- `robot`（L65）
  - 类型：`ArticulationCfg`
  - 含义：机器人模型配置。
  - 关键点：从 `ROBOT_CFG` 复制并替换 prim 路径到 `"{ENV_REGEX_NS}/Robot"`，保证多环境并行时命名空间隔离。

- `height_scanner`（L68）
  - 类型：`RayCasterCfg`
  - 含义：高度扫描传感器。
  - 关键点：
    - 绑定 `torso_link`。
    - `offset` 在机体上方 `(0,0,20)` 发射射线。
    - `ray_alignment="yaw"` 跟随机体偏航对齐。
    - 网格模式 `resolution=0.1`，`size=[1.6, 1.0]`。
    - 只扫描 `/World/ground`。

- `contact_forces`（L76）
  - 类型：`ContactSensorCfg`
  - 含义：接触力传感器，覆盖机器人所有刚体（正则 `.*`）。
  - 关键点：`history_length=3`，`track_air_time=True`。

- `sky_light`（L78）
  - 类型：`AssetBaseCfg`
  - 含义：场景天空光。
  - 关键点：使用 HDR 天空纹理，强度 `750.0`。

## 5. `EventCfg`（L88）

定义：事件配置集合。  
作用：按 `startup/reset/interval` 三类时机注入随机化与扰动。

### 字段定义

- `physics_material`（L92）：启动时随机化机器人刚体摩擦/恢复系数。
- `add_base_mass`（L104）：启动时对 `torso_link` 增加随机质量（`operation="add"`）。
- `base_external_force_torque`（L115）：重置时施加外力扭矩；当前范围均为 0，等效关闭。
- `reset_base`（L125）：重置根状态，随机姿态范围为 x/y/yaw，速度全 0。
- `reset_robot_joints`（L141）：重置关节位置缩放区间 `(1.0,1.0)`，关节速度区间 `(-1.0,1.0)`。
- `push_robot`（L151）：每 5 秒施加一次速度型扰动，x/y 范围 `(-0.5,0.5)`。

## 6. `CommandsCfg`（L160）

定义：命令采样配置。  
作用：生成机器人要跟踪的速度指令。

### 字段定义

- `base_velocity`（L163）
  - 类型：`mdp.UniformLevelVelocityCommandCfg`
  - 含义：统一分布速度命令。
  - 关键参数：
    - `resampling_time_range=(10,10)`：每 10 秒重采样。
    - `rel_standing_envs=0.02`：少量环境是站立命令。
    - `rel_heading_envs=1.0`：启用航向相关环境比例（但 `heading_command=False`，不直接发 heading 命令）。
    - `ranges`：训练采样范围（较窄）。
    - `limit_ranges`：命令上限范围（较宽，play 模式会替换成它）。

## 7. `ActionsCfg`（L180）

定义：动作空间配置。  
作用：定义策略输出如何映射到机器人控制输入。

### 字段定义

- `JointPositionAction`（L183）
  - 类型：`mdp.JointPositionActionCfg`
  - 含义：全关节位置控制动作。
  - 关键参数：
    - `joint_names=[".*"]`：作用于所有关节。
    - `scale=0.25`：动作缩放。
    - `use_default_offset=True`：以默认关节位作为偏置。

## 8. `ObservationsCfg`（L189）

定义：观测配置容器，包含 `PolicyCfg` 与 `CriticCfg` 两组。  
作用：分别给 actor 和 critic 构造输入特征。

### 8.1 `PolicyCfg`（L193）

定义：`ObsGroup` 子类，策略网络观测组。

字段定义：

- `base_ang_vel`：机体角速度，带缩放和均匀噪声。
- `projected_gravity`：重力在机体坐标系的投影，带噪声。
- `velocity_commands`：当前命令（`command_name="base_velocity"`）。
- `joint_pos_rel`：相对关节角，带小噪声。
- `joint_vel_rel`：相对关节速度，带缩放和噪声。
- `last_action`：上一时刻动作。
- `gait_phase`：步态相位（周期 `0.8`）。

方法定义：

- `__post_init__`（L206）
  - `history_length=5`：堆叠 5 帧历史。
  - `enable_corruption=True`：启用观测扰动/腐化机制。
  - `concatenate_terms=True`：将各观测项拼接为单向量。

组实例定义：

- `policy: PolicyCfg = PolicyCfg()`（L212）：实际挂载 policy 观测组。

### 8.2 `CriticCfg`（L215）

定义：`ObsGroup` 子类，价值网络观测组（特权观测）。

字段定义：

- `base_lin_vel`：机体线速度。
- `base_ang_vel`：机体角速度（有缩放）。
- `projected_gravity`：重力投影。
- `velocity_commands`：当前速度命令。
- `joint_pos_rel`：相对关节角。
- `joint_vel_rel`：相对关节速度（有缩放）。
- `last_action`：上一动作。
- `gait_phase`：步态相位。
- 注释掉的 `height_scanner`：预留高度扫描观测项，当前不生效。

方法定义：

- `__post_init__`（L232）：`history_length=5`。

组实例定义：

- `critic: CriticCfg = CriticCfg()`（L236）：实际挂载 critic 观测组。

## 9. `RewardsCfg`（L240）

定义：奖励函数项集合。  
作用：将“任务目标 + 稳定性约束 + 姿态/能耗/接触约束”组合成总奖励。

### 9.1 任务项

- `track_lin_vel_xy`：跟踪 x/y 线速度命令（指数型奖励）。
- `track_ang_vel_z`：跟踪 z 角速度命令（指数型奖励）。
- `alive`：存活奖励。

### 9.2 基座与关节惩罚项

- `base_linear_velocity`：惩罚 z 向线速度（抑制上下窜动）。
- `base_angular_velocity`：惩罚 x/y 角速度（抑制翻摆）。
- `joint_vel`：关节速度 L2 惩罚。
- `joint_acc`：关节加速度 L2 惩罚。
- `action_rate`：动作变化率惩罚（平滑控制）。
- `dof_pos_limits`：关节逼近极限惩罚。
- `energy`：能耗惩罚。

### 9.3 姿态/关节偏离项

- `joint_deviation_arms`：约束肩/肘/腕关节偏离。
- `joint_deviation_waists`：约束腰部关节偏离。
- `joint_deviation_legs`：约束髋部 roll/yaw 偏离。
- `flat_orientation_l2`：机身水平姿态惩罚。
- `base_height`：机身高度偏离目标 `0.78` 惩罚。

### 9.4 足端与接触项

- `gait`：按周期与相位奖励交替步态。
- `feet_slide`：惩罚足端接触时滑动。
- `feet_clearance`：奖励合适抬脚高度。
- `undesired_contacts`：惩罚非期望部位接触（通过 body 正则过滤）。

## 10. `TerminationsCfg`（L343）

定义：回合终止条件。  
作用：满足任一条件则 done。

- `time_out`：回合超时。
- `base_height`：根高度低于 `0.2` 终止。
- `bad_orientation`：姿态角超阈值 `0.8` 终止。

## 11. `CurriculumCfg`（L352）

定义：课程学习项。  
作用：动态调节任务难度。

- 注释掉的 `terrain_levels`：当前不启用地形等级课程。
- `lin_vel_cmd_levels`：启用速度命令等级课程。

## 12. `RobotEnvCfg`（L360）

定义：主环境配置类，继承 `ManagerBasedRLEnvCfg`。  
作用：把 scene/obs/action/command/reward/done/event/curriculum 全部装配成完整训练环境。

### 字段定义

- `scene`（L364, L365）：
  - 出现两次定义。
  - 后者覆盖前者。
  - 最终生效值：`RobotSceneCfg(num_envs=1024, env_spacing=2.5)`。
- `observations`：`ObservationsCfg()`。
- `actions`：`ActionsCfg()`。
- `commands`：`CommandsCfg()`。
- `rewards`：`RewardsCfg()`。
- `terminations`：`TerminationsCfg()`。
- `events`：`EventCfg()`。
- `curriculum`：`CurriculumCfg()`。

### 方法定义：`__post_init__`（L376）

该方法在配置实例化后执行，用于二次设置。

- `self.decimation = 4`：控制频率分频。
- `self.episode_length_s = 20.0`：回合时长 20 秒。
- `self.sim.dt = 0.005`：物理步长。
- `self.sim.render_interval = self.decimation`：渲染间隔。
- `self.sim.physics_material = self.scene.terrain.physics_material`：仿真材质与地形材质对齐。
- `self.sim.physx.gpu_max_rigid_patch_count` 出现两次赋值：
  - 先设 `10 * 2**15`，后设 `5 * 2**15`。
  - 最终生效值是后者 `5 * 2**15`。
- 传感器更新周期：
  - `contact_forces.update_period = dt`
  - `height_scanner.update_period = decimation * dt`
- 地形课程开关逻辑：
  - 若 `curriculum.terrain_levels` 存在且地形生成器存在，则 `terrain_generator.curriculum=True`。
  - 否则设为 `False`。

## 13. `RobotPlayEnvCfg`（L404）

定义：播放/推理配置，继承 `RobotEnvCfg`。  
作用：在不改训练主配置的前提下，提供更轻量、便于测试/演示的环境。

### 方法定义：`__post_init__`（L405）

- 调用 `super().__post_init__()`，继承主环境初始化。
- 改小并行环境数：`self.scene.num_envs = 32`。
- 调整地形网格：`num_rows = 2`，`num_cols = 10`。
- 放宽命令范围：`self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges`。

## 14. 速查表（定义名 -> 作用一句话）

| 定义名 | 作用 |
|---|---|
| `COBBLESTONE_ROAD_CFG` | 地形生成参数集合 |
| `RobotSceneCfg` | 场景总配置（地形、机器人、传感器、灯光） |
| `EventCfg` | 事件注入配置（启动/重置/周期扰动） |
| `CommandsCfg` | 目标命令采样配置 |
| `ActionsCfg` | 动作空间定义 |
| `ObservationsCfg` | 观测总配置（policy + critic） |
| `ObservationsCfg.PolicyCfg` | policy 输入特征定义 |
| `ObservationsCfg.CriticCfg` | critic 输入特征定义 |
| `RewardsCfg` | 奖励项定义 |
| `TerminationsCfg` | 终止条件定义 |
| `CurriculumCfg` | 课程学习项定义 |
| `RobotEnvCfg` | 训练环境总装配配置 |
| `RobotPlayEnvCfg` | 推理/演示环境配置 |

## 15. 两个最容易忽略的“最终生效值”

- `scene` 定义重复：最终是 `num_envs=1024`（不是 4096）。
- `gpu_max_rigid_patch_count` 重复赋值：最终是 `5 * 2**15`。
