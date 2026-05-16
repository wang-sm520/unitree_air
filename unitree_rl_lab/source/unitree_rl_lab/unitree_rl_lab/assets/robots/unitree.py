# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Unitree robots.

Reference: https://github.com/unitreerobotics/unitree_ros
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils import configclass

from unitree_rl_lab.assets.robots import unitree_actuators

UNITREE_MODEL_DIR = "/home/hpf/wsm/unitree_rl/unitree_model"
UNITREE_ROS_DIR = "/home/hpf/wsm/unitree_rl/unitree_ros"
ROBOT_ASSETS_DIR = os.path.dirname(os.path.abspath(__file__))


@configclass
class UnitreeArticulationCfg(ArticulationCfg):
    """Configuration for Unitree articulations."""

    joint_sdk_names: list[str] = None

    soft_joint_pos_limit_factor = 0.9


@configclass
class UnitreeUsdFileCfg(sim_utils.UsdFileCfg):
    activate_contact_sensors: bool = True
    rigid_props = sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=False,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0,
    )
    articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
    )


@configclass
class UnitreeUrdfFileCfg(sim_utils.UrdfFileCfg):
    fix_base: bool = False
    activate_contact_sensors: bool = True
    replace_cylinders_with_capsules = True
    joint_drive = sim_utils.UrdfConverterCfg.JointDriveCfg(
        gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
    )
    articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False,
        solver_position_iteration_count=8,
        solver_velocity_iteration_count=4,
    )
    rigid_props = sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=False,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0,
    )

    def replace_asset(self, meshes_dir, urdf_path):
        """Replace the asset with a temporary copy to avoid modifying the original asset.

        When need to change the collisions, place the modified URDF file separately in this repository,
        and let `meshes_dir` be provided by `unitree_ros`.
        This function will auto construct a complete `robot_description` file structure in the `/tmp` directory.
        Note: The mesh references inside the URDF should be in the same directory level as the URDF itself.
        """
        tmp_meshes_dir = "/tmp/IsaacLab/unitree_rl_lab/meshes"
        if os.path.exists(tmp_meshes_dir):
            os.remove(tmp_meshes_dir)
        os.makedirs("/tmp/IsaacLab/unitree_rl_lab", exist_ok=True)
        os.symlink(meshes_dir, tmp_meshes_dir)

        self.asset_path = "/tmp/IsaacLab/unitree_rl_lab/robot.urdf"
        if os.path.exists(self.asset_path):
            os.remove(self.asset_path)
        os.symlink(urdf_path, self.asset_path)






ARM_NEW_4_CFG = UnitreeArticulationCfg(
   spawn=UnitreeUrdfFileCfg(
        asset_path=f"{UNITREE_ROS_DIR}/robots/robot/urdf/robot.urdf",
    ),
    # spawn=UnitreeUsdFileCfg(
    #     usd_path=f"{UNITREE_ROS_DIR}/robots/arm_new_description/urdf/arm_new/arm_new.usd",
    # ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.85),  # 提高初始高度
        joint_pos={
            # 腿部 - 双腿微屈
            ".*_hip_pitch_joint": -0.2,
            ".*_hip_roll_joint": 0.0,
            ".*_hip_yaw_joint": 0.0,
            ".*_knee_joint": 0.4, 
            ".*_ankle_pitch_joint": -0.2,  
            ".*_ankle_roll_joint": 0.0,
            # 手臂 - 自然下垂
            ".*_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.1,
            "right_shoulder_roll_joint": -0.1,
            ".*_shoulder_yaw_joint": 0.0,
            ".*_elbow_joint": -0.1,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        # 腰部偏航 (waist_yaw_joint: Kp=200 kd=5.0)
        # waist_roll_joint 和 torso_joint 已在 URDF 中改为 fixed，不再需要 actuator
        "waist_yaw": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint"],
            effort_limit_sim=88,
            velocity_limit_sim=32.0,
            stiffness=200,
            damping=5,
            armature=0.01,
        ),
        # 髋关节俯仰 (hip_pitch: Kp=100 Kd=2.0)
        # 注意: USD中左腿是 "a__left_hip_pitch_joint"，右腿是 "right_hip_pitch_joint"
        "hip_pitch": ImplicitActuatorCfg(
            joint_names_expr=["right_hip_pitch_joint", "a__left_hip_pitch_joint"],
            effort_limit_sim=88,
            velocity_limit_sim=32.0,
            stiffness=100,
            damping=2.0,
            armature=0.01,
        ),
        # 髋关节侧摆 (hip_roll: Kp=100, Kd=2.0)
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint"],
            effort_limit_sim=139,
            velocity_limit_sim=20.0,
            stiffness=100,
            damping=2.0,
            armature=0.01,
        ),
        # 髋关节偏航 (hip_yaw: Kp=100, Kd=2.0)
        "hip_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint"],
            effort_limit_sim=88,
            velocity_limit_sim=32.0,
            stiffness=100,
            damping=2.0,
            armature=0.01,
        ),
        # 膝关节 (knee: Kp=150, Kd=0.8)
        "knee": ImplicitActuatorCfg(
            joint_names_expr=[".*_knee_joint"],
            effort_limit_sim=139,
            velocity_limit_sim=20.0,
            stiffness=150,
            damping=4,
            armature=0.01,
        ),
        # 踝关节 (ankle_pitch/roll: Kp=6.5, Kd=0.8)
        "ankle": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim=50,
            velocity_limit_sim=37.0,
            stiffness=40,
            damping=2,
            armature=0.01,
        ),
        # 肩关节俯仰 (shoulder_pitch: Kp=16.0, Kd=2.0)
        "shoulder_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch_joint"],
            effort_limit_sim=40,
            velocity_limit_sim=20.0,
            stiffness=40,
            damping=1,
            armature=0.01,
        ),
        # 肩关节侧摆 (shoulder_roll: Kp=20.0, Kd=2.0)
        "shoulder_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_roll_joint"],
            effort_limit_sim=40,
            velocity_limit_sim=20.0,
            stiffness=40,
            damping=1,
            armature=0.01,
        ),
        # 肩关节偏航 (shoulder_yaw: Kp=5.0, Kd=0.5)
        "shoulder_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_yaw_joint"],
            effort_limit_sim=20,
            velocity_limit_sim=30.0,
            stiffness=40,
            damping=1,
            armature=0.01,
        ),
        # 肘关节 (elbow: Kp=8.0, Kd=0.5)
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=[".*_elbow_joint"],
            effort_limit_sim=20,
            velocity_limit_sim=30.0,
            stiffness=40,
            damping=1,
            armature=0.01,
        ),
        # 手腕 (wrist: Kp=5.0~6.0, Kd=0.5)
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_wrist_roll_joint",
                ".*_wrist_pitch_joint",
                ".*_wrist_yaw_joint",
            ],
            effort_limit_sim=10,
            velocity_limit_sim=22.0,
            stiffness={
                ".*_wrist_roll_joint": 40,
                ".*_wrist_pitch_joint": 40,
                ".*_wrist_yaw_joint": 40,
            },
            damping=1,
            armature=0.01,
        ),
    },
    # 关节名称列表 (用于部署到真机，共27个关节)
    # waist_roll_joint 和 torso_joint 已改为 fixed，不在列表中
    joint_sdk_names=[
        # 腰部 (1)
        "waist_yaw_joint",
        # 左腿 (6) - 注意USD中的名称带有 "a__" 前缀
        "a__left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        # 右腿 (6)
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        # 左臂 (7)
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        # 右臂 (7)
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ],
)


ARMATURE_RS00 = 0.001
ARMATURE_RS03 = 0.02
ARMATURE_RS04 = 0.04
ARMATURE_RS05 = 0.0007
ARMATURE_RS06 = 0.012

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.3

STIFFNESS_RS00 = ARMATURE_RS00 * NATURAL_FREQ**2
STIFFNESS_RS03 = ARMATURE_RS03 * NATURAL_FREQ**2
STIFFNESS_RS04 = ARMATURE_RS04 * NATURAL_FREQ**2
STIFFNESS_RS05 = ARMATURE_RS05 * NATURAL_FREQ**2
STIFFNESS_RS06 = ARMATURE_RS06 * NATURAL_FREQ**2

DAMPING_RS00 = 2.0 * DAMPING_RATIO * ARMATURE_RS00 * NATURAL_FREQ
DAMPING_RS03 = 2.0 * DAMPING_RATIO * ARMATURE_RS03 * NATURAL_FREQ
DAMPING_RS04 = 2.0 * DAMPING_RATIO * ARMATURE_RS04 * NATURAL_FREQ
DAMPING_RS05 = 2.0 * DAMPING_RATIO * ARMATURE_RS05 * NATURAL_FREQ
DAMPING_RS06 = 2.0 * DAMPING_RATIO * ARMATURE_RS06 * NATURAL_FREQ


# =============================================================================
# Custom Robot: ArmNew4 (29 DOF)
# =============================================================================

AIR_LJ_CFG = UnitreeArticulationCfg(
    # spawn=UnitreeUsdFileCfg(
    #     usd_path=f"/home/amiao/wsm/unitree_rl/unitree_ros/robots/air_lj/usd/air_lj.usd",
    # ),
    # spawn=UnitreeUrdfFileCfg(
    #     asset_path=f"{UNITREE_ROS_DIR}/robots/air_4.7/urdf/air_4.7.urdf",
    # ),
    
    spawn=UnitreeUrdfFileCfg(
        asset_path=f"{UNITREE_ROS_DIR}/robots/robot_001_5.13/urdf/urdf_v2_5.13.urdf",
    ),
    # spawn=UnitreeUrdfFileCfg(
    #     asset_path=f"{UNITREE_ROS_DIR}/robots/robot/urdf/robot_4.11.urdf",
    # ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.9),
        joint_pos={
            ".*hip_pitch_joint": -0.1,
            ".*_knee_joint": 0.3,
            ".*_ankle_pitch_joint": -0.2,
            ".*_shoulder_pitch_joint": 0.3,
            "left_shoulder_roll_joint": 0.25,
            "right_shoulder_roll_joint": -0.25,
            ".*_elbow_joint": 0.97,
            "left_wrist_roll_joint": 0.15,
            "right_wrist_roll_joint": -0.15,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        # 腰部偏航 waist_yaw_joint -> RS03
        # waist_roll_joint 和 torso_joint 已在 URDF 中改为 fixed，不再需要 actuator
        "waist_yaw": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint"],
            effort_limit_sim=60,
            velocity_limit_sim=20,
            stiffness=STIFFNESS_RS03,
            damping=DAMPING_RS03,
            armature=ARMATURE_RS03,
        ),
        # 髋关节俯仰 hip_pitch -> RS04
        "hip_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*hip_pitch_joint"],
            effort_limit_sim=120,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_RS04,
            damping=DAMPING_RS04,
            armature=ARMATURE_RS04,
        ),
        # 髋关节侧摆 hip_roll -> RS04
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint"],
            effort_limit_sim=120,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_RS04,
            damping=DAMPING_RS04,
            armature=ARMATURE_RS04,
        ),
        # 髋关节偏航 hip_yaw -> RS03
        "hip_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint"],
            effort_limit_sim=60,
            velocity_limit_sim=20,
            stiffness=STIFFNESS_RS03,
            damping=DAMPING_RS03,
            armature=ARMATURE_RS03,
        ),
        # 膝关节 knee -> RS04
        "knee": ImplicitActuatorCfg(
            joint_names_expr=[".*_knee_joint"],
            effort_limit_sim=120,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_RS04,
            damping=DAMPING_RS04,
            armature=ARMATURE_RS04,
        ),
        # 踝关节 ankle_pitch / ankle_roll -> RS06
        "ankle": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim=36,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_RS06*2.0,
            damping=DAMPING_RS06*2.0,
            armature=ARMATURE_RS06*2.0,
        ),
        # 肩关节俯仰 shoulder_pitch -> RS03
        "shoulder_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch_joint"],
            effort_limit_sim=60,
            velocity_limit_sim=20.0,
            stiffness=STIFFNESS_RS03,
            damping=DAMPING_RS03,
            armature=ARMATURE_RS03,
        ),
        # 肩关节侧摆 shoulder_roll -> RS03
        "shoulder_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_roll_joint"],
            effort_limit_sim=60,
            velocity_limit_sim=20.0,
            stiffness=STIFFNESS_RS03,
            damping=DAMPING_RS03,
            armature=ARMATURE_RS03,
        ),
        # 肩关节偏航 shoulder_yaw -> RS06
        "shoulder_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_yaw_joint"],
            effort_limit_sim=36,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_RS06,
            damping=DAMPING_RS06,
            armature=ARMATURE_RS06,
        ),
        # 肘关节 elbow -> RS06
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=[".*_elbow_joint"],
            effort_limit_sim=36,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_RS06,
            damping=DAMPING_RS06,
            armature=ARMATURE_RS06,
        ),
        # 手腕 wrist_roll / wrist_pitch / wrist_yaw -> RS00
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_wrist_roll_joint",
                ".*_wrist_pitch_joint",
                ".*_wrist_yaw_joint",
            ],
            effort_limit_sim=14,
            velocity_limit_sim=33.0,
            stiffness=STIFFNESS_RS00,
            damping=DAMPING_RS00,
            armature=ARMATURE_RS00,
        ),
    },
    # 关节名称列表 (用于部署到真机，共27个关节)
    # waist_roll_joint 和 torso_joint 已改为 fixed，不在列表中
    joint_sdk_names=[
        # 腰部 (1)
        "waist_yaw_joint",#0
        # 右臂 (7)
        "right_shoulder_pitch_joint",#1
        "right_shoulder_roll_joint",#2
        "right_shoulder_yaw_joint",#3
        "right_elbow_joint",#4
        "right_wrist_roll_joint",#5
        "right_wrist_yaw_joint",#6
        "right_wrist_pitch_joint",#7
        # 左臂 (7)
        "left_shoulder_pitch_joint",#8
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_yaw_joint",
        "left_wrist_pitch_joint",#14
        # 右腿 (6)
        "right_hip_pitch_joint",#15
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",#19
        "right_ankle_roll_joint",#20
        # 左腿 (6)
        "left_hip_pitch_joint",#21
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",#25
        "left_ankle_roll_joint",#26
    ],
)


# =============================================================================
# AIR_WSM 25 DOF variant: 在 AIR_LJ 基础上把两个 ankle_roll 关节锁死
# URDF: robot_4.11_25dof.urdf 把 *_ankle_roll_joint 改为 fixed，link 保留
# =============================================================================

AIR_WSM_25DOF_CFG = UnitreeArticulationCfg(
    spawn=UnitreeUrdfFileCfg(
        asset_path=f"{UNITREE_ROS_DIR}/robots/robot/urdf/robot_4.11_25dof.urdf",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.9),
        joint_pos={
            ".*hip_pitch_joint": -0.1,
            ".*_knee_joint": 0.3,
            ".*_ankle_pitch_joint": -0.2,
            ".*_shoulder_pitch_joint": 0.3,
            "left_shoulder_roll_joint": 0.25,
            "right_shoulder_roll_joint": -0.25,
            ".*_elbow_joint": 0.97,
            "left_wrist_roll_joint": 0.15,
            "right_wrist_roll_joint": -0.15,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "waist_yaw": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint"],
            effort_limit_sim=60,
            velocity_limit_sim=20,
            stiffness=STIFFNESS_RS03,
            damping=DAMPING_RS03,
            armature=ARMATURE_RS03,
        ),
        "hip_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*hip_pitch_joint"],
            effort_limit_sim=120,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_RS04,
            damping=DAMPING_RS04,
            armature=ARMATURE_RS04,
        ),
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint"],
            effort_limit_sim=120,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_RS04,
            damping=DAMPING_RS04,
            armature=ARMATURE_RS04,
        ),
        "hip_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint"],
            effort_limit_sim=60,
            velocity_limit_sim=20,
            stiffness=STIFFNESS_RS03,
            damping=DAMPING_RS03,
            armature=ARMATURE_RS03,
        ),
        "knee": ImplicitActuatorCfg(
            joint_names_expr=[".*_knee_joint"],
            effort_limit_sim=120,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_RS04,
            damping=DAMPING_RS04,
            armature=ARMATURE_RS04,
        ),
        # 踝关节只剩 ankle_pitch；ankle_roll 已在 URDF 改为 fixed
        "ankle": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint"],
            effort_limit_sim=36,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_RS06*2.0,
            damping=DAMPING_RS06*2.0,
            armature=ARMATURE_RS06*2.0,
        ),
        "shoulder_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch_joint"],
            effort_limit_sim=60,
            velocity_limit_sim=20.0,
            stiffness=STIFFNESS_RS03,
            damping=DAMPING_RS03,
            armature=ARMATURE_RS03,
        ),
        "shoulder_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_roll_joint"],
            effort_limit_sim=60,
            velocity_limit_sim=20.0,
            stiffness=STIFFNESS_RS03,
            damping=DAMPING_RS03,
            armature=ARMATURE_RS03,
        ),
        "shoulder_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_yaw_joint"],
            effort_limit_sim=36,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_RS06,
            damping=DAMPING_RS06,
            armature=ARMATURE_RS06,
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=[".*_elbow_joint"],
            effort_limit_sim=36,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_RS06,
            damping=DAMPING_RS06,
            armature=ARMATURE_RS06,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_wrist_roll_joint",
                ".*_wrist_pitch_joint",
                ".*_wrist_yaw_joint",
            ],
            effort_limit_sim=14,
            velocity_limit_sim=33.0,
            stiffness=STIFFNESS_RS00,
            damping=DAMPING_RS00,
            armature=ARMATURE_RS00,
        ),
    },
    # 关节名称列表 (用于部署到真机，共25个关节)
    # waist_roll_joint / torso_joint / *_ankle_roll_joint 已在 URDF 中改为 fixed，不在列表中
    joint_sdk_names=[
        # 腰部 (1)
        "waist_yaw_joint",
        # 右臂 (7)
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_yaw_joint",
        "right_wrist_pitch_joint",
        # 左臂 (7)
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_yaw_joint",
        "left_wrist_pitch_joint",
        # 右腿 (5)
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        # 左腿 (5)
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
    ],
)


# =============================================================================
# ELF3 29 DOF (BXI robotics) — for AMP deployment compatibility.
# kp/kd, default joint pos, effort/velocity limits mirror bx_lab_amp/.../elf3.py
# (cross-checked against amp.py:41-66 from bxi_rl_controller_ros2_example).
# =============================================================================

ELF3_29DOF_CFG = UnitreeArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ROBOT_ASSETS_DIR}/elf3/bx_29dof.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.05),
        joint_pos={
            "waist_y_joint": 0.0,
            "waist_x_joint": 0.0,
            "waist_z_joint": 0.0,

            "l_hip_y_joint": -0.3,
            "l_hip_x_joint": 0.0,
            "l_hip_z_joint": 0.0,
            "l_knee_y_joint": 0.6,
            "l_ankle_y_joint": -0.3,
            "l_ankle_x_joint": 0.0,

            "r_hip_y_joint": -0.3,
            "r_hip_x_joint": 0.0,
            "r_hip_z_joint": 0.0,
            "r_knee_y_joint": 0.6,
            "r_ankle_y_joint": -0.3,
            "r_ankle_x_joint": 0.0,

            "l_shoulder_y_joint": 0.2,
            "l_shoulder_x_joint": 0.2,
            "l_shoulder_z_joint": 0.0,
            "l_elbow_y_joint": 0.6,
            "l_wrist_x_joint": 0.0,
            "l_wrist_y_joint": 0.0,
            "l_wrist_z_joint": 0.0,

            "r_shoulder_y_joint": 0.2,
            "r_shoulder_x_joint": -0.2,
            "r_shoulder_z_joint": 0.0,
            "r_elbow_y_joint": 0.6,
            "r_wrist_x_joint": 0.0,
            "r_wrist_y_joint": 0.0,
            "r_wrist_z_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "waist": ImplicitActuatorCfg(
            joint_names_expr=[
                "waist_y_joint",
                "waist_x_joint",
                "waist_z_joint",
            ],
            effort_limit_sim={
                "waist_y_joint": 100,
                "waist_x_joint": 100,
                "waist_z_joint": 100,
            },
            velocity_limit_sim={
                "waist_y_joint": 20,
                "waist_x_joint": 20,
                "waist_z_joint": 20,
            },
            stiffness={
                "waist_y_joint": 108.448,
                "waist_x_joint": 162.672,
                "waist_z_joint": 176.421,
            },
            damping={
                "waist_y_joint": 6.904,
                "waist_x_joint": 10.356,
                "waist_z_joint": 11.231,
            },
        ),
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_y_joint",
                ".*_hip_x_joint",
                ".*_hip_z_joint",
                ".*_knee_y_joint",
            ],
            effort_limit_sim={
                ".*_hip_y_joint": 100,
                ".*_hip_x_joint": 100,
                ".*_hip_z_joint": 50,
                ".*_knee_y_joint": 150,
            },
            velocity_limit_sim={
                ".*_hip_y_joint": 20,
                ".*_hip_x_joint": 20,
                ".*_hip_z_joint": 20,
                ".*_knee_y_joint": 20,
            },
            stiffness={
                ".*_hip_y_joint": 176.421,
                ".*_hip_x_joint": 176.421,
                ".*_hip_z_joint": 54.224,
                ".*_knee_y_joint": 176.421,
            },
            damping={
                ".*_hip_y_joint": 11.231,
                ".*_hip_x_joint": 11.231,
                ".*_hip_z_joint": 3.452,
                ".*_knee_y_joint": 11.231,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_ankle_y_joint",
                ".*_ankle_x_joint",
            ],
            effort_limit_sim={
                ".*_ankle_y_joint": 50,
                ".*_ankle_x_joint": 20,
            },
            velocity_limit_sim={
                ".*_ankle_y_joint": 20,
                ".*_ankle_x_joint": 20,
            },
            stiffness={
                ".*_ankle_y_joint": 33.493,
                ".*_ankle_x_joint": 21.771,
            },
            damping={
                ".*_ankle_y_joint": 2.132,
                ".*_ankle_x_joint": 1.386,
            },
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_y_joint",
                ".*_shoulder_x_joint",
                ".*_shoulder_z_joint",
                ".*_elbow_y_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_y_joint": 50,
                ".*_shoulder_x_joint": 50,
                ".*_shoulder_z_joint": 25,
                ".*_elbow_y_joint": 50,
            },
            velocity_limit_sim={
                ".*_shoulder_y_joint": 20,
                ".*_shoulder_x_joint": 20,
                ".*_shoulder_z_joint": 20,
                ".*_elbow_y_joint": 20,
            },
            stiffness={
                ".*_shoulder_y_joint": 54.224,
                ".*_shoulder_x_joint": 54.224,
                ".*_shoulder_z_joint": 16.747,
                ".*_elbow_y_joint": 54.224,
            },
            damping={
                ".*_shoulder_y_joint": 3.452,
                ".*_shoulder_x_joint": 3.452,
                ".*_shoulder_z_joint": 1.066,
                ".*_elbow_y_joint": 3.452,
            },
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_wrist_x_joint",
                ".*_wrist_y_joint",
                ".*_wrist_z_joint",
            ],
            effort_limit_sim={
                ".*_wrist_x_joint": 25,
                ".*_wrist_y_joint": 25,
                ".*_wrist_z_joint": 25,
            },
            velocity_limit_sim={
                ".*_wrist_x_joint": 20,
                ".*_wrist_y_joint": 20,
                ".*_wrist_z_joint": 20,
            },
            stiffness={
                ".*_wrist_x_joint": 16.747,
                ".*_wrist_y_joint": 16.747,
                ".*_wrist_z_joint": 16.747,
            },
            damping={
                ".*_wrist_x_joint": 1.066,
                ".*_wrist_y_joint": 1.066,
                ".*_wrist_z_joint": 1.066,
            },
        ),
    },
)
