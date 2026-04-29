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


# ARMATURE_5020 = 0.003609725
# ARMATURE_7520_14 = 0.010177520
# ARMATURE_7520_22 = 0.025101925
# ARMATURE_4010 = 0.00425
# NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
# DAMPING_RATIO = 2.0

# STIFFNESS_5020 = ARMATURE_5020 * NATURAL_FREQ**2  # 14.25062309787429
# STIFFNESS_7520_14 = ARMATURE_7520_14 * NATURAL_FREQ**2  # 40.17923847137318
# STIFFNESS_7520_22 = ARMATURE_7520_22 * NATURAL_FREQ**2  # 99.09842777666113
# STIFFNESS_4010 = ARMATURE_4010 * NATURAL_FREQ**2  # 16.77832748089279

# DAMPING_5020 = 2.0 * DAMPING_RATIO * ARMATURE_5020 * NATURAL_FREQ  # 0.907222843292423
# DAMPING_7520_14 = 2.0 * DAMPING_RATIO * ARMATURE_7520_14 * NATURAL_FREQ  # 2.5578897650279457
# DAMPING_7520_22 = 2.0 * DAMPING_RATIO * ARMATURE_7520_22 * NATURAL_FREQ  # 6.3088018534966395
# DAMPING_4010 = 2.0 * DAMPING_RATIO * ARMATURE_4010 * NATURAL_FREQ  # 1.06814150219

STIFFNESS_5020 = 50    #踝关节 肩关节
STIFFNESS_7520_14 =  200  #腰部  髋部
STIFFNESS_7520_22 = 400  #髋关节  膝关节
STIFFNESS_4010 = 50   #手腕

DAMPING_5020 = 2
DAMPING_7520_14 = 10 
DAMPING_7520_22 = 15 
DAMPING_4010 = 2
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
        asset_path=f"{UNITREE_ROS_DIR}/robots/robot/urdf/robot_4.11.urdf",
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
        # 腰部偏航 (waist_yaw_joint)
        # waist_roll_joint 和 torso_joint 已在 URDF 中改为 fixed，不再需要 actuator
        "waist_yaw": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint"],
            effort_limit_sim=48,
            velocity_limit_sim=20,
            stiffness=STIFFNESS_7520_14,
            damping=DAMPING_7520_14,
            armature=0.02,
        ),
        # 髋关节俯仰 (参照 G1 29dof)
        "hip_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*hip_pitch_joint"],
            effort_limit_sim=96,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_7520_14,
            damping=DAMPING_7520_14,
            armature=0.04,
        ),
        # 髋关节侧摆 (参照 G1 29dof)
        "hip_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint"],
            effort_limit_sim=96,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_7520_22,
            damping=DAMPING_7520_22,
            armature=0.04,
        ),
        # 髋关节偏航 (参照 G1 29dof)
        "hip_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint"],
            effort_limit_sim=48,
            velocity_limit_sim=20,
            stiffness=STIFFNESS_7520_14,
            damping=DAMPING_7520_14,
            armature=0.02,
        ),
        # 膝关节 (参照 G1 29dof)
        "knee": ImplicitActuatorCfg(
            joint_names_expr=[".*_knee_joint"],
            effort_limit_sim=96,
            velocity_limit_sim=15,
            stiffness=STIFFNESS_7520_22,
            damping=DAMPING_7520_22,
            armature=0.04,
        ),
        # 踝关节 (参照 G1 29dof)
        "ankle": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim=29,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_5020,
            damping=DAMPING_5020,
            armature=0.012,
        ),
        # 肩关节俯仰 (参照 G1 29dof)
        "shoulder_pitch": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch_joint"],
            effort_limit_sim=48,
            velocity_limit_sim=20.0,
            stiffness=STIFFNESS_5020,
            damping=DAMPING_5020,
            armature=0.02,
        ),
        # 肩关节侧摆 (参照 G1 29dof)
        "shoulder_roll": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_roll_joint"],
            effort_limit_sim=48,
            velocity_limit_sim=20.0,
            stiffness=STIFFNESS_5020,
            damping=DAMPING_5020,
            armature=0.02,
        ),
        # 肩关节偏航 (参照 G1 29dof)
        "shoulder_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_yaw_joint"],
            effort_limit_sim=29,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_5020,
            damping=DAMPING_5020,
            armature=0.012,
        ),
        # 肘关节 (参照 G1 29dof)
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=[".*_elbow_joint"],
            effort_limit_sim=29,
            velocity_limit_sim=50.0,
            stiffness=STIFFNESS_5020,
            damping=DAMPING_5020,
            armature=0.012,
        ),
        # 手腕 (参照 G1 29dof)
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_wrist_roll_joint",
                ".*_wrist_pitch_joint",
                ".*_wrist_yaw_joint",
            ],
            effort_limit_sim=10,
            velocity_limit_sim=33.0,
            stiffness={
                ".*_wrist_roll_joint": STIFFNESS_5020,
                ".*_wrist_pitch_joint": STIFFNESS_4010,
                ".*_wrist_yaw_joint": STIFFNESS_4010,
            },
            damping={
                ".*_wrist_roll_joint": DAMPING_5020,
                ".*_wrist_pitch_joint": DAMPING_4010,
                ".*_wrist_yaw_joint": DAMPING_4010,
            },
            armature={
                ".*_wrist_roll_joint": 0.001,
                ".*_wrist_pitch_joint": 0.001,
                ".*_wrist_yaw_joint": 0.001,
            },
        ),
    },
    # 关节名称列表 (用于部署到真机，共27个关节)
    # waist_roll_joint 和 torso_joint 已改为 fixed，不在列表中
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
        # 右腿 (6)
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        # 左腿 (6)
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    ],
)
