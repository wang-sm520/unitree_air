# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for DT114 humanoid robot (New Version)."""

import os
import shutil

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils import configclass

# Path to the NEW DT114 URDF package
DT114_URDF_DIR = "/home/hpf/wsm/unitree_rl/unitree_ros/robots/xrv1"


@configclass
class DT114ArticulationCfg(ArticulationCfg):
    """Configuration for DT114 articulations."""

    joint_sdk_names: list[str] = None
    soft_joint_pos_limit_factor = 0.9


@configclass
class DT114UrdfFileCfg(sim_utils.UrdfFileCfg):
    """URDF configuration for DT114 with mesh path handling."""

    fix_base: bool = False
    activate_contact_sensors: bool = True
    replace_cylinders_with_capsules = True
    joint_drive = sim_utils.UrdfConverterCfg.JointDriveCfg(
        gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
    )
    articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False,
        solver_position_iteration_count=4,
        solver_velocity_iteration_count=0,
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

    def replace_asset(self):
        """Set up symlinks and modify URDF for mesh resolution."""
        # Create temporary directory structure
        tmp_dir = "/tmp/IsaacLab/unitree_rl_lab/dt114_new"
        meshes_symlink = f"{tmp_dir}/meshes"

        os.makedirs(tmp_dir, exist_ok=True)

        # Remove existing symlink if present
        if os.path.islink(meshes_symlink):
            os.remove(meshes_symlink)
        elif os.path.exists(meshes_symlink):
            shutil.rmtree(meshes_symlink)

        # Create symlink to meshes directory
        meshes_dir = f"{DT114_URDF_DIR}/meshes"
        if not os.path.exists(meshes_dir):
            meshes_dir = f"{DT114_URDF_DIR}/urdf/meshes"
        
        os.symlink(meshes_dir, meshes_symlink)

        # Copy URDF and update paths
        urdf_src = f"{DT114_URDF_DIR}/urdf/arm new 4_1.27.urdf"
        self.asset_path = f"{tmp_dir}/dt114_new.urdf"

        if os.path.exists(self.asset_path):
            os.remove(self.asset_path)

        # Read URDF and update package paths to relative paths
        with open(urdf_src, "r", encoding="utf-8") as f:
            urdf_content = f.read()

        # Replace package:// paths with relative paths
        urdf_content = urdf_content.replace("package://arm new 4/meshes/", "meshes/")
        
        # Replace robot name to avoid dot issues
        urdf_content = urdf_content.replace('name="arm new 4"', 'name="dt114_new"')

        import re

        def _set_joint_axis(urdf_text: str, joint_name: str, axis_xyz: str) -> str:
            pattern = re.compile(
                rf'(<joint\s+name="{re.escape(joint_name)}"[\s\S]*?<axis\s+xyz=")([^"]+)(")',
                re.MULTILINE,
            )

            def _replace(match: re.Match) -> str:
                return match.group(1) + axis_xyz + match.group(3)

            return pattern.sub(_replace, urdf_text, count=1)

        def _set_joint_limit(urdf_text: str, joint_name: str, lower: str, upper: str) -> str:
            # Matches <limit lower="..." upper="..." ... />
            # We want to replace lower and upper values
            # The order of attributes might vary, so let's be robust
            pattern = re.compile(
                rf'(<joint\s+name="{re.escape(joint_name)}"[\s\S]*?<limit\s+)([^>]*?)(/>)',
                re.MULTILINE,
            )
            
            def _replace(match: re.Match) -> str:
                attrs = match.group(2)
                # Replace lower="..."
                attrs = re.sub(r'lower="[^"]+"', f'lower="{lower}"', attrs)
                # Replace upper="..."
                attrs = re.sub(r'upper="[^"]+"', f'upper="{upper}"', attrs)
                return match.group(1) + attrs + match.group(3)

            return pattern.sub(_replace, urdf_text, count=1)

        def _fix_joint(urdf_text: str, joint_name: str) -> str:
            """Change joint type to fixed."""
            pattern = re.compile(
                rf'(<joint\s+name="{re.escape(joint_name)}"\s+type=")([^"]+)(")',
                re.MULTILINE,
            )
            return pattern.sub(r'\1fixed\3', urdf_text, count=1)

        def _remove_collision(urdf_text: str, link_name: str) -> str:
            """Remove collision block from a specific link."""
            pattern = re.compile(
                rf'(<link\s+name="{re.escape(link_name)}"[\s\S]*?)(<collision>[\s\S]*?</collision>)([\s\S]*?</link>)',
                re.MULTILINE,
            )
            return pattern.sub(r'\1\3', urdf_text, count=1)

        # Fix upper body joints (make them fixed)
        upper_body_joints = [
            "waist_yaw_joint", "waist_roll_joint", "torso_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint"
        ]
        for jnt in upper_body_joints:
            urdf_content = _fix_joint(urdf_content, jnt)

        # Remove collisions for upper body links
        upper_body_links = [
            "waist_yaw_link", "waist_roll_link", "torso_link",
            "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link",
            "left_elbow_link", "left_wrist_roll_link", "left_wrist_yaw_link", "left_wrist_pitch_link",
            "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link",
            "right_elbow_link", "right_wrist_roll_link", "right_wrist_yaw_link", "right_wrist_pitch_link"
        ]
        for lnk in upper_body_links:
            urdf_content = _remove_collision(urdf_content, lnk)

        # Fix joint axes (Y-axis for pitch)
        # 0_left_knee_joint needs to be flipped (0 1 0 -> 0 -1 0) because the mesh/joint definition is mirrored? 
        # Actually standard humanoid leg configuration usually requires Pitch to be consistent.
        # Let's check if left knee needs -1 axis to bend forward with positive command.
        # If the joint frame is rotated 180 deg around Z or X, Y axis might be inverted.
        # Based on common issues, let's try flipping the left knee axis to -1.
        # urdf_content = _set_joint_axis(urdf_content, "0_left_knee_joint", "0.0000 -1.0000 0.0000") 
        # urdf_content = _set_joint_axis(urdf_content, "0_right_ankle_pitch_joint", "0.0000 1.0000 0.0000")
        # urdf_content = _set_joint_axis(urdf_content, "0_left_ankle_pitch_joint", "0.0000 -1.0000 0.0000")

        # Set knee joint limits to prevent hyperextension
        # Both Knees: -0.5 to 1.0 per latest request
        urdf_content = _set_joint_limit(urdf_content, "left_knee_joint", "-0.5", "1.0")
        urdf_content = _set_joint_limit(urdf_content, "right_knee_joint", "-0.5", "1.0")

        # Standardize joint names (remove 0_ prefix if any)
        urdf_content = urdf_content.replace('name="0_left_hip_pitch_joint"', 'name="left_hip_pitch_joint"')

        with open(self.asset_path, "w", encoding="utf-8") as f:
            f.write(urdf_content)


# Create spawn config instance and set up paths
_dt114_spawn_cfg = DT114UrdfFileCfg(
    asset_path=f"{DT114_URDF_DIR}/urdf/arm.SLDASM.urdf",  # Will be replaced
)
_dt114_spawn_cfg.replace_asset()

DT114_NEW_JOINT_SDK_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

DT114_NEW_KP = [
    1000.0,  # left_hip_pitch_joint
    1000.0,  # left_hip_roll_joint
    1000.0,   # left_hip_yaw_joint
    1000.0,   # left_knee_joint
    100.0,   # left_ankle_pitch_joint
    100.0,   # left_ankle_roll_joint
    1000.0,  # right_hip_pitch_joint
    1000.0,  # right_hip_roll_joint
    1000.0,   # right_hip_yaw_joint
    1000.0,   # right_knee_joint
    100.0,   # right_ankle_pitch_joint
    100.0,   # right_ankle_roll_joint
]

DT114_NEW_KD = [
    30.0,  # left_hip_pitch_joint
    30.0,  # left_hip_roll_joint
    30.0,  # left_hip_yaw_joint
    30.0,  # left_knee_joint
    8.0,  # left_ankle_pitch_joint
    8.0,  # left_ankle_roll_joint
    30.0,  # right_hip_pitch_joint
    30.0,  # right_hip_roll_joint
    30.0,  # right_hip_yaw_joint
    30.0,  # right_knee_joint
    8.0,  # right_ankle_pitch_joint
    8.0,  # right_ankle_roll_joint
]

DT114_NEW_STIFFNESS = dict(zip(DT114_NEW_JOINT_SDK_NAMES, DT114_NEW_KP))
DT114_NEW_DAMPING = dict(zip(DT114_NEW_JOINT_SDK_NAMES, DT114_NEW_KD))

UNITREE_DT114_NEW_CFG = DT114ArticulationCfg(
    spawn=_dt114_spawn_cfg,
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.89),
        joint_pos={
            # Default pose
            # ".*": 0.0,
            # Adjust knees/hips for standing if needed
            ".*_hip_roll_joint": 0.0,
            ".*_hip_pitch_joint": -0.2,
            # Both knees use positive init state for final urdf
            "right_knee_joint": 0.35,
            # Left knee uses positive init state (since limits are [0, 1.4])
            "left_knee_joint": 0.35,
            ".*_ankle_pitch_joint": 0.0,
            ".*_ankle_roll_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        # Leg main actuators - hip pitch and hip yaw
        "legs_main": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_pitch_joint",
                ".*_hip_yaw_joint",
            ],
            effort_limit_sim=88,
            velocity_limit_sim=32.0,
            stiffness={
                "left_hip_pitch_joint": DT114_NEW_STIFFNESS["left_hip_pitch_joint"],
                "right_hip_pitch_joint": DT114_NEW_STIFFNESS["right_hip_pitch_joint"],
                "left_hip_yaw_joint": DT114_NEW_STIFFNESS["left_hip_yaw_joint"],
                "right_hip_yaw_joint": DT114_NEW_STIFFNESS["right_hip_yaw_joint"],
            },
            damping={
                "left_hip_pitch_joint": DT114_NEW_DAMPING["left_hip_pitch_joint"],
                "right_hip_pitch_joint": DT114_NEW_DAMPING["right_hip_pitch_joint"],
                "left_hip_yaw_joint": DT114_NEW_DAMPING["left_hip_yaw_joint"],
                "right_hip_yaw_joint": DT114_NEW_DAMPING["right_hip_yaw_joint"],
            },
            armature=0.01,
        ),
        # Leg roll and knee actuators
        "legs_roll_knee": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_roll_joint",
                ".*_knee_joint",
            ],
            effort_limit_sim=139,
            velocity_limit_sim=20.0,
            stiffness={
                "left_hip_roll_joint": DT114_NEW_STIFFNESS["left_hip_roll_joint"],
                "right_hip_roll_joint": DT114_NEW_STIFFNESS["right_hip_roll_joint"],
                "left_knee_joint": DT114_NEW_STIFFNESS["left_knee_joint"],
                "right_knee_joint": DT114_NEW_STIFFNESS["right_knee_joint"],
            },
            damping={
                "left_hip_roll_joint": DT114_NEW_DAMPING["left_hip_roll_joint"],
                "right_hip_roll_joint": DT114_NEW_DAMPING["right_hip_roll_joint"],
                "left_knee_joint": DT114_NEW_DAMPING["left_knee_joint"],
                "right_knee_joint": DT114_NEW_DAMPING["right_knee_joint"],
            },
            armature=0.01,
        ),
        # Ankle actuators
        "ankles": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_ankle_pitch_joint",
                ".*_ankle_roll_joint",
            ],
            effort_limit_sim=35,
            velocity_limit_sim=30,
            stiffness={
                "left_ankle_pitch_joint": DT114_NEW_STIFFNESS["left_ankle_pitch_joint"],
                "right_ankle_pitch_joint": DT114_NEW_STIFFNESS["right_ankle_pitch_joint"],
                "left_ankle_roll_joint": DT114_NEW_STIFFNESS["left_ankle_roll_joint"],
                "right_ankle_roll_joint": DT114_NEW_STIFFNESS["right_ankle_roll_joint"],
            },
            damping={
                "left_ankle_pitch_joint": DT114_NEW_DAMPING["left_ankle_pitch_joint"],
                "right_ankle_pitch_joint": DT114_NEW_DAMPING["right_ankle_pitch_joint"],
                "left_ankle_roll_joint": DT114_NEW_DAMPING["left_ankle_roll_joint"],
                "right_ankle_roll_joint": DT114_NEW_DAMPING["right_ankle_roll_joint"],
            },
            armature=0.01,
        ),
    },
    joint_sdk_names=DT114_NEW_JOINT_SDK_NAMES,
)
