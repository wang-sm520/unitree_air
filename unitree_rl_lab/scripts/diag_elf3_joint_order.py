"""Diagnostic: print the IsaacLab-resolved joint order for the elf3 task.

Compares against the order hard-coded in bxi `amp.py` (HumanoidGaitPolicyLite) to
determine whether observation/action permutations are needed at deployment time.

Usage:
    python scripts/diag_elf3_joint_order.py
"""

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import gymnasium as gym

import isaaclab_tasks  # noqa: F401
import unitree_rl_lab.tasks  # noqa: F401
from unitree_rl_lab.utils.parser_cfg import parse_env_cfg


# Expected order from bxi amp.py:100-133 (the `isaac_to_mujoco_idx` joint-name comments)
AMP_ISAAC_ORDER = [
    "waist_y_joint", "waist_x_joint", "waist_z_joint",
    "l_hip_y_joint", "l_hip_x_joint", "l_hip_z_joint",
    "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
    "r_hip_y_joint", "r_hip_x_joint", "r_hip_z_joint",
    "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
    "l_shoulder_y_joint", "l_shoulder_x_joint", "l_shoulder_z_joint",
    "l_elbow_y_joint", "l_wrist_x_joint", "l_wrist_y_joint", "l_wrist_z_joint",
    "r_shoulder_y_joint", "r_shoulder_x_joint", "r_shoulder_z_joint",
    "r_elbow_y_joint", "r_wrist_x_joint", "r_wrist_y_joint", "r_wrist_z_joint",
]

TASK_ID = "Unitree-Elf3-29dof-Velocity"


def main():
    env_cfg = parse_env_cfg(
        TASK_ID,
        device="cuda:0",
        num_envs=1,
        use_fabric=True,
        entry_point_key="play_env_cfg_entry_point",
    )
    env = gym.make(TASK_ID, cfg=env_cfg, render_mode=None)
    env.reset()

    isaac_names = list(env.unwrapped.scene["robot"].data.joint_names)

    print("\n" + "=" * 80, flush=True)
    print("DIAGNOSTIC: ELF3 joint_names resolved by IsaacLab", flush=True)
    print("=" * 80, flush=True)
    print(f"{'idx':>3} | {'IsaacLab order':<26} | {'amp.py expected':<26} | match", flush=True)
    print("-" * 80, flush=True)
    n = max(len(isaac_names), len(AMP_ISAAC_ORDER))
    n_mismatch = 0
    for i in range(n):
        a = isaac_names[i] if i < len(isaac_names) else "(missing)"
        b = AMP_ISAAC_ORDER[i] if i < len(AMP_ISAAC_ORDER) else "(missing)"
        match = "OK" if a == b else "MISMATCH"
        if a != b:
            n_mismatch += 1
        print(f"{i:>3} | {a:<26} | {b:<26} | {match}", flush=True)
    print("=" * 80, flush=True)
    if n_mismatch == 0:
        print(f"RESULT: joint orders are IDENTICAL ({len(isaac_names)} joints).", flush=True)
    else:
        print(f"RESULT: {n_mismatch} positions differ. Deployment WILL be miswired.", flush=True)
    print("=" * 80, flush=True)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
