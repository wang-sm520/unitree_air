"""Test PD control - modify target joint positions in MuJoCo order."""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import yaml

from deploy_mujoco import get_project_root


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


# Predefined poses (MuJoCo joint order)
# ArmNew4 MuJoCo actuator order (indices 0-28):
# 0-2: waist_yaw, waist_roll, torso
# 3-9: right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow, right_wrist_roll, right_wrist_yaw, right_wrist_pitch
# 10-16: left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow, left_wrist_roll, left_wrist_yaw, left_wrist_pitch
# 17-22: right_hip_pitch, right_hip_roll, right_hip_yaw, right_knee, right_ankle_pitch, right_ankle_roll
# 23-28: left_hip_pitch, left_hip_roll, left_hip_yaw, left_knee, left_ankle_pitch, left_ankle_roll
POSES = {
    "default": {},
    "knee_bend": {
        20: 0.8,   # right_knee
        26: 0.8,   # left_knee
        17: 0.3,   # right_hip_pitch
        23: 0.3,   # left_hip_pitch
        21: -0.3,  # right_ankle_pitch
        27: -0.3,  # left_ankle_pitch
    },
    "deep_squat": {
        17: 0.6,   # right_hip_pitch
        20: 1.2,   # right_knee
        21: -0.6,  # right_ankle_pitch
        23: 0.6,   # left_hip_pitch
        26: 1.2,   # left_knee
        27: -0.6,  # left_ankle_pitch
    },
    "lean_forward": {
        17: 0.4,   # right_hip_pitch
        23: 0.4,   # left_hip_pitch
    },
    "left_leg_lift": {
        23: 0.8,   # left_hip_pitch
        26: 1.2,   # left_knee
    },
    "right_leg_lift": {
        17: 0.8,   # right_hip_pitch
        20: 1.2,   # right_knee
    },
    "stand_tall": {
        17: -0.1,  # right_hip_pitch
        23: -0.1,  # left_hip_pitch
    },
    "arms_tpose": {
        4: -1.5,   # right_shoulder_roll
        11: 1.5,   # left_shoulder_roll
    },
    # Arms forward, legs half squat - all 29 joints (0-28)
    "arms_forward_half_squat": {
        # Waist (0-2)
        0: 0.0,    # waist_yaw
        1: 0.0,    # waist_roll
        2: 0.0,    # torso
        # Right arm (3-9)
        3: 0.3,    # right_shoulder_pitch
        4: -0.2,    # right_shoulder_roll
        5: 0.0,    # right_shoulder_yaw
        6: -0.3,    # right_elbow
        7: 0.0,    # right_wrist_roll
        8: 0.0,    # right_wrist_yaw
        9: 0.0,    # right_wrist_pitch
        # Left arm (10-16)
        10: 0.3,   # left_shoulder_pitch
        11: 0.2,   # left_shoulder_roll
        12: 0.0,   # left_shoulder_yaw
        13: -0.3,   # left_elbow
        14: 0.0,   # left_wrist_roll
        15: 0.0,   # left_wrist_yaw
        16: 0.0,   # left_wrist_pitch
        # Right leg (17-22)
        17: -0.212,   # right_hip_pitch
        18: -0.7,   # right_hip_roll
        19: 0.7,   # right_hip_yaw
        20: 0.712,   # right_knee
        21: -0.70,  # right_ankle_pitch
        22: 0.0,   # right_ankle_roll
        # Left leg (23-28)
        23: -0.212,   # left_hip_pitch
        24: 0.0,   # left_hip_roll
        25: -0.0,   # left_hip_yaw
        26: 0.712,   # left_knee
        27: -0.70,  # left_ankle_pitch
        28: 0.0,   # left_ankle_roll
    },
}


def main():
    parser = argparse.ArgumentParser(description="Test PD control")
    parser.add_argument("config_file", type=str, help="Config file")
    parser.add_argument("--pose", "-p", type=str, default="default",
                        choices=list(POSES.keys()),
                        help="Pose name")
    parser.add_argument("--duration", "-d", type=float, default=30.0,
                        help="Simulation duration (s)")
    args = parser.parse_args()

    if Path(args.config_file).is_absolute():
        config_path = Path(args.config_file)
    else:
        config_path = Path(__file__).parent / "configs" / args.config_file

    # Load config
    with open(config_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    project_root = get_project_root()

    # Load robot params
    robot_params_path = config["robot_params_path"].replace("{PROJECT_ROOT}", str(project_root))
    with open(robot_params_path, "r") as f:
        robot_params = yaml.load(f, Loader=yaml.FullLoader)

    kps = np.array(robot_params["stiffness"], dtype=np.float32)
    kds = np.array(robot_params["damping"], dtype=np.float32)
    default_angles = np.array(robot_params["default_joint_pos"], dtype=np.float32)

    # Load MuJoCo model
    xml_path = config["xml_path"].replace("{PROJECT_ROOT}", str(project_root))
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = config.get("simulation_dt", 0.002)

    n_actuators = m.nu
    print(f"Model: {xml_path}")
    print(f"Actuators: {n_actuators}")

    # Build default target in MuJoCo order
    joint_ids_map = np.array(robot_params["joint_ids_map"]) if robot_params.get("joint_ids_map") else None
    if joint_ids_map is not None:
        # Map default_angles (policy order) to MuJoCo order
        target_mujoco = np.zeros(n_actuators, dtype=np.float32)
        for mujoco_idx, policy_idx in enumerate(joint_ids_map):
            if policy_idx < len(default_angles):
                target_mujoco[mujoco_idx] = default_angles[policy_idx]
    else:
        target_mujoco = default_angles.copy()

    # Apply pose modifications
    print(f"\nPose: {args.pose}")
    if args.pose in POSES:
        for idx, value in POSES[args.pose].items():
            if idx < n_actuators:
                target_mujoco[idx] = value
                print(f"  Joint {idx}: {value:.2f}")

    # Set initial position
    d.qpos[7:] = target_mujoco
    mujoco.mj_forward(m, d)

    print(f"\nStarting simulation for {args.duration}s...")

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start_time = time.time()

        while viewer.is_running() and time.time() - start_time < args.duration:
            step_start = time.time()

            qj = d.qpos[7:].copy()
            dqj = d.qvel[6:].copy()

            tau = pd_control(target_mujoco, qj, kps, np.zeros_like(kds), dqj, kds)
            d.ctrl[:] = tau

            mujoco.mj_step(m, d)
            viewer.sync()

            dt = m.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)

    print("Done.")


if __name__ == "__main__":
    main()
