# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Spawn-stability check: zero-action rollout to verify the initial pose is statically stable.

Loads the play env cfg of a task, spawns N envs, and steps the simulation with
torch.zeros actions (i.e. policy outputs nothing — robot is only held by PD against
init_state.joint_pos). Periodically prints base height / orientation / joint deviation.

Useful for sanity-checking changes to `init_state` (pos.z, joint_pos defaults) in
unitree.py before launching a full training run.

Example:
    cd /home/hpf/wsm/unitree_rl/unitree_rl_lab
    ./unitree_rl_lab.sh -p ... # NOTE: this script is not wired into unitree_rl_lab.sh
    # Run directly:
    python scripts/rsl_rl/spawn_check.py --task Unitree-LJ-Velocity --num_envs 4 --duration 5.0
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Zero-action spawn stability check.")
parser.add_argument("--task", type=str, default="Unitree-LJ-Velocity", help="Task name (must have play_env_cfg_entry_point).")
parser.add_argument("--num_envs", type=int, default=4, help="Number of envs to spawn.")
parser.add_argument("--duration", type=float, default=5.0, help="Seconds of zero-action simulation to run.")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations.")
parser.add_argument("--print_every", type=float, default=0.5, help="Print interval in seconds.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import time

import gymnasium as gym
import torch

import unitree_rl_lab.tasks  # noqa: F401
from unitree_rl_lab.utils.parser_cfg import parse_env_cfg


def main():
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
        entry_point_key="play_env_cfg_entry_point",
    )

    # Neutralize ALL randomization so we observe the *pure* init_state.
    # Without this, friction randomization (0.1–1.0) gives some envs slippery
    # feet that slide out under load and cause forward fall after ~1s, masking
    # whether the pose itself is statically stable.
    try:
        # reset events
        env_cfg.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}
        env_cfg.events.reset_base.params["velocity_range"] = {
            "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
            "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
        }
        env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        env_cfg.events.reset_robot_joints.params["velocity_range"] = (0.0, 0.0)
        # startup events — fire once at env creation, must be neutralized in cfg
        env_cfg.events.physics_material.params["static_friction_range"] = (1.0, 1.0)
        env_cfg.events.physics_material.params["dynamic_friction_range"] = (1.0, 1.0)
        env_cfg.events.physics_material.params["restitution_range"] = (0.0, 0.0)
        env_cfg.events.add_base_mass.params["mass_distribution_params"] = (0.0, 0.0)
        # interval push — disable so it doesn't fire during the rollout
        env_cfg.events.push_robot.params["velocity_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0)}
        print("[INFO] neutralized: reset randomization, friction/mass randomization, push_robot")
    except Exception as e:
        print(f"[WARN] could not neutralize randomization: {e}")

    env = gym.make(args_cli.task, cfg=env_cfg)
    print(f"[INFO] env created: num_envs={env.unwrapped.num_envs}, action_dim={env.action_space.shape}")

    dt = env.unwrapped.step_dt
    total_steps = max(1, int(math.ceil(args_cli.duration / dt)))
    print_every_steps = max(1, int(round(args_cli.print_every / dt)))
    print(f"[INFO] step_dt={dt:.4f}s | running {total_steps} steps (~{args_cli.duration:.1f}s)")

    obs, _ = env.reset()

    # Zero action template: shape (num_envs, action_dim)
    action_shape = env.action_space.shape
    zero_action = torch.zeros(action_shape, device=env.unwrapped.device)

    robot = env.unwrapped.scene["robot"]
    num_envs = env.unwrapped.num_envs
    device = env.unwrapped.device
    init_z = robot.data.root_pos_w[:, 2].clone()
    print(f"[INFO] initial base_z (per-env mean): {init_z.mean().item():.4f} m | min: {init_z.min().item():.4f} | max: {init_z.max().item():.4f}")

    # Per-env running stats — updated EVERY step, not just at print intervals.
    # Tilt and base_z drop are tracked across the whole rollout so post-reset
    # samples can't hide a fall that happened between print points.
    max_tilt_deg_per_env = torch.zeros(num_envs, device=device)
    min_base_z_per_env = init_z.clone()
    reset_count_per_env = torch.zeros(num_envs, dtype=torch.long, device=device)

    # Track episode_length_buf to detect resets (it goes to 0 on reset).
    # ManagerBasedRLEnv exposes this directly.
    has_ep_buf = hasattr(env.unwrapped, "episode_length_buf")
    if has_ep_buf:
        prev_ep_len = env.unwrapped.episode_length_buf.clone()
    else:
        prev_ep_len = None

    start_wall = time.time()
    for step_idx in range(total_steps):
        with torch.inference_mode():
            obs, _, terminated, truncated, _ = env.step(zero_action)

        # Per-step state (BEFORE any reset that happens inside the next step()).
        base_z = robot.data.root_pos_w[:, 2]
        proj_grav_z = robot.data.projected_gravity_b[:, 2]
        uprightness = (-proj_grav_z).clamp(-1.0, 1.0)
        tilt_deg = torch.rad2deg(torch.acos(uprightness))

        # Update running maxima per env.
        max_tilt_deg_per_env = torch.maximum(max_tilt_deg_per_env, tilt_deg)
        min_base_z_per_env = torch.minimum(min_base_z_per_env, base_z)

        # Detect resets: episode_length_buf decreased (or stayed at 0 after being non-zero).
        if has_ep_buf:
            cur_ep_len = env.unwrapped.episode_length_buf
            reset_mask = cur_ep_len < prev_ep_len  # this step triggered termination/truncation
            reset_count_per_env += reset_mask.long()
            prev_ep_len = cur_ep_len.clone()
        else:
            # Fallback: use terminated/truncated returned by step().
            done = (terminated | truncated) if isinstance(terminated, torch.Tensor) else None
            if done is not None:
                reset_count_per_env += done.long()

        if step_idx % print_every_steps == 0 or step_idx == total_steps - 1:
            t = (step_idx + 1) * dt
            cum_resets = reset_count_per_env.sum().item()
            print(
                f"  t={t:5.2f}s  base_z[mean/min/max]={base_z.mean():.3f}/{base_z.min():.3f}/{base_z.max():.3f}m  "
                f"tilt[mean/max]={tilt_deg.mean():.1f}/{tilt_deg.max():.1f}°  "
                f"cum_resets={cum_resets}"
            )

    wall = time.time() - start_wall
    print(f"[INFO] done in {wall:.1f}s wall-time")

    total_resets = reset_count_per_env.sum().item()
    envs_that_reset = (reset_count_per_env > 0).sum().item()
    max_tilt_overall = max_tilt_deg_per_env.max().item()
    max_tilt_mean = max_tilt_deg_per_env.mean().item()
    max_drop = (init_z - min_base_z_per_env).max().item()
    mean_drop = (init_z - min_base_z_per_env).mean().item()

    print("\n=== SPAWN STABILITY SUMMARY ===")
    print(f"  init_z (mean):                 {init_z.mean().item():.4f} m")
    print(f"  lowest base_z (min/mean):      {min_base_z_per_env.min().item():.4f} / {min_base_z_per_env.mean().item():.4f} m")
    print(f"  max base_z drop (max/mean):    {max_drop:.4f} / {mean_drop:.4f} m")
    print(f"  max tilt over rollout (max/mean): {max_tilt_overall:.1f}° / {max_tilt_mean:.1f}°")
    print(f"  total resets (sum / envs-affected): {total_resets} / {envs_that_reset}/{num_envs}")

    # Verdict — based on whole-rollout state, not just last frame.
    if total_resets > 0:
        print(f"  verdict: ❌ UNSTABLE — {envs_that_reset}/{num_envs} envs reset (termination triggered, robot fell)")
    elif max_tilt_overall > 25.0:
        print(f"  verdict: ❌ UNSTABLE — no reset, but peak tilt {max_tilt_overall:.1f}° indicates falling motion")
    elif max_tilt_overall > 10.0 or max_drop > 0.05:
        print(f"  verdict: ⚠️  MARGINAL — settled but oscillates (peak tilt {max_tilt_overall:.1f}°, max drop {max_drop*100:.1f}cm)")
    else:
        print(f"  verdict: ✅ STABLE — peak tilt {max_tilt_overall:.1f}°, max drop {max_drop*100:.1f}cm, no resets")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
