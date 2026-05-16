"""Standalone ONNX exporter for elf3 RSL-RL policies.

Produces an ONNX file that matches the bxi `amp.py` deployment contract
(`HumanoidGaitPolicyLite`):

    session.run(["actions"], {"obs": obs_tensor})[0][0]

- Input  name: "obs",      shape [1, 960]  (96 single-frame * 10 history)
- Output name: "actions",  shape [1, num_actions]
- Opset 11, dynamic batch axis.

Usage:
    python scripts/export_elf3_onnx.py \\
        --checkpoint logs/rsl_rl/.../model_xxx.pt \\
        --output /path/to/policy.onnx \\
        [--task Unitree-Elf3-29dof-Velocity]
"""

import argparse
from importlib.metadata import version

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Export an elf3 RSL-RL policy to ONNX for bxi amp.py.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the RSL-RL .pt checkpoint to export.")
parser.add_argument("--output", type=str, required=True, help="Destination path for the exported .onnx file.")
parser.add_argument(
    "--task",
    type=str,
    default="Unitree-Elf3-29dof-Velocity",
    help="Gym task id used to build the env (so observation/action dims are read from the registered cfg).",
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to instantiate (1 is sufficient for export)."
)
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument(
    "--expected_obs_dim",
    type=int,
    default=960,
    help="Asserted single-batch obs dimension (default 960 = 96 * 10). Set to 0 to skip the assertion.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import copy
import os

import gymnasium as gym
import torch
import torch.nn as nn

from rsl_rl.runners import OnPolicyRunner

import isaaclab_tasks  # noqa: F401
from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.utils.assets import retrieve_file_path
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
from isaaclab_tasks.utils import load_cfg_from_registry

import unitree_rl_lab.tasks  # noqa: F401
from unitree_rl_lab.utils.parser_cfg import parse_env_cfg


class _PolicyWrapper(nn.Module):
    """Wraps `normalizer -> actor_mlp` into a flat module taking [B, obs_dim] -> [B, num_actions]."""

    def __init__(self, actor: nn.Module, normalizer: nn.Module | None):
        super().__init__()
        self.normalizer = copy.deepcopy(normalizer) if normalizer is not None else nn.Identity()
        self.actor = copy.deepcopy(actor)
        if hasattr(self.actor, "distribution"):
            del self.actor.distribution

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.actor(self.normalizer(obs))


def _resolve_actor_module(policy_nn: nn.Module) -> nn.Module:
    """Pick the deterministic actor sub-network from an RSL-RL policy across versions."""
    if hasattr(policy_nn, "actor") and hasattr(policy_nn.actor, "mlp"):
        return policy_nn.actor.mlp
    if hasattr(policy_nn, "actor"):
        return policy_nn.actor
    if hasattr(policy_nn, "student") and hasattr(policy_nn.student, "mlp"):
        return policy_nn.student.mlp
    if hasattr(policy_nn, "student"):
        return policy_nn.student
    raise RuntimeError("Could not locate actor module on the RSL-RL policy.")


def _resolve_normalizer(policy_nn: nn.Module) -> nn.Module | None:
    for attr in ("actor_obs_normalizer", "student_obs_normalizer", "obs_normalizer"):
        if hasattr(policy_nn, attr):
            return getattr(policy_nn, attr)
    return None


def main():
    # Build env cfg via the project's helper (mirrors play.py).
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
        entry_point_key="play_env_cfg_entry_point",
    )
    agent_cfg: RslRlOnPolicyRunnerCfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, version("rsl-rl-lib"))

    resume_path = retrieve_file_path(args_cli.checkpoint)
    print(f"[INFO] Loading checkpoint: {resume_path}")

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=None)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # Sanity-check the observation dimension matches the bxi contract.
    obs_space = env.observation_space
    obs_shape = getattr(obs_space, "shape", None)
    if obs_shape is None or len(obs_shape) < 1:
        raise RuntimeError(f"Unexpected observation_space: {obs_space}")
    obs_dim = int(obs_shape[-1])
    print(f"[INFO] Env observation dim: {obs_dim}")
    if args_cli.expected_obs_dim > 0 and obs_dim != args_cli.expected_obs_dim:
        raise AssertionError(
            f"obs_dim {obs_dim} != expected {args_cli.expected_obs_dim}. "
            "bxi amp.py requires 960 (96 * 10 history). Adjust env or pass --expected_obs_dim 0 to override."
        )

    # Build runner and load checkpoint.
    if not hasattr(agent_cfg, "class_name") or agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class for ONNX export: {agent_cfg.class_name}")
    runner.load(resume_path)

    # Locate the actor module across rsl-rl versions (same hasattr chain as play.py).
    if hasattr(runner.alg, "actor"):
        policy_nn = runner.alg.actor
    elif hasattr(runner.alg, "policy"):
        policy_nn = runner.alg.policy
    else:
        policy_nn = runner.alg.actor_critic

    if getattr(policy_nn, "is_recurrent", False):
        raise NotImplementedError(
            "This exporter only supports feed-forward policies; bxi amp.py expects a stateless ONNX."
        )

    actor_module = _resolve_actor_module(policy_nn)
    normalizer = _resolve_normalizer(policy_nn)

    exporter = _PolicyWrapper(actor_module, normalizer).cpu().eval()

    # Verify the wrapper runs and produces the expected output shape.
    dummy = torch.zeros(1, obs_dim, dtype=torch.float32)
    with torch.no_grad():
        sample_out = exporter(dummy)
    num_actions = int(sample_out.shape[-1])
    print(f"[INFO] Action dim: {num_actions}")

    output_path = os.path.abspath(args_cli.output)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    torch.onnx.export(
        exporter,
        dummy,
        output_path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
    )
    print(f"[INFO] Wrote ONNX to: {output_path}")

    # Verify with onnxruntime and assert the bxi contract.
    import onnxruntime as ort

    sess = ort.InferenceSession(output_path, providers=["CPUExecutionProvider"])
    in_info = sess.get_inputs()[0]
    out_info = sess.get_outputs()[0]
    print(f"Input : name={in_info.name}  shape={in_info.shape}  type={in_info.type}")
    print(f"Output: name={out_info.name} shape={out_info.shape} type={out_info.type}")

    assert in_info.name == "obs", f"ONNX input name is {in_info.name!r}, expected 'obs' (bxi amp.py contract)."
    assert out_info.name == "actions", (
        f"ONNX output name is {out_info.name!r}, expected 'actions' (bxi amp.py contract)."
    )

    import numpy as np

    np_dummy = np.zeros((1, obs_dim), dtype=np.float32)
    rt_out = sess.run(["actions"], {"obs": np_dummy})[0]
    print(f"[INFO] onnxruntime smoke run OK, output shape={rt_out.shape}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
