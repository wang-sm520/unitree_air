from __future__ import annotations

from collections.abc import Sequence

import torch
from isaaclab.envs.mdp.actions import JointPositionActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.utils import DelayBuffer, configclass


class DelayedJointPositionAction(JointPositionAction):
    """Joint position action with randomized physics-step command latency."""

    cfg: DelayedJointPositionActionCfg

    def __init__(self, cfg: DelayedJointPositionActionCfg, env):
        if cfg.min_delay < 0:
            raise ValueError(f"min_delay must be non-negative, got {cfg.min_delay}.")
        if cfg.max_delay < cfg.min_delay:
            raise ValueError(
                f"max_delay must be greater than or equal to min_delay, got {cfg.max_delay} < {cfg.min_delay}."
            )

        super().__init__(cfg, env)
        self._delay_buffer = DelayBuffer(cfg.max_delay, self.num_envs, device=self.device)
        self.reset(slice(None))

    def reset(self, env_ids: Sequence[int] | slice | None = None) -> None:
        reset_env_ids = slice(None) if env_ids is None else env_ids
        super().reset(reset_env_ids)

        if isinstance(reset_env_ids, slice):
            start, stop, step = reset_env_ids.indices(self.num_envs)
            num_envs = len(range(start, stop, step))
        else:
            num_envs = len(reset_env_ids)

        time_lags = torch.randint(
            low=self.cfg.min_delay,
            high=self.cfg.max_delay + 1,
            size=(num_envs,),
            dtype=torch.int,
            device=self.device,
        )
        self._delay_buffer.set_time_lag(time_lags, reset_env_ids)
        self._delay_buffer.reset(reset_env_ids)

    def apply_actions(self):
        delayed_actions = self._delay_buffer.compute(self.processed_actions)
        self._asset.set_joint_position_target(delayed_actions, joint_ids=self._joint_ids)


@configclass
class DelayedJointPositionActionCfg(JointPositionActionCfg):
    """Configuration for randomized-latency joint position actions.

    The delay is counted in physics steps because :meth:`apply_actions` runs once per simulation step.
    With ``sim.dt = 0.005``, ``min_delay=10`` and ``max_delay=20`` correspond to 50--100 ms.
    """

    class_type: type = DelayedJointPositionAction

    min_delay: int = 0
    """Minimum action latency in physics steps."""

    max_delay: int = 0
    """Maximum action latency in physics steps."""
