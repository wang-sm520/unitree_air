"""PPO config override for elf3 — higher entropy to keep exploration alive."""

from isaaclab.utils import configclass

from unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg import BasePPORunnerCfg


@configclass
class Elf3PPORunnerCfg(BasePPORunnerCfg):
    """Elf3-specific PPO override.

    Reason: prior elf3 runs collapsed action std to ~0.22 and got stuck in a
    stand-still local optimum. Higher entropy_coef counter-acts premature
    exploration collapse.
    """

    def __post_init__(self):
        # Override the algorithm's entropy_coef while keeping the rest of BasePPO.
        self.algorithm.entropy_coef = 0.025
        # Slightly larger initial std to bias toward action exploration on restart.
        self.policy.init_noise_std = 1.0
