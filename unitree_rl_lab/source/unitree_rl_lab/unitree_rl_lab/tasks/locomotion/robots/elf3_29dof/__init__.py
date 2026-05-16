"""ELF3 29-DOF (BXI) locomotion task — bxi amp.py deployment compatible."""

import gymnasium as gym

gym.register(
    id="Unitree-Elf3-29dof-Velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.velocity_env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents_cfg:Elf3PPORunnerCfg",
    },
)
