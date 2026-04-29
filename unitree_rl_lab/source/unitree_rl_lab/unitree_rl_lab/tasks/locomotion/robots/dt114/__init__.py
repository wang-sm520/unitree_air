import gymnasium as gym

gym.register(
    id="Unitree-DT114-Velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.velocity_env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

gym.register(
    id="Unitree-DT114-New-Velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.new_velocity_env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.new_velocity_env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

gym.register(
    id="Unitree-DT114-XR-Stand",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stand_env_cfg:DT114StandEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.stand_env_cfg:DT114StandPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:XRPPORunnerCfg",
    },
)

gym.register(
    id="Unitree-DT114-XR-NoNoise",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "test_cfg_entry_point": f"{__name__}.xrNoNoise_env_cfg:DT114TestEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:XRPPORunnerCfg",
    },
)


gym.register(                                                                 
    id="Unitree-DT114-WSM-Velocity",                                          
    entry_point="isaaclab.envs:ManagerBasedRLEnv",                            
    disable_env_checker=True,                                                 
    kwargs={                                                                  
        "env_cfg_entry_point": f"{__name__}.wsm_velocity_cfg:RobotEnvCfg",    
        "play_env_cfg_entry_point":                                           
f"{__name__}.wsm_velocity_cfg:RobotPlayEnvCfg",                               
        "rsl_rl_cfg_entry_point":                                             
f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",    
    },                                                                       
) 