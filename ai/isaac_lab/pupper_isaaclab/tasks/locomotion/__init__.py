"""Register Pupper v3 velocity-tracking locomotion Gym tasks."""

from __future__ import annotations

import gymnasium as gym

from . import agents
from .pupper_env_cfg import (
    PupperFlatEnvCfg,
    PupperFlatEnvCfg_PLAY,
    PupperRoughEnvCfg,
    PupperRoughEnvCfg_PLAY,
)
from .agents.rsl_rl_ppo_cfg import PupperFlatPPORunnerCfg, PupperRoughPPORunnerCfg

gym.register(
    id="Pupper-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": PupperFlatEnvCfg,
        "rsl_rl_cfg_entry_point": PupperFlatPPORunnerCfg,
    },
)

gym.register(
    id="Pupper-Flat-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": PupperFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": PupperFlatPPORunnerCfg,
    },
)

gym.register(
    id="Pupper-Rough-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": PupperRoughEnvCfg,
        "rsl_rl_cfg_entry_point": PupperRoughPPORunnerCfg,
    },
)

gym.register(
    id="Pupper-Rough-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": PupperRoughEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": PupperRoughPPORunnerCfg,
    },
)
