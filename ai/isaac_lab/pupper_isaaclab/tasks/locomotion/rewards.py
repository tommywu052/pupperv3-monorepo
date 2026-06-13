"""Custom reward terms for Pupper v3 locomotion.

``foot_clearance_reward`` is copied from Isaac Lab's Spot config (kept local so we
don't import the whole Spot package). It rewards *swinging* feet for reaching a
target height off the ground, which encourages higher steps to clear obstacles on
rough terrain. ``target_height`` is an ABSOLUTE world-z target for the foot link
origin, so it must be scaled to the (small) Pupper rather than Spot's 0.1 m.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def foot_clearance_reward(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg,
    target_height: float,
    std: float,
    tanh_mult: float,
) -> torch.Tensor:
    """Reward swinging feet for clearing ``target_height`` off the ground."""
    asset: RigidObject = env.scene[asset_cfg.name]
    foot_z_target_error = torch.square(asset.data.body_pos_w[:, asset_cfg.body_ids, 2] - target_height)
    foot_velocity_tanh = torch.tanh(tanh_mult * torch.norm(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=2))
    reward = foot_z_target_error * foot_velocity_tanh
    return torch.exp(-torch.sum(reward, dim=1) / std)
