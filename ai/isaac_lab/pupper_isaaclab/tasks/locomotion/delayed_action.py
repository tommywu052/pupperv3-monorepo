"""1-step action-latency joint-position action term.

The idealized sim applies the policy's joint-position target on the same control
step it is produced. The real Pupper has ~1 control-step (50 Hz) latency between
the policy output and the target reaching the motor PD loop. A policy trained
without this latency learns a gait that relies on instantaneous target tracking;
on hardware (more dissipative) that gait limit cycle is damped out and the robot
only leans & holds. Modelling the latency here -- exactly like the MJX training
(``latency_distribution: [0.2, 0.8]`` = 1-step delay 80% of the time) -- forces a
gait that stays self-exciting on the real robot.

We keep the high-rate (200 Hz physics) ImplicitActuator PD, which mirrors the
robot's 520 Hz PD, and only delay the *target*. This avoids the dof-acc ringing of
an explicit 50 Hz DelayedPDActuator.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class DelayedJointPositionAction(JointPositionAction):
    """JointPositionAction that applies last step's target with probability ``delay_prob``."""

    cfg: "DelayedJointPositionActionCfg"

    def __init__(self, cfg: "DelayedJointPositionActionCfg", env: "ManagerBasedEnv"):
        super().__init__(cfg, env)
        self._delay_prob = float(cfg.delay_prob)
        # offset == default joint pos (use_default_offset), i.e. the standing target,
        # so initialising the "previous" buffer to it is a benign no-latency start.
        self._prev_processed = self._offset.clone()
        self._applied = self._offset.clone()

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)  # fills self._processed_actions for this step
        use_delayed = torch.rand(self.num_envs, 1, device=self.device) < self._delay_prob
        self._applied = torch.where(use_delayed, self._prev_processed, self._processed_actions)
        self._prev_processed = self._processed_actions.clone()

    def apply_actions(self):
        self._asset.set_joint_position_target(self._applied, joint_ids=self._joint_ids)

    def reset(self, env_ids=None) -> None:
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._prev_processed[ids] = self._offset[ids]
        self._applied[ids] = self._offset[ids]


@configclass
class DelayedJointPositionActionCfg(JointPositionActionCfg):
    """Config for :class:`DelayedJointPositionAction`."""

    class_type: type = DelayedJointPositionAction
    delay_prob: float = 0.8
    """Probability of applying the previous step's target (1-step latency). MJX: 0.8."""
