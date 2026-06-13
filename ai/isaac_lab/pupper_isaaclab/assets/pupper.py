"""Pupper v3 articulation configuration for Isaac Lab.

The numbers here are the single source of truth shared with the deployment
contract (mirrors ``pupperv3_isaac_sim/config.py`` and the metadata embedded in
the RTNeural policy JSON):

  * joint order ("policy order"),
  * default standing pose,
  * joint limits,
  * PD gains used while the policy is active (kp=5, kd=0.25).

The USD asset is produced by ``scripts/convert_pupper_urdf.py`` from
``pupper_v3.edited.fixed.urdf`` (with the dummy ``world`` link / floating joint
stripped so ``base_link`` becomes the free-floating articulation root).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

# --------------------------------------------------------------------------
# Deployment contract (kept identical to pupperv3_isaac_sim/config.py).
# --------------------------------------------------------------------------

# Canonical "policy order" — MUST match neural_controller config.yaml joint_names.
JOINT_NAMES: List[str] = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]

# Standing pose (rad), in policy order.
DEFAULT_JOINT_POS: List[float] = [
    0.26, 0.0, -0.52,
    -0.26, 0.0, 0.52,
    0.26, 0.0, -0.52,
    -0.26, 0.0, 0.52,
]

JOINT_LOWER_LIMITS: List[float] = [
    -1.22, -0.42, -2.79,
    -2.51, -3.14, -0.71,
    -1.22, -0.42, -2.79,
    -2.51, -3.14, -0.71,
]
JOINT_UPPER_LIMITS: List[float] = [
    2.51, 3.14, 0.71,
    1.22, 0.42, 2.79,
    2.51, 3.14, 0.71,
    1.22, 0.42, 2.79,
]

# PD gains while the policy is active (force-mode PD, matches MuJoCo training).
KP: float = 5.0
KD: float = 0.25
ACTION_SCALE: float = 1.0
SPAWN_HEIGHT: float = 0.22


@dataclass
class PupperContract:
    """Static contract shared with training, export and deployment."""

    joint_names: List[str] = field(default_factory=lambda: list(JOINT_NAMES))
    default_joint_pos: List[float] = field(default_factory=lambda: list(DEFAULT_JOINT_POS))
    joint_lower_limits: List[float] = field(default_factory=lambda: list(JOINT_LOWER_LIMITS))
    joint_upper_limits: List[float] = field(default_factory=lambda: list(JOINT_UPPER_LIMITS))
    kp: float = KP
    kd: float = KD
    action_scale: float = ACTION_SCALE
    spawn_height: float = SPAWN_HEIGHT
    # Per-step observation layout (36) and history length (matches MJX training).
    single_observation_size: int = 36
    observation_history: int = 20


PUPPER_CONTRACT = PupperContract()

DEFAULT_JOINT_POS_BY_NAME = dict(zip(JOINT_NAMES, DEFAULT_JOINT_POS))

# Location of the converted USD. ``convert_pupper_urdf.py`` writes here.
_ASSETS_DIR = os.path.dirname(os.path.abspath(__file__))
PUPPER_USD_PATH = os.path.join(_ASSETS_DIR, "usd", "pupper_v3.usd")


# --------------------------------------------------------------------------
# Articulation configuration.
# --------------------------------------------------------------------------
# effort_limit_sim=3.0 N·m matches the MJX MJCF actuator ``forcerange="-3 3"``.
# We keep the ImplicitActuator (PhysX PD at the 200 Hz physics rate, which mirrors
# the real robot's 520 Hz PD) and model the sim2real gap as a 1-step ACTION delay
# instead (see ``DelayedJointPositionActionCfg`` in the env cfg). This matches MJX
# (250 Hz PD + 50 Hz 1-step action latency) -- the recipe that actually walks on
# this hardware. (An explicit DelayedPDActuator runs PD at only 50 Hz, which rings
# and blows up dof_acc, so it is the wrong model for this high-PD-rate robot.)
PUPPER_V3_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=PUPPER_USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, SPAWN_HEIGHT),
        joint_pos=dict(DEFAULT_JOINT_POS_BY_NAME),
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.95,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit_sim=3.0,
            velocity_limit_sim=30.0,
            stiffness={".*": KP},
            damping={".*": KD},
        ),
    },
)
