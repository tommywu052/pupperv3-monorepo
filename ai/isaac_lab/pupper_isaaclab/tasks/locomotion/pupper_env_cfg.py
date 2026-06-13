"""Pupper v3 velocity-tracking locomotion environments (Isaac Lab, approach B).

Subclasses Isaac Lab's standard ``LocomotionVelocityRoughEnvCfg`` and adapts it
to the Pupper v3:

  * robot asset, body names (base_link, feet ``leg_.*_3``, thighs ``leg_.*_2``),
  * small-robot timing (200 Hz physics, 50 Hz control -> decimation 4),
  * command ranges and domain randomization ported from the MJX training config,
  * ASYMMETRIC actor-critic observations: the ``policy`` (actor) group contains
    only real-robot-measurable quantities (no base linear velocity, no height
    scan) so the exported policy is deployable on Jetson NX; the ``critic`` group
    additionally observes privileged base linear velocity (+ height scan on
    rough terrain).
"""

from __future__ import annotations

import math

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
)

from pupper_isaaclab.assets.pupper import PUPPER_V3_CFG
from pupper_isaaclab.tasks.locomotion.delayed_action import DelayedJointPositionActionCfg
from pupper_isaaclab.tasks.locomotion.rewards import foot_clearance_reward

# Pupper link-name patterns (after URDF import; lidar is merged into base_link).
BASE_LINK = "base_link"
FOOT_BODIES = "leg_.*_3"   # lower legs == feet
THIGH_BODIES = "leg_.*_2"  # upper legs == knees


@configclass
class PupperObservationsCfg:
    """Asymmetric observations: deployable actor + privileged critic."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Actor observations — must be reproducible on the real robot."""

        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Critic observations — may use privileged (sim-only) quantities."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        # privileged terrain height map: only the critic sees it, so the exported
        # actor stays blind (45-dim, deployable). Gives the value function terrain
        # awareness -> better advantages -> faster rough-terrain learning.
        # (disabled on flat ground where there is no height_scanner.)
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class PupperRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Pupper v3 on rough terrain."""

    observations: PupperObservationsCfg = PupperObservationsCfg()

    def __post_init__(self):
        super().__post_init__()

        # -- robot --
        self.scene.robot = PUPPER_V3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # -- timing: 200 Hz physics, 50 Hz control (matches the deployment loop) --
        self.sim.dt = 0.005
        self.decimation = 4
        self.episode_length_s = 10.0  # 500 steps @ 50 Hz (matches MJX episode_length)
        self.sim.render_interval = self.decimation

        # -- action: position targets offset by the default pose. scale 0.25 is
        # the proven Isaac Lab quadruped value; with Gaussian exploration noise
        # std=1.0, scale=1.0 makes the random early policy command +/-1 rad joint
        # targets and the robot falls within ~0.3 s (≈100% base-contact resets,
        # no learning signal). Under approach B action_scale is a free hyperparam
        # (the deployment node applies the same scale around the exported policy).
        # 1-step action latency (sim2real): keep the parent's joint targets but route
        # them through the delayed action term (see delayed_action.py). This is the
        # lever that lets the gait survive on the real robot.
        base_act = self.actions.joint_pos
        self.actions.joint_pos = DelayedJointPositionActionCfg(
            asset_name=base_act.asset_name,
            joint_names=base_act.joint_names,
            scale=0.25,
            use_default_offset=True,
            delay_prob=0.8,
        )

        # -- sensors / height scan attached to base_link --
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/" + BASE_LINK
        # scale down the rough terrain for a small robot (standing hip height
        # ~0.13-0.15 m, leg ~0.16 m). Defaults are human-scale (0.23 m stairs,
        # 22 deg slopes) which a small blind robot cannot learn.
        if self.scene.terrain.terrain_generator is not None:
            tg = self.scene.terrain.terrain_generator
            if "boxes" in tg.sub_terrains:
                tg.sub_terrains["boxes"].grid_height_range = (0.02, 0.06)
                tg.sub_terrains["boxes"].grid_width = 0.3
            if "random_rough" in tg.sub_terrains:
                tg.sub_terrains["random_rough"].noise_range = (0.01, 0.04)
                tg.sub_terrains["random_rough"].noise_step = 0.01
            # stairs: cap step height at ~0.08 m (~thigh-link length) with a
            # narrower 0.2 m tread; easy curriculum levels start at 0.02 m.
            for k in ("pyramid_stairs", "pyramid_stairs_inv"):
                if k in tg.sub_terrains:
                    tg.sub_terrains[k].step_height_range = (0.02, 0.05)
                    tg.sub_terrains[k].step_width = 0.2
            # slopes: cap at 0.3 (~17 deg) instead of the default 0.4 (~22 deg).
            for k in ("hf_pyramid_slope", "hf_pyramid_slope_inv"):
                if k in tg.sub_terrains:
                    tg.sub_terrains[k].slope_range = (0.0, 0.3)

        # -- command ranges (from MJX conf/training/default.yaml) --
        cmd = self.commands.base_velocity
        cmd.heading_command = False
        cmd.rel_standing_envs = 0.1  # raised from 0.02: train standstill more (reduce idle jitter)
        cmd.ranges.lin_vel_x = (-0.75, 0.75)
        cmd.ranges.lin_vel_y = (-0.5, 0.5)
        cmd.ranges.ang_vel_z = (-2.0, 2.0)

        # -- rewards: go2-style base, tracking weights from MJX (1.5 / 0.8) --
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.8
        self.rewards.lin_vel_z_l2.weight = -2.0
        self.rewards.ang_vel_xy_l2.weight = -0.05
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.dof_acc_l2.weight = -2.5e-7
        self.rewards.action_rate_l2.weight = -0.05  # raised from -0.01: smoother actions (less jitter)
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = FOOT_BODIES
        self.rewards.feet_air_time.weight = 0.01
        self.rewards.feet_air_time.params["threshold"] = 0.4
        if self.rewards.undesired_contacts is not None:
            self.rewards.undesired_contacts.params["sensor_cfg"].body_names = THIGH_BODIES
            self.rewards.undesired_contacts.weight = -1.0
        self.rewards.flat_orientation_l2.weight = 0.0
        self.rewards.dof_pos_limits.weight = -0.1
        # foot clearance: reward swinging feet for reaching ~0.05 m world-z so the
        # robot lifts its feet over obstacles on rough terrain. target_height is an
        # ABSOLUTE z target scaled to this small robot (Spot uses 0.1 at ~0.5 m tall);
        # tune target_height after watching play if steps look too low/high.
        self.rewards.foot_clearance = RewTerm(
            func=foot_clearance_reward,
            weight=0.3,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_BODIES),
                "target_height": 0.05,
                "std": 0.02,
                "tanh_mult": 2.0,
            },
        )

        # -- posture / anti-jitter -------------------------------------------
        # Fixes the crouched "belly-to-ground" gait and the idle shake seen on
        # hardware. The base reward set has NO height/posture term, so with the
        # soft KP=5 gains the policy learns to sink low (cheapest, most stable).
        # base_height_l2: pull the body up to a natural standing height. 0.14 m ~
        # the default-pose hip height; main knob to tune after watching play.
        # IMPORTANT: kept ABSOLUTE (no sensor_cfg) and modest weight on purpose.
        # A terrain-relative target (sensor_cfg=height_scanner) is UNBOUNDED: when
        # a ray misses the terrain (robot tipped / airborne / terrain edge) the
        # target spikes, the squared penalty explodes, and PPO drives the action-
        # noise std negative -> "RuntimeError: normal expects all elements of
        # std >= 0.0" (this crashed the 8192-env run at ~iter 850). Absolute z is
        # bounded, so it can't blow up the policy.
        self.rewards.base_height_l2 = RewTerm(
            func=mdp.base_height_l2,
            weight=-10.0,
            params={"target_height": 0.14},
        )
        # keep the hip-abduction joints tucked under the body (stops the legs
        # splaying out sideways, which drops the body and looks like crawling).
        self.rewards.joint_deviation_hip = RewTerm(
            func=mdp.joint_deviation_l1,
            weight=-0.5,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names="leg_.*_1")},
        )
        # idle anti-shake: when the velocity command is ~0, pin all joints back
        # to the default standing pose (only active for standing-still envs).
        self.rewards.stand_still = RewTerm(
            func=mdp.stand_still_joint_deviation_l1,
            weight=-0.5,
            params={"command_name": "base_velocity"},
        )

        # -- terminations --
        self.terminations.base_contact.params["sensor_cfg"].body_names = BASE_LINK

        # -- events / domain randomization (ported from MJX) --
        # friction 0.6..1.4
        self.events.physics_material.params["static_friction_range"] = (0.6, 1.4)
        self.events.physics_material.params["dynamic_friction_range"] = (0.6, 1.4)
        # base mass + COM jitter
        self.events.add_base_mass.params["asset_cfg"].body_names = BASE_LINK
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.25, 0.5)
        if self.events.base_com is not None:
            self.events.base_com.params["asset_cfg"].body_names = BASE_LINK
            self.events.base_com.params["com_range"] = {
                "x": (-0.02, 0.03), "y": (-0.005, 0.005), "z": (-0.005, 0.005),
            }
        self.events.base_external_force_torque.params["asset_cfg"].body_names = BASE_LINK
        # actuator-gain randomization (MJX kp_mult [0.6,1.1], kd_mult [0.8,1.5]) --
        # safe here because we use the ImplicitActuator (gains live in PhysX). Combined
        # with the 1-step action latency this is MJX's full sim2real recipe: the delay
        # keeps the gait *alive* on hardware, the gain spread makes it *robust/stable*
        # to the real PD/motor (delay-only walked but tipped; gain-only froze).
        # ``startup`` mode (per-env, once) -- per-reset CPU writes to PhysX are slow.
        self.events.randomize_actuator_gains = EventTerm(
            func=mdp.randomize_actuator_gains,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
                "stiffness_distribution_params": (0.6, 1.1),
                "damping_distribution_params": (0.8, 1.5),
                "operation": "scale",
                "distribution": "uniform",
            },
        )
        # kick: small random velocity pushes (MJX kick_vel ~0.1, kick_probability 0.04)
        if self.events.push_robot is not None:
            self.events.push_robot.interval_range_s = (3.0, 6.0)
            self.events.push_robot.params["velocity_range"] = {"x": (-0.2, 0.2), "y": (-0.2, 0.2)}
        # start near the default standing pose, at rest (the parent's large
        # initial velocity_range would otherwise fling this small robot off its
        # feet on reset -> ~95% immediate base-contact terminations).
        self.events.reset_robot_joints.params["position_range"] = (0.9, 1.1)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-math.pi, math.pi)},
            "velocity_range": {
                "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
            },
        }


@configclass
class PupperRoughEnvCfg_PLAY(PupperRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.scene.terrain.max_init_terrain_level = None
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None


@configclass
class PupperFlatEnvCfg(PupperRoughEnvCfg):
    """Pupper v3 on flat ground (no height scan; deployable actor unchanged)."""

    def __post_init__(self):
        super().__post_init__()
        # flat terrain
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan (also drop it from the privileged critic group)
        self.scene.height_scanner = None
        self.curriculum.terrain_levels = None
        # the privileged critic height_scan needs the height_scanner, which does
        # not exist on flat ground. (base_height_l2 is already absolute.)
        self.observations.critic.height_scan = None
        # flat-ground reward tweaks
        self.rewards.flat_orientation_l2.weight = -2.5
        self.rewards.feet_air_time.weight = 0.25


@configclass
class PupperFlatEnvCfg_PLAY(PupperFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
