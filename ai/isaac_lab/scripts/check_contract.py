#!/usr/bin/env python3
"""Print the REAL Isaac Lab runtime contract and compare it to the deploy node.

This resolves the #1 sim2real footgun: Isaac Lab orders articulation DOFs by its
own scheme (often grouped by kinematic-tree level: all hips, then all thighs,
then all calves), which need NOT match the URDF / MJX / hardware joint order that
the deploy node (``deploy/pupper_onnx_node.py``) assumes in ``JOINT_NAMES``.

If the policy was trained/played in Isaac Lab order but the Pi assembles the
observation and applies actions in ``JOINT_NAMES`` order, the joint_pos / joint_vel
slices of the observation AND the 12 action outputs are silently PERMUTED -> the
robot still stands and "steps", but the gait is scrambled (slides / spins instead
of tracking the command). play looks fine because there obs+action use the same
internal order.

Run (Windows)::

    cd C:\\Nvidia\\IsaacLab\\IsaacLab
    .\\isaaclab.bat -p C:\\Nvidia\\pupperv3\\pupperv3-monorepo\\ai\\isaac_lab\\scripts\\check_contract.py ^
        --task Pupper-Flat-Play-v0 --num_envs 1 --headless

It launches the sim, builds the env, prints the contract, then exits. No policy
needed.
"""

from __future__ import annotations

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Print Isaac Lab joint/obs contract.")
parser.add_argument("--task", type=str, default="Pupper-Flat-Play-v0")
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

# force headless: we only need the contract, not rendering
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
import pupper_isaaclab.tasks  # noqa: F401,E402  (registers Pupper-* tasks)

# deploy-side assumed order (keep in sync with deploy/pupper_onnx_node.py)
DEPLOY_JOINT_NAMES = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]


OUT_PATH = r"C:\Nvidia\pupperv3\contract_out.txt"


def main():
    lines = []

    def emit(s=""):
        print(s, flush=True)
        lines.append(str(s))

    try:
        from isaaclab_tasks.utils import parse_env_cfg

        env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
        env = gym.make(args_cli.task, cfg=env_cfg)
        u = env.unwrapped

        robot = u.scene["robot"]
        sim_order = list(robot.joint_names)

        emit("================ ISAAC LAB RUNTIME CONTRACT ================")
        emit(f"task                 : {args_cli.task}")
        emit(f"num joints           : {len(sim_order)}")
        emit("--- articulation joint order (robot.joint_names) ---")
        for i, n in enumerate(sim_order):
            emit(f"  [{i:2d}] {n}")

        # action term joint order (what the 12 policy outputs map to)
        try:
            act_term = u.action_manager.get_term("joint_pos")
            act_joint_ids = act_term._joint_ids
            if isinstance(act_joint_ids, slice):
                act_order = sim_order
            else:
                act_order = [sim_order[j] for j in list(act_joint_ids)]
            emit("--- action term 'joint_pos' output order (policy action -> joint) ---")
            for i, n in enumerate(act_order):
                emit(f"  action[{i:2d}] -> {n}")
        except Exception as e:  # noqa: BLE001
            act_order = sim_order
            emit(f"[warn] could not introspect action term: {e}; assuming sim_order")

        # observation term layout
        emit("--- observation term layout (policy group) ---")
        try:
            obs_mgr = u.observation_manager
            terms = obs_mgr.active_terms["policy"]
            dims = obs_mgr.group_obs_term_dim["policy"]
            off = 0
            for t, d in zip(terms, dims):
                n = int(d[0]) if hasattr(d, "__len__") else int(d)
                emit(f"  [{off:2d}:{off + n:2d}] {t}  (dim {n})")
                off += n
            emit(f"  total obs dim = {off}")
        except Exception as e:  # noqa: BLE001
            emit(f"[warn] could not introspect observation terms: {e}")

        # the verdict
        emit("================ COMPARISON TO DEPLOY ================")
        emit(f"deploy JOINT_NAMES   : {DEPLOY_JOINT_NAMES}")
        emit(f"isaac action order   : {act_order}")
        if act_order == DEPLOY_JOINT_NAMES:
            emit(">>> MATCH: deploy joint order is correct. Look elsewhere.")
        else:
            perm = [DEPLOY_JOINT_NAMES.index(n) for n in act_order]
            inv = [act_order.index(n) for n in DEPLOY_JOINT_NAMES]
            emit(">>> MISMATCH! joint order differs -> obs+action are permuted on the Pi.")
            emit(f"    SIM2DEPLOY perm (policy_slice = deploy_arr[perm]) = {perm}")
            emit(f"    DEPLOY2SIM inv  (deploy_arr[inv] = policy_vec)    = {inv}")

        env.close()
    except Exception as e:  # noqa: BLE001
        import traceback

        lines.append("EXCEPTION:\n" + traceback.format_exc())
    finally:
        with open(OUT_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
    simulation_app.close()
