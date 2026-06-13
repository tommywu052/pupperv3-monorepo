"""Convert the Pupper v3 URDF into a USD asset for Isaac Lab.

Two things matter for a clean import (both learned the hard way in
``pupperv3_isaac_sim``):

1. The shipped URDF has a dummy ``world`` link joined to ``base_link`` by a
   ``floating`` joint. Isaac's importer turns ``world`` into the articulation
   root, which cannot hold a floating joint inside a PhysX articulation and
   yields **zero** controllable DOFs. We strip ``world`` and any floating /
   ``parent=world`` joint first so ``base_link`` becomes the free-floating root.
2. We import as a *floating base* (``fix_base=False``) with position drives.
   Per-joint kp/kd are set by the ArticulationCfg actuators (kp=5, kd=0.25),
   not here, so the drive gains below are placeholders.

Run with Isaac Lab's launcher (Windows)::

    cd C:\\Nvidia\\IsaacLab\\IsaacLab
    .\\isaaclab.bat -p C:\\Nvidia\\pupperv3\\pupperv3-monorepo\\ai\\isaac_lab\\scripts\\convert_pupper_urdf.py --headless
"""

from __future__ import annotations

import argparse
import os
import xml.etree.ElementTree as ET

from isaaclab.app import AppLauncher

# --------------------------------------------------------------------------
# Default paths.
# --------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_THIS_DIR)  # ai/isaac_lab
# ai/isaac_lab -> ai -> pupperv3-monorepo
_MONOREPO = os.path.dirname(os.path.dirname(_PKG_ROOT))
_DEFAULT_URDF = os.path.join(
    _MONOREPO, "ros2_ws", "src", "pupper_v3_description",
    "description", "urdf", "pupper_v3.edited.fixed.urdf",
)
_DEFAULT_OUT = os.path.join(_PKG_ROOT, "pupper_isaaclab", "assets", "usd", "pupper_v3.usd")


def parse_args():
    parser = argparse.ArgumentParser(description="Convert Pupper v3 URDF -> USD for Isaac Lab.")
    parser.add_argument("--urdf", default=_DEFAULT_URDF, help="path to the input URDF")
    parser.add_argument("--out", default=_DEFAULT_OUT, help="path to the output USD")
    parser.add_argument("--joint-stiffness", type=float, default=5.0)
    parser.add_argument("--joint-damping", type=float, default=0.25)
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def prepare_floating_base_urdf(urdf_path: str) -> str:
    """Strip the dummy ``world`` link / floating joints; return cleaned URDF path.

    Written next to the original so ``package://`` / relative mesh paths keep
    resolving. Returns the original path unchanged if nothing needed removing.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    removed_links, removed_joints = [], []
    for link in list(root.findall("link")):
        if link.get("name") == "world":
            root.remove(link)
            removed_links.append("world")
    for joint in list(root.findall("joint")):
        parent = joint.find("parent")
        parent_link = parent.get("link") if parent is not None else None
        if joint.get("type") == "floating" or parent_link == "world":
            root.remove(joint)
            removed_joints.append(joint.get("name"))

    if not removed_links and not removed_joints:
        return urdf_path

    out_path = os.path.join(os.path.dirname(os.path.abspath(urdf_path)), "_isaac_floating_base.urdf")
    tree.write(out_path, xml_declaration=True, encoding="utf-8")
    print(f"[pupper] prepared floating-base URDF: removed links={removed_links} "
          f"joints={removed_joints} -> {out_path}")
    return out_path


def main():
    args = parse_args()

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    # Imports that require the running app.
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
    from isaaclab.utils.assets import check_file_path
    from isaaclab.utils.dict import print_dict

    urdf_path = os.path.abspath(args.urdf)
    if not check_file_path(urdf_path):
        raise ValueError(f"Invalid URDF path: {urdf_path}")

    clean_urdf = prepare_floating_base_urdf(urdf_path)

    dest_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(dest_path), exist_ok=True)

    cfg = UrdfConverterCfg(
        asset_path=clean_urdf,
        usd_dir=os.path.dirname(dest_path),
        usd_file_name=os.path.basename(dest_path),
        fix_base=False,
        merge_fixed_joints=True,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=args.joint_stiffness,
                damping=args.joint_damping,
            ),
            target_type="position",
        ),
    )

    print("-" * 80)
    print(f"Input URDF : {clean_urdf}")
    print(f"Output USD : {dest_path}")
    print("URDF converter config:")
    print_dict(cfg.to_dict(), nesting=0)
    print("-" * 80)

    converter = UrdfConverter(cfg)
    print(f"[pupper] generated USD: {converter.usd_path}")

    simulation_app.close()


if __name__ == "__main__":
    main()
