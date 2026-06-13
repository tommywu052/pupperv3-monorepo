"""Pupper v3 Isaac Lab port.

Reinforcement-learning training package for the Pupper v3 quadruped on
NVIDIA Isaac Lab (PhysX GPU simulation + rsl_rl PPO).

Design (chosen approach):
  * Observation/action: the Isaac Lab standard locomotion contract (not the
    legacy 36-d MJX contract). To keep the deployed policy runnable on the real
    robot / Jetson NX, the setup is an ASYMMETRIC actor-critic: the actor
    observes only real-robot-measurable quantities (IMU angular velocity,
    projected gravity, velocity command, joint pos/vel, last action), while the
    critic additionally observes privileged base linear velocity.
  * Deployment runtime: the trained policy is exported to ONNX and built into a
    TensorRT engine for Jetson NX inference (the legacy RTNeural C++ path is not
    reused under this approach).

Importing this module registers the Gym tasks (see ``tasks``).
"""

from __future__ import annotations

__version__ = "0.1.0"
