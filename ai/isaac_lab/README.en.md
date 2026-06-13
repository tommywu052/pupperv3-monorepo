# Pupper v3 — Isaac Lab RL Port + sim2real

> Language: [繁體中文](README.md) | **English**

Porting the Pupper v3 walking policy from **MJX (MuJoCo XLA) + Brax PPO** to
**NVIDIA Isaac Lab (PhysX GPU sim + rsl_rl PPO)**, all the way to the real robot
(Raspberry Pi, onnxruntime inference) under live teleop.

- **Training**: native Windows Isaac Lab v2.3.2 + Isaac Sim 5.1, RTX Pro 6000 (Blackwell, 96GB).
- **Observation/action contract (Approach B)**: standard Isaac Lab locomotion contract. To stay
  deployable on the real robot / Jetson, we use an **asymmetric actor-critic**: the actor only uses
  quantities measurable on hardware (IMU angular velocity, projected gravity, commands, joint
  pos/vel, last action); the critic may additionally use privileged quantities (base linear
  velocity, and a terrain height map on rough terrain).
- **Deployment (Approach 2)**: training artifact → **ONNX**, run in real time on the Raspberry Pi
  (onnxruntime CPU — 50 Hz with plenty of headroom) as a drop-in replacement for the C++
  `neural_controller` (RTNeural); TensorRT is only needed on Jetson NX.

The final robot **walks naturally in all directions and is keyboard/joystick teleoperable**. The
network architecture was never changed — the final `policy.onnx` walks unmodified once the
deployment-side ordering was fixed.

---

## 1. Contract (must match across training / export / deployment)

Standard Isaac Lab locomotion contract (**Approach B**), single-frame, 45-dim observation:

```
obs(45) = base_ang_vel(3)
        + projected_gravity(3)
        + velocity_commands(3)          # vx, vy, wz
        + (joint_pos - default)(12)     # policy joint order
        + joint_vel(12)                 # policy joint order
        + last_action(12)               # previous raw network output
action(12) = raw network output
target = default_joint_pos + 0.25 * action      (clipped to joint limits)
PD gains : KP = 5.0,  KD = 0.25      (force-mode PD, matches MuJoCo)
control  : 50 Hz       physics : 200 Hz
projected_gravity = quat_rotate_inverse(imu_orientation, [0,0,-1])
```

Single source of truth for the numbers (default pose / limits / gains):
`pupper_isaaclab/assets/pupper.py`. **Only the actor is exported**; the critic's privileged
observations do not affect deployment.

---

## 2. Training recipe: bring MJX's sim2real recipe into Isaac Lab

Physics alignment alone (effort/velocity limits, KP/KD) is not enough — early versions walked well
in sim but on hardware only "leaned without stepping", froze at low speed, or shook at high speed.
The missing pieces are MJX's domain randomizations for crossing the sim2real gap; all are required:

| Mechanism | Setting | Purpose |
|-----------|---------|---------|
| **ImplicitActuator** | PhysX PD at the 200 Hz physics rate | Mirrors the robot's ~520 Hz internal PD. **Do not** use `DelayedPDActuator` (PD at only 50 Hz rings and blows up `dof_acc`) |
| **1-step action latency** | `DelayedJointPositionAction`, `delay_prob=0.8` | Models actuation delay; the key to making the gait *alive* on hardware |
| **actuator gain randomization** | `randomize_actuator_gains`, stiffness ×[0.6,1.1], damping ×[0.8,1.5], `mode="startup"` | Mirrors MJX `kp_mult/kd_mult`; makes the gait *robust*, no underdamped tip-over |
| Others | friction, base mass/COM jitter, kick events | Standard locomotion DR |

> Lesson: delay-only → it steps but tips over (underdamped); gain-rand-only → it freezes.
> **Delay + gain-rand together** is what makes it stable. See
> `tasks/locomotion/delayed_action.py` and `tasks/locomotion/pupper_env_cfg.py`.

**Posture / anti-jitter rewards** (rough variant): the base reward set has no height/posture term,
so with the soft KP the policy learns to crouch and walk belly-to-ground. We added
`base_height_l2` (absolute target 0.14 m), `joint_deviation_hip`, `stand_still` (pin to the default
pose under near-zero command), and raised the `action_rate` penalty and the standing-env ratio.
> ⚠️ `base_height_l2` is deliberately **absolute** (no terrain-relative `sensor_cfg`): a
> terrain-relative target is unbounded — when a height-scan ray misses the terrain it spikes and
> drives PPO's action-noise std negative (`RuntimeError: normal expects std >= 0.0`), crashing training.

**Tasks**: `Pupper-Flat-v0` (flat), `Pupper-Rough-v0` (rough/stairs/slopes; the critic additionally
sees a terrain height map). Flat takes ~600 iters; rough takes a few thousand depending on the curriculum.

---

## 3. Export

`scripts/play.py` (adapted from Isaac Lab rsl_rl play) loads a checkpoint and exports with
`export_policy_as_onnx(policy_nn, normalizer=...)`. **The observation normalizer (per-dimension
running mean/std) is baked into the ONNX**, so the deployment side feeds the **raw** observation
(do not normalize yourself). Output: `exported/policy.onnx` (+ `policy.pt`).

> Because the normalizer is per-element, **every observation dimension's order must be exactly
> correct**, otherwise each value is applied to the wrong statistic → see the joint-order pitfall in §5.

---

## 4. Real-robot deployment (Raspberry Pi, onnxruntime)

`deploy/pupper_onnx_node.py` runs inference directly on the Pi (aarch64 / ROS 2 jazzy) with
onnxruntime + rclpy, **without touching any C++**, reusing the ros2_control interfaces already
provided by `robot.service`:

| Dir | Topic | Purpose |
|-----|-------|---------|
| sub | `/joint_states` | joint pos/vel (names are unordered; the node re-orders by name to **hardware order**, then to policy order) |
| sub | `/imu_sensor_broadcaster/imu` | body-frame angular velocity + orientation (for projected gravity) |
| sub | `/cmd_vel` or `/joy` | velocity command |
| pub | `/forward_position_controller/commands` | joint position targets |
| pub | `/forward_kp_controller/commands` / `/forward_kd_controller/commands` | kp=5.0 / kd=0.25 |

The `forward_*` controllers are inactive by default; on engage the node calls
`/controller_manager/switch_controller` to swap `neural_controller` for the three forward controllers.

**Upload + run** (on the Pi, first `source /opt/ros/jazzy/setup.bash` + the ros2_ws overlay,
`export ROS_LOCALHOST_ONLY=1`):

```bash
# upload from the dev machine (scp)
scp <exported>/policy.onnx        pi@<PI_IP>:/home/pi/pupper_policy/policy.onnx
scp deploy/pupper_onnx_node.py    pi@<PI_IP>:/home/pi/pupper_policy/pupper_onnx_node.py

# on the Pi: dry-run (read + infer only, no motor commands — safe)
python3 ~/pupper_policy/pupper_onnx_node.py --duration 12
# engage (drives motors — keep the robot elevated / clear)
python3 ~/pupper_policy/pupper_onnx_node.py --engage --switch
# joystick teleop (Square = stand up + drive, sticks move it)
python3 ~/pupper_policy/pupper_onnx_node.py --engage --switch --joy --joy-engage-button 3
```

**Safety**: `--switch` (swap to `forward_*`), init ramp → fade-in (smoothly return to the default
stance, then blend in the policy), tip-over e-stop (`projected_gravity.z > -0.5` locks the default
pose and raises kd), return to stance on exit. For autostart on boot use
`deploy/pupper_policy.service` + `deploy/pupper_policy.sh` (systemd).

**Joystick & e-stop**: PS layout `X=0`, `O=1` are taken by `estop_controller`, so **Square (=3)**
engages this policy. **R3 (button 12) = e-stop**: the node subscribes to `/emergency_stop` and,
*before* the controllers are deactivated, writes `kp=0, kd=estop_kd(0.1)` so the legs **go limp**
(same as RTNeural's `on_deactivate`) instead of locking stiff; **press Square again after an e-stop**
to stand back up. Axis mapping (same as `teleop_twist_joy`): left stick up/down = `vx` (×0.75),
left/right = `vy` (×0.5), right stick left/right = `wz` (×2.0).

---

## 5. Root-cause pitfall: Isaac Lab joint-order mismatch (the biggest sim2real culprit)

**Symptom**: walks great in sim (play); on hardware it stands and steps, but "forward" becomes
sliding/spinning in place, yaw barely responds, and it shakes at high speed.

**Root cause**: Isaac Lab and the hardware order the 12 joints differently —
- **Hardware / MJCF / `neural_controller` config**: grouped **by leg**, `FR_1,FR_2,FR_3, FL..., BR..., BL...`
- **Isaac Lab (PhysX DOF order after URDF→USD)**: grouped **by kinematic level**, `_1×4 → _2×4 → _3×4`
  (within a level: `back_l, back_r, front_l, front_r`)

In play the policy uses Isaac Lab order for both obs and action (self-consistent, so it works), but
the deployment node originally assumed hardware order == policy order → the obs `joint_pos`/`joint_vel`
and the action were **silently shuffled**. Since joints in the same level have similar ranges, it
still stands and steps, but each leg's motion goes to a different leg → directions are scrambled.

**Fix (deployment side only — no retrain, no re-export)**: read `/joint_states` and send motor
commands in **hardware order**, but build the obs and interpret the action in **policy order**:

```
q_pol      = q_hw[PERM]                       # to policy order before feeding the net
target_pol = default_pol + 0.25 * action      # action is in policy order
target_hw  = target_pol[INV]                  # back to hardware order before forward_position_controller
PERM = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]   # hardware → policy
INV  = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]   # policy → hardware
```

Verification tool: `scripts/check_contract.py` (loads the real articulation, prints
`robot.joint_names`, the action/obs layout, and auto-computes the mapping).

**Why MJX never hit this**: MuJoCo keeps the MJCF declaration order throughout, so training
obs/action and the deployment config share the same "by-leg" order and are aligned by construction.
Isaac Lab switched simulators and PhysX reordered the DOFs while the contract was not updated.
> Lesson: **switching simulators = the joint-order contract must be re-verified**. Never assume the
> order — always check `robot.joint_names`.

---

## 6. Directory layout

```
ai/isaac_lab/
├── pupper_isaaclab/
│   ├── assets/pupper.py            # ArticulationCfg + deployment contract (pose/limits/PD gains/USD)
│   └── tasks/locomotion/
│       ├── pupper_env_cfg.py       # env cfg (action latency + gain rand + posture rewards + terrain)
│       ├── delayed_action.py       # DelayedJointPositionAction (sim2real key)
│       ├── rewards.py              # custom rewards (foot_clearance, etc.)
│       └── agents/                 # rsl_rl PPO cfg
├── scripts/
│   ├── convert_pupper_urdf.py      # URDF (world link stripped) → USD
│   ├── train.py / play.py          # train / play+export ONNX
│   ├── check_contract.py           # print Isaac Lab's real joint order vs deployment (run when debugging)
│   └── verify_onnx.py              # verify exported ONNX I/O
├── deploy/                         # Raspberry Pi onnxruntime inference node + systemd
│   ├── pupper_onnx_node.py
│   ├── pupper_policy.service / pupper_policy.sh
│   └── diag_cmd_response.py
└── pyproject.toml
```

---

## 7. End-to-end reproduction

```powershell
cd C:\Nvidia\IsaacLab\IsaacLab
# (1) URDF → USD
.\isaaclab.bat -p ...\ai\isaac_lab\scripts\convert_pupper_urdf.py --headless
# (2) install this package
.\isaaclab.bat -p -m pip install -e C:\Nvidia\pupperv3\pupperv3-monorepo\ai\isaac_lab
# (3) train (flat ~600 iters; rough via Pupper-Rough-v0)
.\isaaclab.bat -p ...\ai\isaac_lab\scripts\train.py --task Pupper-Flat-v0 --headless
# (4) play + export ONNX (normalizer baked into ONNX)
.\isaaclab.bat -p ...\ai\isaac_lab\scripts\play.py --task Pupper-Flat-Play-v0 --num_envs 32
# (5) verify the joint/obs contract (always run after switching sim or editing the URDF)
.\isaaclab.bat -p ...\ai\isaac_lab\scripts\check_contract.py --task Pupper-Flat-Play-v0 --headless
```

```bash
# (6) upload to the Pi and run (see §4)
scp <exported>/policy.onnx pi@<PI_IP>:/home/pi/pupper_policy/policy.onnx
python3 ~/pupper_policy/pupper_onnx_node.py --duration 12                                   # dry-run
python3 ~/pupper_policy/pupper_onnx_node.py --engage --switch --joy --joy-engage-button 3   # joystick (Square engages)
```

---

## 8. Checklist / lessons

- [ ] **Joint order**: after switching sim / editing the URDF, align with `check_contract.py` first (easiest and most silent trap).
- [ ] **Observation order & normalizer**: the ONNX has a per-element normalizer; every obs dimension's order must be right.
- [ ] **sim2real DR**: 1-step action latency + actuator gain randomization are both required.
- [ ] **Actuator model**: for high-PD-rate robots use ImplicitActuator + action delay, not a 50 Hz DelayedPDActuator.
- [ ] **Reward numerics**: avoid unbounded rewards (e.g. terrain-relative `base_height_l2`), or the std goes negative and training crashes.
- [ ] **IMU frame**: treat the working RTNeural as ground truth (`ang_vel` used directly, `projected_gravity = R(q)^T·[0,0,-1]`).
- [ ] **Safety**: always elevate/clear the robot, dry-run first, keep the tip-over e-stop and return-to-stance on exit.
- [ ] **Soft e-stop**: with `forward_command_controller` a standard deactivation does not go limp; subscribe to `/emergency_stop`, write `kp=0/kd=estop_kd` before deactivation, and let the engage key re-`switch` to recover.
