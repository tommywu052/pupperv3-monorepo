#!/usr/bin/env python3
"""Pupper v3 — onnxruntime locomotion inference node (runs ON the Raspberry Pi).

Drop-in alternative to the C++ RTNeural ``neural_controller``. Instead of the
legacy 36-d * 20-history MJX contract, this runs the **Isaac Lab standard
locomotion policy** exported from ``ai/isaac_lab`` (approach B):

    observation (45) = [ base_ang_vel(3),
                         projected_gravity(3),
                         velocity_commands(3),     # /cmd_vel  vx, vy, wz
                         joint_pos_rel(12),        # q - default, policy order
                         joint_vel_rel(12),        # qd        , policy order
                         last_action(12) ]         # previous RAW network output
    action (12)      = raw network output
    joint target     = default_joint_pos + ACTION_SCALE * action      (clipped)

The robot's force-mode PD then tracks ``joint target`` with (KP, KD) — the SAME
gains used during Isaac Lab training, set here via the ``forward_kp/kd`` command
controllers.

ROS interface (all already provided by ``robot.service`` / ros2_control):
    sub  /joint_states                  sensor_msgs/JointState   (names SCRAMBLED -> remapped)
    sub  /imu_sensor_broadcaster/imu    sensor_msgs/Imu          (body-frame ang vel + orientation)
    sub  /cmd_vel                       geometry_msgs/Twist      (velocity command)
    pub  /forward_position_controller/commands  std_msgs/Float64MultiArray  (engage only)
    pub  /forward_kp_controller/commands         std_msgs/Float64MultiArray  (engage only)
    pub  /forward_kd_controller/commands         std_msgs/Float64MultiArray  (engage only)

SAFETY — two modes:
    --dry-run (DEFAULT): read sensors + run the policy + LOG everything, but
        NEVER publish motor commands. Zero risk. Use this first to confirm the
        observation assembly, joint remap, IMU and 50 Hz loop are all correct.
    --engage: actually drive the robot. Switches ros2_control from
        ``neural_controller`` to the three ``forward_*`` controllers, sets
        (KP, KD), runs an init ramp (current pose -> default stance), then a
        fade-in (default stance -> policy), with a projected-gravity tip-over
        e-stop. ONLY run with the robot on a stand / ready to catch.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState, Imu, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Empty

# --------------------------------------------------------------------------
# Deployment contract — identical to ai/isaac_lab/pupper_isaaclab/assets/pupper.py
# and neural_controller/launch/config.yaml.
# --------------------------------------------------------------------------
JOINT_NAMES = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]
DEFAULT_JOINT_POS = np.array(
    [0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52],
    dtype=np.float64,
)
LOWER = np.array([-1.22, -0.42, -2.79, -2.51, -3.14, -0.71,
                  -1.22, -0.42, -2.79, -2.51, -3.14, -0.71], dtype=np.float64)
UPPER = np.array([2.51, 3.14, 0.71, 1.22, 0.42, 2.79,
                  2.51, 3.14, 0.71, 1.22, 0.42, 2.79], dtype=np.float64)

KP = 5.0           # Isaac Lab ImplicitActuator stiffness used in training
KD = 0.25          # ... damping
ACTION_SCALE = 0.25
GRAVITY_VEC = np.array([0.0, 0.0, -1.0], dtype=np.float64)

# --------------------------------------------------------------------------
# Joint-order remap (CRITICAL — verified with scripts/check_contract.py).
# ``JOINT_NAMES`` above is the HARDWARE order (== forward_position_controller /
# neural_controller config, grouped per leg: FR_1,FR_2,FR_3, FL_..., BR_..., BL...).
# Isaac Lab, however, orders the articulation DOFs by kinematic-tree LEVEL
# (all four hips ``_1``, then all thighs ``_2``, then all calves ``_3``; within a
# level alphabetically: back_l, back_r, front_l, front_r). The exported policy
# therefore reads its joint_pos/joint_vel observation slices AND emits its 12
# action outputs in *POLICY* order — NOT hardware order.
#
# => Build the observation and interpret the action in POLICY order, but keep
#    reading /joint_states and publishing motor commands in HARDWARE order.
#    (Skipping this remap makes the robot stand & "step" but slide/spin instead
#    of tracking the command — the gait is silently permuted.)
# --------------------------------------------------------------------------
POLICY_JOINT_NAMES = [
    "leg_back_l_1", "leg_back_r_1", "leg_front_l_1", "leg_front_r_1",
    "leg_back_l_2", "leg_back_r_2", "leg_front_l_2", "leg_front_r_2",
    "leg_back_l_3", "leg_back_r_3", "leg_front_l_3", "leg_front_r_3",
]
# a_pol = a_hw[PERM_HW2POL] ;  a_hw = a_pol[INV_POL2HW]
PERM_HW2POL = np.array([JOINT_NAMES.index(n) for n in POLICY_JOINT_NAMES])
INV_POL2HW = np.array([POLICY_JOINT_NAMES.index(n) for n in JOINT_NAMES])
DEFAULT_JOINT_POS_POL = DEFAULT_JOINT_POS[PERM_HW2POL]
LOWER_POL = LOWER[PERM_HW2POL]
UPPER_POL = UPPER[PERM_HW2POL]

WALKING_CONTROLLERS_OFF = ["neural_controller", "neural_controller_three_legged"]
FORWARD_CONTROLLERS_ON = [
    "forward_position_controller",
    "forward_kp_controller",
    "forward_kd_controller",
]


def quat_rotate_inverse(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Isaac Lab math_utils.quat_rotate_inverse (q = w,x,y,z)."""
    w = q_wxyz[0]
    qvec = q_wxyz[1:]
    a = v * (2.0 * w * w - 1.0)
    b = np.cross(qvec, v) * (2.0 * w)
    c = qvec * (np.dot(qvec, v) * 2.0)
    return a - b + c


class TeleopKeys:
    """Non-blocking stdin key reader (POSIX cbreak).

    No-op (``enabled=False``) when stdin is not a TTY — so the node still runs
    fine when launched over a non-interactive SSH ``exec`` channel; you only get
    live keys from a real interactive SSH session (``ssh pi@...`` then run it).
    """

    def __init__(self):
        self.enabled = False
        self.fd = None
        self.old = None
        try:
            import sys
            import termios
            import tty

            if sys.stdin.isatty():
                self.fd = sys.stdin.fileno()
                self.old = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
                self.enabled = True
        except Exception:
            self.enabled = False

    def poll(self):
        if not self.enabled:
            return []
        import select
        import sys

        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            keys.append(sys.stdin.read(1))
        return keys

    def restore(self):
        if self.enabled and self.old is not None:
            import termios

            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.enabled = False


class PupperOnnxNode(Node):
    def __init__(self, args):
        super().__init__("pupper_onnx_node")
        self.args = args
        self.engage = args.engage
        self.scale = args.action_scale
        self.kp = args.kp
        self.kd = args.kd
        self.dt = 1.0 / args.rate

        # --- ONNX policy (fixed batch dim 1) ---
        self.sess = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
        self.in_name = self.sess.get_inputs()[0].name
        self.out_name = self.sess.get_outputs()[0].name
        in_shape = self.sess.get_inputs()[0].shape
        self.get_logger().info(
            f"loaded {args.model}  in='{self.in_name}'{in_shape} "
            f"out='{self.out_name}'{self.sess.get_outputs()[0].shape}  "
            f"mode={'ENGAGE' if self.engage else 'DRY-RUN'}"
        )

        # --- state ---
        self.q = None          # latest joint pos in POLICY order
        self.qd = None         # latest joint vel in POLICY order
        self.q_time = 0.0
        self.ang_vel = np.zeros(3)
        self.proj_grav = np.array([0.0, 0.0, -1.0])
        self.imu_time = 0.0
        self.cmd = np.zeros(3)  # vx, vy, wz (from /cmd_vel)
        self.walk_target = np.array([args.walk_vx, args.walk_vy, args.walk_wz])
        self.walk_enabled = bool(np.any(self.walk_target != 0.0))
        # --- teleop (keyboard) ---
        self.teleop = bool(getattr(args, "teleop", False))
        if self.teleop:
            self.walk_enabled = False  # built-in schedule off; drive from keys
        self.teleop_vmax = float(getattr(args, "teleop_vmax", 0.6))
        self.teleop_wmax = float(getattr(args, "teleop_wmax", 1.5))
        self.key_reader = None  # set by main() when --teleop
        # --- joystick (/joy) ---
        self.joy = bool(getattr(args, "joy", False))
        if self.joy:
            self.walk_enabled = False  # built-in schedule off; drive from joy axes
        self._prev_buttons = []        # for rising-edge detection
        self.last_action = np.zeros(12, dtype=np.float64)
        self.name_to_idx = None  # built lazily from first JointState

        # active = currently engaged/standing. Without --joy it follows --engage
        # (immediate). With --joy it stays False until the engage button (Square)
        # is pressed, so the robot waits on the current controller until you ask.
        self.active = self.engage and not self.joy
        self._switched = False

        # --- run / phase bookkeeping ---
        self.start_t = time.monotonic()
        self.n_infer = 0
        self.win_n = 0          # inferences since last report (windowed Hz)
        self.win_dact = 0.0     # sum of per-step |Δaction| (gait activity)
        self.last_report = self.start_t
        self.report_period = 1.0
        self.estopped = False        # tip-over: stiff catch-hold at default
        self.soft_estopped = False   # /emergency_stop (R3): limp like RTNeural
        self.estop_kd = float(getattr(args, "estop_kd", 0.1))

        # --- QoS: IMU broadcaster uses SensorData (best effort) ---
        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self._on_imu, imu_qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        if self.joy:
            self.create_subscription(Joy, "/joy", self._on_joy, 10)

        if self.engage:
            self.pub_pos = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
            self.pub_kp = self.create_publisher(Float64MultiArray, "/forward_kp_controller/commands", 10)
            self.pub_kd = self.create_publisher(Float64MultiArray, "/forward_kd_controller/commands", 10)
            # In joy mode, defer the controller switch until the engage button is
            # pressed (so we don't steal control from neural_controller on launch).
            if args.switch and not self.joy:
                self._switch_controllers()
                self._switched = True
            # estop_controller (R3) deactivates our forward_* controllers; mirror
            # RTNeural's on_deactivate soften so the legs go limp instead of stiff.
            self.create_subscription(Empty, "/emergency_stop", self._on_estop, 10)
            self.engage_t = None  # set when active

        self.timer = self.create_timer(self.dt, self._step)

    # ---------------- subscriptions ----------------
    def _on_js(self, msg: JointState):
        if self.name_to_idx is None:
            self.name_to_idx = [msg.name.index(j) for j in JOINT_NAMES]
            self.get_logger().info(f"joint remap (msg->policy) = {self.name_to_idx}")
        idx = self.name_to_idx
        pos = msg.position
        vel = msg.velocity
        self.q = np.array([pos[i] for i in idx], dtype=np.float64)
        self.qd = (np.array([vel[i] for i in idx], dtype=np.float64)
                   if len(vel) >= 12 else np.zeros(12))
        self.q_time = time.monotonic()

    def _on_imu(self, msg: Imu):
        self.ang_vel = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        q = np.array([msg.orientation.w, msg.orientation.x,
                      msg.orientation.y, msg.orientation.z])
        n = np.linalg.norm(q)
        if n > 1e-6:
            q = q / n
            self.proj_grav = quat_rotate_inverse(q, GRAVITY_VEC)
        self.imu_time = time.monotonic()

    def _on_cmd(self, msg: Twist):
        if self.joy:
            return  # in joy mode the command comes from /joy axes, not /cmd_vel
        self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    # ---------------- joystick ----------------
    @staticmethod
    def _axis(axes, i, scale, deadzone):
        if i < 0 or i >= len(axes):
            return 0.0
        v = float(axes[i])
        return 0.0 if abs(v) < deadzone else v * scale

    def _rising(self, buttons, idx):
        """True on the press edge of button ``idx`` (handles autorepeat /joy)."""
        if idx < 0 or idx >= len(buttons):
            return False
        was = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
        return buttons[idx] == 1 and was == 0

    def _on_joy(self, msg: Joy):
        a = self.args
        btn = list(msg.buttons)

        # engage button (Square): start standing + drive on the rising edge.
        # Always re-switch so we recover after an R3 e-stop deactivated our
        # forward_* controllers (and reset both e-stop states).
        if self._rising(btn, a.joy_engage_button) and not self.active and self.engage:
            self.active = True
            self.estopped = False
            self.soft_estopped = False
            self.engage_t = None  # restart init-ramp + fade from "now"
            if a.switch:
                self._switch_controllers()
                self._switched = True
            self.get_logger().info(
                f"[JOY] ENGAGE (button {a.joy_engage_button}) -> stand + drive")

        # stop button (optional): hold default stance, wait for re-engage
        if a.joy_stop_button >= 0 and self._rising(btn, a.joy_stop_button) and self.active:
            self.get_logger().info(f"[JOY] STOP (button {a.joy_stop_button}) -> hold default")
            self.active = False
            self.cmd = np.zeros(3)
            self._safe_shutdown()

        # discover unknown buttons (helps find the right index live)
        if a.joy_log_buttons:
            for i, b in enumerate(btn):
                was = self._prev_buttons[i] if i < len(self._prev_buttons) else 0
                if b == 1 and was == 0:
                    self.get_logger().info(f"[JOY] button {i} pressed")

        self._prev_buttons = btn

        # axes -> velocity command (only while engaged)
        if self.active:
            self.cmd = np.array([
                self._axis(msg.axes, a.joy_axis_vx, a.joy_vx_scale, a.joy_deadzone),
                self._axis(msg.axes, a.joy_axis_vy, a.joy_vy_scale, a.joy_deadzone),
                self._axis(msg.axes, a.joy_axis_wz, a.joy_wz_scale, a.joy_deadzone),
            ])

    def _on_estop(self, _msg: Empty):
        """R3 / estop_controller: go limp like RTNeural.

        estop_controller deactivates our forward_* controllers right after this
        fires. We burst low gains (kp=0, kd=estop_kd) first so the LAST command
        the hardware latches is soft -> the legs go limp instead of staying stiff.
        Re-press the engage button (Square) to re-activate and stand back up.
        """
        if not self.active and not self.engage:
            return
        self.get_logger().warn("[ESTOP] /emergency_stop -> limp (kp=0, kd=%.2f); press Square to recover"
                               % self.estop_kd)
        self.active = False
        self.soft_estopped = True
        self.cmd = np.zeros(3)
        hold = list(self.q) if self.q is not None else list(DEFAULT_JOINT_POS)
        for _ in range(40):  # ~0.4 s, must land before the deactivate switch
            self.pub_kp.publish(Float64MultiArray(data=[0.0] * 12))
            self.pub_kd.publish(Float64MultiArray(data=[self.estop_kd] * 12))
            self.pub_pos.publish(Float64MultiArray(data=hold))
            time.sleep(0.01)

    # ---------------- teleop ----------------
    def _handle_keys(self):
        """Map WASD/QE keys to the velocity command (setpoint style, persists)."""
        if not self.key_reader:
            return
        changed = False
        for k in self.key_reader.poll():
            kl = k.lower()
            if kl == "w":
                self.cmd[0] = min(self.cmd[0] + 0.1, self.teleop_vmax); changed = True
            elif kl == "s":
                self.cmd[0] = max(self.cmd[0] - 0.1, -self.teleop_vmax); changed = True
            elif kl == "a":
                self.cmd[2] = min(self.cmd[2] + 0.25, self.teleop_wmax); changed = True
            elif kl == "d":
                self.cmd[2] = max(self.cmd[2] - 0.25, -self.teleop_wmax); changed = True
            elif kl == "q":
                self.cmd[1] = min(self.cmd[1] + 0.1, self.teleop_vmax); changed = True
            elif kl == "e":
                self.cmd[1] = max(self.cmd[1] - 0.1, -self.teleop_vmax); changed = True
            elif k == " ":
                self.cmd[:] = 0.0; changed = True
            elif kl in ("z", "\x03"):  # z or Ctrl-C -> quit
                self.get_logger().info("teleop quit -> safe shutdown")
                self._safe_shutdown()
                self.timer.cancel()
                rclpy.shutdown()
                return
        if changed:
            self.get_logger().info(
                f"[TELEOP] cmd vx={self.cmd[0]:+.2f} vy={self.cmd[1]:+.2f} wz={self.cmd[2]:+.2f}  "
                f"(w/s fwd  a/d turn  q/e strafe  space stop  z quit)"
            )

    # ---------------- main loop ----------------
    def _step(self):
        now = time.monotonic()
        self._handle_keys()
        if not rclpy.ok():
            return
        if self.args.duration and (now - self.start_t) > self.args.duration:
            self.get_logger().info("duration reached, returning to default stance + shutting down.")
            self._safe_shutdown()
            self.timer.cancel()
            rclpy.shutdown()
            return
        if self.q is None or self.q_time == 0.0:
            return  # wait for first sensor data

        if self.active and self.engage_t is None:
            self.engage_t = now
        cmd_used = self._effective_cmd(now)

        # sensors arrive in HARDWARE order -> reorder to POLICY order for the net
        q_pol = self.q[PERM_HW2POL]
        qd_pol = self.qd[PERM_HW2POL]
        obs = np.concatenate([
            self.ang_vel,
            self.proj_grav,
            cmd_used,
            q_pol - DEFAULT_JOINT_POS_POL,
            qd_pol,
            self.last_action,            # stored in POLICY order
        ]).astype(np.float32).reshape(1, -1)

        action = self.sess.run([self.out_name], {self.in_name: obs})[0].reshape(-1).astype(np.float64)
        self.win_dact += float(np.abs(action - self.last_action).mean())  # gait activity
        self.last_action = action        # POLICY order
        self.n_infer += 1
        self.win_n += 1

        # action is POLICY order -> compute target there, then scatter to HW order
        target_pol = DEFAULT_JOINT_POS_POL + self.scale * action
        target_pol = np.clip(target_pol, LOWER_POL, UPPER_POL)
        target = target_pol[INV_POL2HW]  # HARDWARE order for forward_position_controller

        # tip-over detection: gravity should point ~ -Z in body frame when level.
        tilt_ok = self.proj_grav[2] < -0.5
        if not tilt_ok and self.active and not self.estopped:
            self.estopped = True
            self.get_logger().warn(
                f"TIP-OVER e-stop (proj_grav={self.proj_grav.round(2)}) -> holding default pose, kd low")

        if self.active:
            self._drive(now, target)

        self._maybe_report(now, cmd_used, action, target)

    def _effective_cmd(self, now):
        """Velocity command for the observation.

        With a built-in walk schedule (--walk-*), the command is held at 0 while
        the robot stands up (init + fade) and for ``walk_delay`` s afterwards,
        then ramped to the target over ``walk_ramp`` s. Otherwise the latest
        /cmd_vel is used.
        """
        if not (self.active and self.walk_enabled):
            return self.cmd
        el = now - (self.engage_t or now)
        t0 = self.args.init_duration + self.args.fade_duration + self.args.walk_delay
        if el < t0:
            return np.zeros(3)
        a = min((el - t0) / max(self.args.walk_ramp, 1e-3), 1.0)
        return a * self.walk_target

    def _drive(self, now, target):
        if self.engage_t is None:
            self.engage_t = now
        el = now - self.engage_t
        self.pub_kp.publish(Float64MultiArray(data=[self.kp] * 12))

        if self.estopped:
            self.pub_kd.publish(Float64MultiArray(data=[max(self.kd, 0.5)] * 12))
            self.pub_pos.publish(Float64MultiArray(data=list(DEFAULT_JOINT_POS)))
            return

        self.pub_kd.publish(Float64MultiArray(data=[self.kd] * 12))
        if el < self.args.init_duration:
            # ramp current measured pose -> default stance (policy ignored)
            a = el / max(self.args.init_duration, 1e-3)
            cmd = (1 - a) * self.q + a * DEFAULT_JOINT_POS
        elif el < self.args.init_duration + self.args.fade_duration:
            # fade default stance -> policy target
            a = (el - self.args.init_duration) / max(self.args.fade_duration, 1e-3)
            cmd = (1 - a) * DEFAULT_JOINT_POS + a * target
        else:
            cmd = target
        cmd = np.clip(cmd, LOWER, UPPER)
        self.pub_pos.publish(Float64MultiArray(data=list(cmd)))

    def _maybe_report(self, now, cmd_used, action, target):
        if now - self.last_report < self.report_period:
            return
        hz = self.win_n / (now - self.last_report)  # windowed (true loop rate)
        gait = self.win_dact / max(self.win_n, 1)   # mean per-step |Δaction|
        self.win_n = 0
        self.win_dact = 0.0
        js_age = now - self.q_time
        imu_age = now - self.imu_time if self.imu_time else -1
        tag = "ENG" if self.active else ("STBY" if self.engage else "DRY")
        self.get_logger().info(
            f"[{tag}] hz={hz:5.1f} "
            f"js_age={js_age*1e3:4.0f}ms imu_age={imu_age*1e3:4.0f}ms "
            f"cmd={np.asarray(cmd_used).round(2)} | "
            f"angvel|={np.linalg.norm(self.ang_vel):.2f} "
            f"projg={self.proj_grav.round(2)} gait={gait:.3f} | "
            f"act[{action.min():+.2f},{action.max():+.2f}] "
            f"tgt[{target.min():+.2f},{target.max():+.2f}]"
        )
        self.last_report = now

    def _safe_shutdown(self):
        """Leave the robot holding the default stance (engage only)."""
        if not self.engage:
            return
        for _ in range(20):  # ~0.2 s of default-pose hold
            self.pub_kp.publish(Float64MultiArray(data=[self.kp] * 12))
            self.pub_kd.publish(Float64MultiArray(data=[self.kd] * 12))
            self.pub_pos.publish(Float64MultiArray(data=list(DEFAULT_JOINT_POS)))
            time.sleep(0.01)
        self.get_logger().info("holding default stance under forward_position_controller.")

    # ---------------- controller switching ----------------
    def _switch_controllers(self):
        from controller_manager_msgs.srv import SwitchController
        cli = self.create_client(SwitchController, "/controller_manager/switch_controller")
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("switch_controller service unavailable; skipping switch")
            return
        req = SwitchController.Request()
        req.activate_controllers = FORWARD_CONTROLLERS_ON
        req.deactivate_controllers = WALKING_CONTROLLERS_OFF
        req.strictness = 1  # BEST_EFFORT
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        ok = fut.done() and fut.result() and fut.result().ok
        self.get_logger().info(f"controller switch -> forward_*: {'OK' if ok else 'FAILED'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="/home/pi/pupper_policy/policy.onnx")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--action-scale", type=float, default=ACTION_SCALE)
    ap.add_argument("--kp", type=float, default=KP, help="PD stiffness (training value 5.0; real RTNeural uses 7.5)")
    ap.add_argument("--kd", type=float, default=KD, help="PD damping")
    ap.add_argument("--estop-kd", type=float, default=0.1,
                    help="damping when /emergency_stop (R3) fires -> limp like RTNeural")
    ap.add_argument("--duration", type=float, default=0.0, help="auto-exit after N s (0 = run forever)")
    ap.add_argument("--engage", action="store_true", help="actually drive motors (default: dry-run)")
    ap.add_argument("--switch", action="store_true", help="switch ros2_control to forward_* controllers on start")
    ap.add_argument("--init-duration", type=float, default=1.5)
    ap.add_argument("--fade-duration", type=float, default=1.5)
    # built-in walk schedule (overrides /cmd_vel when any is nonzero)
    ap.add_argument("--walk-vx", type=float, default=0.0, help="forward vel command [m/s]")
    ap.add_argument("--walk-vy", type=float, default=0.0, help="lateral vel command [m/s]")
    ap.add_argument("--walk-wz", type=float, default=0.0, help="yaw rate command [rad/s]")
    ap.add_argument("--walk-delay", type=float, default=1.5, help="stand-still hold after fade before walking [s]")
    ap.add_argument("--walk-ramp", type=float, default=1.5, help="ramp 0->target command [s]")
    # keyboard teleop (run from an INTERACTIVE ssh session so stdin is a TTY)
    ap.add_argument("--teleop", action="store_true",
                    help="WASD keyboard drive: w/s fwd, a/d turn, q/e strafe, space stop, z quit")
    ap.add_argument("--teleop-vmax", type=float, default=0.6, help="max |vx|,|vy| command [m/s]")
    ap.add_argument("--teleop-wmax", type=float, default=1.5, help="max |wz| command [rad/s]")
    # joystick (/joy): press the engage button (Square) to stand, then drive with the sticks.
    # button/axis indices follow the PS-layout joy_linux mapping used by config.yaml.
    ap.add_argument("--joy", action="store_true",
                    help="joystick drive: Square=engage/stand, sticks=move (subscribes /joy)")
    ap.add_argument("--joy-engage-button", type=int, default=3,
                    help="button index to stand+drive (PS Square; X=0,O=1 are taken by estop_controller)")
    ap.add_argument("--joy-stop-button", type=int, default=-1,
                    help="button index to stop+hold default (-1 = disabled)")
    ap.add_argument("--joy-log-buttons", action="store_true",
                    help="log every button press index (use to find Square's index)")
    ap.add_argument("--joy-axis-vx", type=int, default=1, help="axis for forward vel (teleop default: 1)")
    ap.add_argument("--joy-axis-vy", type=int, default=0, help="axis for lateral vel (teleop default: 0)")
    ap.add_argument("--joy-axis-wz", type=int, default=3, help="axis for yaw rate (teleop default: 3)")
    ap.add_argument("--joy-vx-scale", type=float, default=0.75, help="full-stick forward vel [m/s]")
    ap.add_argument("--joy-vy-scale", type=float, default=0.5, help="full-stick lateral vel [m/s]")
    ap.add_argument("--joy-wz-scale", type=float, default=2.0, help="full-stick yaw rate [rad/s]")
    ap.add_argument("--joy-deadzone", type=float, default=0.08, help="stick deadzone (0-1)")
    args = ap.parse_args()

    rclpy.init()
    node = PupperOnnxNode(args)
    if args.joy:
        node.get_logger().info(
            f"JOY ready — press button {args.joy_engage_button} (Square) to stand + drive; "
            f"sticks: vx=axis{args.joy_axis_vx} vy=axis{args.joy_axis_vy} wz=axis{args.joy_axis_wz}. "
            f"(robot waits on the current controller until you press Square; "
            f"add --joy-log-buttons if Square isn't button {args.joy_engage_button})"
        )
    if args.teleop:
        node.key_reader = TeleopKeys()
        if node.key_reader.enabled:
            node.get_logger().info(
                "TELEOP ready — keys: w/s forward  a/d turn  q/e strafe  space stop  z quit. "
                "(tip: tap w a few times to clear the ~0.4 m/s step-into-gait threshold)"
            )
        else:
            node.get_logger().warn(
                "TELEOP requested but stdin is NOT a TTY — keys disabled. "
                "Run from an interactive 'ssh pi@...' session, not a piped/exec command."
            )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node._safe_shutdown()
        except Exception:
            pass
    finally:
        if node.key_reader is not None:
            node.key_reader.restore()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
