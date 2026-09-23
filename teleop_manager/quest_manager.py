# Quest manager thread server
#
# Streams Meta Quest 3-point VR poses (head + both wrists) and BrainCo finger
# commands to the WBC deploy policy over ZMQ, replacing the Pico manager for
# Quest-based teleoperation.

"""
Data flow:

    Quest (Unity) --TCP--> ros_tcp_endpoint --ROS1--> quest_relay (Docker)
        --ZMQ SUB "quest_data" (msgpack, port 5559)--> THIS SCRIPT
        --ZMQ PUB "command"/"planner"/"manager_state" (port 5556)--> deploy policy

Input (quest_relay snapshot, ROS FLU frame: X-forward, Y-left, Z-up;
quaternions scalar-first [w, x, y, z]):
    head_pos[3], head_quat[4], left/right_wrist_pos[3], left/right_wrist_quat[4],
    left/right_landmarks[21][3] (MANO-21), left/right_tracked, timestamp

Output ("planner" topic, robot ROOT frame, X-forward, Y-left, Z-up):
    vr_position[9]     rows = [L-wrist, R-wrist, head], KEY-FRAME points, i.e.
                       the local offsets ([0.18, -/+0.025, 0] for the wrists,
                       [0, 0, 0.35] above the torso for the head) are ALREADY
                       applied, rotated by the live commanded orientation —
                       the deploy uses the sent values directly
                       (see g1_deploy_onnx_ref.cpp GatherVR3PointPosition).
    vr_orientation[12] scalar-first quaternions for the same three rows.
    left/right_hand_joints[7]  BrainCo: 6 normalized motors [0=open, 1=closed]
                       + one 0.0 padding slot.
    mode + movement[3] locomotion: the operator's head planar velocity (in the
                       fixed calibration frame, same frame as facing) drives a
                       SLOW_WALK/WALK command so the robot walks when the
                       operator walks. Disable with --disable-walk (robot then
                       only turns in place via facing). --static-base goes
                       further: no walk AND facing pinned to neutral, so the
                       base stays put and only the arms/hands move.
    mode + height      crouch: the operator's head DROP below the calibration
                       height (plus the '-'/'=' keyboard trim) commands an
                       IDEL_SQUAT with a target base height. Independent of
                       --static-base / --disable-walk. Squat is a static planner
                       mode, so while crouched the robot cannot walk (it still
                       turns via facing); crouch therefore wins over walking.
                       The head channel is OFF unless --enable-crouch is passed;
                       the '-'/'=' trim works either way.

Usage:
    # Live Quest (relay container must be running, see run_quest_relay.py)
    python quest_manager_thread_server.py

    # Turn in place only, no base translation
    python quest_manager_thread_server.py --disable-walk

    # Static base: only arms/hands move, head/torso fixed at reset pose
    python quest_manager_thread_server.py --static-base

    # Replay a recorded trajectory (record_quest_data.py NPZ format)
    python quest_manager_thread_server.py --replay data/quest/traj_xxx.npz

Keyboard:
    s          two-stage start (live Quest): 1st press starts the policy and ramps
               the robot from its current pose up to the calibration pose (the FK
               reference the operator will mirror); 2nd press runs a countdown,
               calibrates, and enters live VR_3PT teleop. (Replay: single press.)
    r          recalibrate: pauses teleop and eases the robot back to the
               reference pose, then runs the countdown and recalibrates (uses
               measured robot joints as FK ref). (Replay: immediate, no ramp.)
    f          toggle finger retargeting
    p          pause / resume teleop (freeze robot, smooth resume)
    c          toggle data collection        x   toggle data abort
    - / =      crouch deeper / stand back up (trim added to the head-height crouch)
    b          reset the sim box to its spawn pose (between episodes)
    0          FULL sim reset (recovery; robot snaps to default pose — pause first).
               A 'reanchor' command goes out right behind it, re-initializing the
               deploy's planner and heading anchor on the post-reset pose, so the
               policy holds the spawn heading instead of steering back to the
               pre-reset one.
    q          stop policy and exit
    k          (replay) pause / resume playback
    <- / ->    (replay) step one frame back / forward
"""

from __future__ import annotations

import argparse
import queue
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import msgpack
import numpy as np
import zmq
from scipy.spatial.transform import Rotation as sRot

from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
from teleop_manager.vis.vr3pt_pose_visualizer import (
    G1_KEY_FRAME_OFFSETS,
    get_g1_key_frame_poses,
)
from teleop_manager.zmq_io.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
    pack_pose_message,
)
from teleop_manager.zmq_io.zmq_poller import ZMQPoller
from retargeting.hand.interface import FingerRetargeting


# StreamMode values understood by the deploy state machine (see
# pico_manager_thread_server.StreamMode / mock_quest_streamer.py).
STREAM_MODE_OFF = 0
STREAM_MODE_PLANNER_VR_3PT = 5

# Manager teleop phase (live Quest two-stage start):
#   OFF    — policy stopped, nothing streamed.
#   RAMP   — policy started; VR targets are ramped from the robot's current pose
#            to the calibration pose, then held there until the 2nd 's'.
#   TELEOP — calibrated; live VR_3PT targets streamed.
PHASE_OFF = 0
PHASE_RAMP = 1
PHASE_TELEOP = 2

# LocomotionMode values (see localmotion_kplanner.hpp). IDLE keeps the robot
# stationary (only turning via the facing command); SLOW_WALK / WALK translate
# the base along the planner movement_direction.
LOCOMOTION_IDLE = 0
LOCOMOTION_SLOW_WALK = 1
LOCOMOTION_WALK = 2
LOCOMOTION_SQUAT = 4  # IDEL_SQUAT: static pose, the deploy forces speed=-1 for it
_WALK_MODES = {"slow": LOCOMOTION_SLOW_WALK, "walk": LOCOMOTION_WALK}

# Base-height command semantics, mirroring the deploy's own mapping in
# ros2_input_handler.hpp (>=0.72 stand, [0.5, 0.72) squat, <0.5 kneel).
PLANNER_STAND_HEIGHT = 0.78874  # PlannerConfig::default_height
CROUCH_ENGAGE_HEIGHT = 0.72  # at or above this the deploy treats the command as standing
HEIGHT_DEFAULT = -1.0  # wire sentinel: "use the mode default"

# Wire format always carries 7 hand values; BrainCo uses the first 6
# (normalized [0=open, 1=closed]) and the 7th slot is padding.
WIRE_HAND_DOF = 7

_SIDES = ("left", "right")


# ---------------------------------------------------------------------------
# Small rotation helpers (quaternions scalar-first [w, x, y, z] throughout)
# ---------------------------------------------------------------------------


def _rot(quat_wxyz: np.ndarray) -> sRot:
    q = np.asarray(quat_wxyz, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        return sRot.identity()
    return sRot.from_quat(q / norm, scalar_first=True)


def _quat(rot: sRot) -> np.ndarray:
    return rot.as_quat(scalar_first=True)


def _yaw_of(quat_wxyz: np.ndarray) -> float:
    """Heading angle: projection of the rotated forward axis (+X) on the XY plane."""
    fwd = _rot(quat_wxyz).apply([1.0, 0.0, 0.0])
    return float(np.arctan2(fwd[1], fwd[0]))


def _rz(yaw: float) -> sRot:
    return sRot.from_euler("z", yaw)


def _wrap(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def _smoothstep(x: float) -> float:
    """Clamp x to [0, 1] and apply cubic smoothstep easing (0->0, 1->1, flat ends)."""
    x = max(0.0, min(1.0, float(x)))
    return x * x * (3.0 - 2.0 * x)


def _slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    """Shortest-path normalized lerp between two wxyz quaternions (fine for short ramps)."""
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    q = (1.0 - alpha) * q0 + alpha * q1
    return q / max(np.linalg.norm(q), 1e-9)


# ---------------------------------------------------------------------------
# Low-pass filter on the 3-point teleop command
# ---------------------------------------------------------------------------


class PoseLowPass:
    """Exponential moving average on the VR 3-point targets (pos + quat).

    Smooths the per-row wrist/head targets to take the edge off Quest tracking
    jitter, complementing the existing low-passes on the head locomotion
    velocity and in the finger retargeter.

    Frame-rate independent: the blend factor is derived from the measured loop
    dt and a time constant ``tau`` (alpha = dt / (tau + dt)), the same scheme
    used for the head velocity filter. ``tau <= 0`` disables filtering. The 3
    position rows are lerped; the 3 orientation rows are slerped (shortest
    path). State persists across calls; ``reset()`` drops it so the next frame
    re-seeds the filter (call it after calibration / pause to avoid a lagged
    jump from stale state).
    """

    def __init__(self, tau: float):
        self.tau = max(0.0, float(tau))
        self._pos: np.ndarray | None = None
        self._quat: np.ndarray | None = None

    def reset(self) -> None:
        self._pos = None
        self._quat = None

    def __call__(
        self, pos: np.ndarray, quat: np.ndarray, dt: float
    ) -> tuple[np.ndarray, np.ndarray]:
        if self.tau <= 0.0 or dt <= 0.0:
            return pos, quat
        if self._pos is None or self._quat is None:
            self._pos = pos.copy()
            self._quat = quat.copy()
            return pos.copy(), quat.copy()
        alpha = dt / (self.tau + dt)
        self._pos = (1.0 - alpha) * self._pos + alpha * pos
        self._quat = np.concatenate(
            [_slerp(self._quat[4 * i : 4 * i + 4], quat[4 * i : 4 * i + 4], alpha) for i in range(3)]
        )
        return self._pos.copy(), self._quat.copy()


# ---------------------------------------------------------------------------
# Robot FK reference (rest pose of the wrist links in the root frame)
# ---------------------------------------------------------------------------


class RobotRestReference:
    """FK of the G1 wrist links + torso, used as the calibration anchor."""

    def __init__(self):
        self._robot_model = instantiate_g1_robot_model(hand_type="brainco")
        self._hand_convention = {
            side: self._compute_hand_convention(side) for side in _SIDES
        }
        print("[QuestManager] G1 robot model loaded for FK calibration")

    def _compute_hand_convention(self, side: str) -> sRot:
        """Constant local rotation N from the wrist LINK frame to the PHYSICAL
        hand frame (X = fingers, Z = back of hand), from the brainco finger FK.

        The Quest wrist TF uses the physical convention (verified against the
        MANO landmarks in recorded data), but the G1 wrist link frame is rolled
        ~90 deg about the forearm axis relative to it (back of hand = link -/+Y,
        not link Z). Comparing the two raw frames without N commands every
        wrist orientation 90 deg off — palms-down operator showed up as the
        robot's palms-inward rest pose.
        """
        rm = self._robot_model
        rm.cache_forward_kinematics(rm.default_body_pose, auto_clip=False)
        # Left and right hand URDFs capitalize the proximal link names differently.
        suffix = "proximal_Link" if side == "left" else "proximal_link"
        mid = np.asarray(rm.frame_placement(f"{side}_middle_{suffix}").translation)
        idx = np.asarray(rm.frame_placement(f"{side}_index_{suffix}").translation)
        pky = np.asarray(rm.frame_placement(f"{side}_pinky_{suffix}").translation)
        wrist = rm.frame_placement(f"{side}_wrist_yaw_link")
        wrist_pos = np.asarray(wrist.translation)
        wrist_rot = sRot.from_matrix(np.asarray(wrist.rotation))

        fingers = mid - wrist_pos
        fingers /= np.linalg.norm(fingers)
        across = idx - pky  # index->pinky spans the knuckles, thumb side first
        across /= np.linalg.norm(across)
        back = np.cross(fingers, across) if side == "right" else np.cross(across, fingers)
        y = np.cross(back, fingers)
        y /= np.linalg.norm(y)
        back = np.cross(fingers, y)
        phys = sRot.from_matrix(np.stack([fingers, y, back], axis=1))
        return wrist_rot.inv() * phys

    def hand_convention(self, side: str) -> sRot:
        """Wrist link frame -> physical hand frame (constant in link coords)."""
        return self._hand_convention[side]

    def compute(self, body_q_29: np.ndarray | None = None) -> dict:
        """FK key-frame poses in the root frame.

        Args:
            body_q_29: measured 29-DOF body joints, or None for the model's
                       default (rest) pose.

        Returns dict:
            {"left": (link_pos, link_rot), "right": (...), "torso_pos": (3,)}
            Positions are the wrist LINK origins (no key-frame offset applied);
            the offset is re-applied at runtime with the live orientation.
        """
        q = None
        if body_q_29 is not None:
            q = self._robot_model.get_configuration_from_actuated_joints(
                body_actuated_joint_values=np.asarray(body_q_29, dtype=np.float64)[:29]
            )
        poses = get_g1_key_frame_poses(self._robot_model, q=q, apply_offset=False)
        return {
            "left": (
                np.asarray(poses["left_wrist"]["position"], dtype=np.float64),
                _rot(poses["left_wrist"]["orientation_wxyz"]),
            ),
            "right": (
                np.asarray(poses["right_wrist"]["position"], dtype=np.float64),
                _rot(poses["right_wrist"]["orientation_wxyz"]),
            ),
            "torso_pos": np.asarray(poses["torso"]["position"], dtype=np.float64),
        }


class RobotFeedback:
    """Reads measured robot joints from the deploy g1_debug ZMQ stream."""

    def __init__(self, host: str, port: int):
        self._poller = ZMQPoller(host=host, port=port, topic="g1_debug")

    def measured_body_q(self) -> np.ndarray | None:
        data = self._poller.get_data()
        if data is None:
            return None
        try:
            unpacked = msgpack.unpackb(data, raw=False)
        except Exception as e:
            print(f"[QuestManager] Failed to unpack g1_debug feedback: {e}")
            return None
        if "body_q_measured" not in unpacked:
            print("[QuestManager] body_q_measured missing from g1_debug feedback")
            return None
        return np.array(unpacked["body_q_measured"], dtype=np.float64)


# ---------------------------------------------------------------------------
# 3-point pose tracking + calibration
# ---------------------------------------------------------------------------


class QuestThreePointTracker:
    """Maps Quest head/wrist poses to robot-root-frame VR 3-point targets.

    Calibration (operator in rest pose, mirroring the robot's reference pose):
      - R0 = Rz(head yaw): the operator's heading frame at calibration. All
        runtime quantities are expressed in this fixed frame, which is taken
        to coincide with the robot root frame.
      - v_cal = R0^-1 (p_wrist - p_head): the head->wrist offset vector. Using
        the head-relative delta (rather than absolute wrist position) makes
        tracking invariant to the operator walking or leaning.
      - W_cal: operator wrist orientation. Captures the (unknown, fixed)
        Quest-hand-frame vs robot-wrist-frame convention offset.
      - (link_ref, R_ref): robot wrist LINK pose from FK — the robot pose that
        the operator's rest pose maps onto.

    Runtime, per wrist, with Rh(t) = Rz(head_yaw(t)) the CURRENT heading frame:
        v(t)    = Rh(t)^-1 (p_wrist(t) - p_head(t))
        link(t) = link_ref + pos_scale * (v(t) - v_cal)
        R(t)    = Rh(t)^-1 W(t) W_cal^-1 R0 * R_ref       (world-frame delta)
        sent(t) = link(t) + R(t) @ key_frame_offset

    Using the CURRENT heading frame (not the fixed R0) makes the targets
    body-relative: when the operator turns their whole body about Z, v(t) and
    R(t) are unchanged, so the arms do not chase the rotation. The rotation
    itself is returned as yaw_rel and must be sent as the planner ``facing``
    command, which makes the robot's whole body turn to follow the operator.

    The world-frame delta makes R(t) independent of the Quest hand-frame
    convention: any fixed offset C in W = W_physical * C cancels in
    W(t) W_cal^-1.

    Head row: identity orientation — the torso target stays aligned with the
    root, which itself rotates via the ``facing`` command; position via the
    fixed kinematic chain torso + [0, 0, 0.35].

    Locomotion: the operator's head also drives base translation. compute()
    returns the head's planar velocity expressed in the fixed R0 frame (the
    same frame as ``facing``), low-pass filtered and computed from sensor
    timestamps so it is independent of this loop's polling rate. The manager
    turns that velocity into a planner movement_direction + speed so the robot
    walks when the operator walks. (The head ROW of vr_position stays fixed;
    walking is a separate planner command, not a moving torso target.)

    Crouch: compute() also returns how far the head has sunk below its
    calibration height, which the manager maps to a planner height command. No
    compensation is needed on the targets here — vr_position is pelvis-relative
    downstream, and v(t) above is head-relative, so an operator who crouches
    with their hands moves the arms down with the pelvis, while one who crouches
    keeping their hands put gets a rising v(t) that holds the hands in place.
    """

    # Head-velocity low-pass time constant (s) and the staleness window after
    # which a frozen / dropped Quest stream decays the velocity back to zero.
    _VEL_TAU = 0.25
    _VEL_STALE_SEC = 0.3

    _WRIST_OFFSET = {
        "left": np.asarray(G1_KEY_FRAME_OFFSETS["left_wrist"], dtype=np.float64),
        "right": np.asarray(G1_KEY_FRAME_OFFSETS["right_wrist"], dtype=np.float64),
    }
    _HEAD_OFFSET = np.asarray(G1_KEY_FRAME_OFFSETS["torso"], dtype=np.float64)

    def __init__(
        self,
        robot_ref: RobotRestReference,
        pos_scale: float = 0.8,
        static_base: bool = False,
    ):
        self._robot_ref = robot_ref
        self.pos_scale = float(pos_scale)
        # When the base heading is pinned (--static-base), wrist targets must be
        # expressed in the FIXED calibration frame R0 rather than the live head
        # heading frame Rh(t). Otherwise turning only the head rotates Rh(t),
        # which rotates v and r_cmd and makes the hands drift even though the
        # operator's hands are still. With a live base the facing command turns
        # the robot to cancel that rotation; with static_base facing is pinned,
        # so the rotation must be removed here instead.
        self.static_base = bool(static_base)
        self._calibrated = False
        self._yaw_cal = 0.0
        self._r0 = sRot.identity()
        self._r0_inv = sRot.identity()
        self._v_cal: dict[str, np.ndarray] = {}
        self._w_cal_inv: dict[str, sRot] = {}
        self._link_ref: dict[str, np.ndarray] = {}
        self._rot_ref: dict[str, sRot] = {}
        self._torso_pos = np.zeros(3)
        self._head_z_cal = 0.0
        # Head planar velocity tracking (R0 frame), reset on each calibration.
        self._prev_head_pos: np.ndarray | None = None
        self._prev_ts: float | None = None
        self._head_vel = np.zeros(3)
        self._last_vel_wall = 0.0

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    def calibrate(self, frame: dict, body_q_29: np.ndarray | None = None) -> None:
        """Capture calibration from one Quest frame against the robot FK reference."""
        fk = self._robot_ref.compute(body_q_29)
        self._yaw_cal = _yaw_of(frame["head_quat"])
        self._r0 = _rz(self._yaw_cal)
        self._r0_inv = self._r0.inv()

        head_pos = np.asarray(frame["head_pos"], dtype=np.float64)
        self._head_z_cal = float(head_pos[2])
        for side in _SIDES:
            wrist_pos = np.asarray(frame[f"{side}_wrist_pos"], dtype=np.float64)
            self._v_cal[side] = self._r0_inv.apply(wrist_pos - head_pos)
            self._w_cal_inv[side] = _rot(frame[f"{side}_wrist_quat"]).inv()
            self._link_ref[side], self._rot_ref[side] = fk[side]
        self._torso_pos = fk["torso_pos"]
        self._prev_head_pos = None
        self._prev_ts = None
        self._head_vel = np.zeros(3)
        self._last_vel_wall = time.monotonic()
        self._calibrated = True

        src = "default rest pose" if body_q_29 is None else "measured robot joints"
        print(
            f"[QuestManager] Calibration captured (FK ref: {src}, head yaw "
            f"{np.degrees(self._yaw_cal):+.1f} deg)"
        )
        for side in _SIDES:
            v = self._v_cal[side]
            print(f"  {side}: head->wrist cal vector [{v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.3f}]")

    def target_from_fk(
        self, body_q_29: np.ndarray | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """VR 3-point target (pos (9,), quat (12,)) for a static robot pose.

        Built with the exact same formula compute() uses at calibration, so the
        target for ``body_q_29=None`` (the model default / calibration pose) is
        identical to the first live teleop target once the operator is
        calibrated in the rest pose — the ramp can hand off to teleop seamlessly.
        Needs no calibration; used to drive the pre-teleop ramp to pose.
        """
        fk = self._robot_ref.compute(body_q_29)
        pos = np.zeros((3, 3), dtype=np.float64)
        quat = np.zeros((3, 4), dtype=np.float64)
        for i, side in enumerate(_SIDES):
            link, rot = fk[side]
            pos[i] = link + rot.apply(self._WRIST_OFFSET[side])
            quat[i] = _quat(rot)
        pos[2] = fk["torso_pos"] + self._HEAD_OFFSET
        quat[2] = np.array([1.0, 0.0, 0.0, 0.0])
        return pos.flatten(), quat.flatten()

    def _update_head_velocity(self, head_pos: np.ndarray, ts: float) -> None:
        """Low-pass head planar velocity in the fixed R0 frame.

        Velocity is differenced over sensor timestamps (not loop time), so a
        repeated frame (sample-and-hold relay, paused replay) contributes no
        spurious motion; if no fresh frame arrives within ``_VEL_STALE_SEC``
        the estimate decays to zero so the robot does not keep walking.
        """
        now = time.monotonic()
        if self._prev_head_pos is None or self._prev_ts is None:
            self._prev_head_pos = head_pos
            self._prev_ts = ts
            self._last_vel_wall = now
            return
        dt = ts - self._prev_ts
        if dt < -1e-4:
            # Sensor time jumped backwards (replay looped or stepped back):
            # re-anchor on this frame instead of freezing on a stale prev
            # timestamp or differencing across the discontinuity.
            self._prev_head_pos = head_pos
            self._prev_ts = ts
            self._last_vel_wall = now
            return
        if dt > 1e-4:
            raw = self._r0_inv.apply(head_pos - self._prev_head_pos) / dt
            raw[2] = 0.0
            alpha = dt / (self._VEL_TAU + dt)
            self._head_vel = (1.0 - alpha) * self._head_vel + alpha * raw
            self._prev_head_pos = head_pos
            self._prev_ts = ts
            self._last_vel_wall = now
        elif now - self._last_vel_wall > self._VEL_STALE_SEC:
            self._head_vel = np.zeros(3)

    def compute(self, frame: dict) -> tuple[np.ndarray, np.ndarray, float, np.ndarray, float]:
        """One Quest frame -> (vr_position (9,), vr_orientation (12,), yaw_rel,
        head_vel (3,), head_drop).

        Positions/orientations are body-relative targets in the robot root
        frame; yaw_rel is the operator heading change since calibration and
        must be sent as the planner ``facing`` direction to turn the robot;
        head_vel is the operator's planar head velocity (m/s, z=0) in the fixed
        R0 frame, used to drive base locomotion; head_drop is how far the
        operator's head has sunk below its calibration height (m, positive
        down), used to drive the crouch.
        """
        if not self._calibrated:
            raise RuntimeError("compute() called before calibrate()")

        yaw_rel = _wrap(_yaw_of(frame["head_quat"]) - self._yaw_cal)
        # Frame the wrist targets are expressed in. Normally the live head
        # heading frame Rh(t) (body-relative targets; the facing command turns
        # the robot to follow). Under static_base the base heading is pinned, so
        # we use the fixed calibration frame R0 instead — head-only rotation then
        # leaves the hand targets unchanged. yaw_rel is still returned (harmless;
        # facing is pinned downstream).
        rh_inv = self._r0_inv if self.static_base else _rz(self._yaw_cal + yaw_rel).inv()

        pos = np.zeros((3, 3), dtype=np.float64)
        quat = np.zeros((3, 4), dtype=np.float64)
        head_pos = np.asarray(frame["head_pos"], dtype=np.float64)
        self._update_head_velocity(head_pos, float(frame["timestamp"]))

        for i, side in enumerate(_SIDES):
            wrist_pos = np.asarray(frame[f"{side}_wrist_pos"], dtype=np.float64)
            v = rh_inv.apply(wrist_pos - head_pos)
            link = self._link_ref[side] + self.pos_scale * (v - self._v_cal[side])

            w = _rot(frame[f"{side}_wrist_quat"])
            r_cmd = rh_inv * w * self._w_cal_inv[side] * self._r0 * self._rot_ref[side]

            pos[i] = link + r_cmd.apply(self._WRIST_OFFSET[side])
            quat[i] = _quat(r_cmd)

        # Head row: torso aligned with the root (rotation goes through facing)
        pos[2] = self._torso_pos + self._HEAD_OFFSET
        quat[2] = np.array([1.0, 0.0, 0.0, 0.0])

        head_drop = self._head_z_cal - float(head_pos[2])
        return pos.flatten(), quat.flatten(), yaw_rel, self._head_vel.copy(), head_drop



# ---------------------------------------------------------------------------
# Data sources
# ---------------------------------------------------------------------------


def _frame_from_snapshot(snap: dict) -> dict:
    """Normalize a relay snapshot into a dict of numpy arrays."""
    return {
        "head_pos": np.asarray(snap["head_pos"], dtype=np.float64),
        "head_quat": np.asarray(snap["head_quat"], dtype=np.float64),
        "left_wrist_pos": np.asarray(snap["left_wrist_pos"], dtype=np.float64),
        "left_wrist_quat": np.asarray(snap["left_wrist_quat"], dtype=np.float64),
        "right_wrist_pos": np.asarray(snap["right_wrist_pos"], dtype=np.float64),
        "right_wrist_quat": np.asarray(snap["right_wrist_quat"], dtype=np.float64),
        "left_landmarks": np.asarray(snap["left_landmarks"], dtype=np.float64),
        "right_landmarks": np.asarray(snap["right_landmarks"], dtype=np.float64),
        "left_tracked": bool(snap["left_tracked"]),
        "right_tracked": bool(snap["right_tracked"]),
        "timestamp": float(snap["timestamp"]),
    }


class LiveQuestSource:
    """Background ZMQ SUB thread holding the latest quest_data snapshot."""

    is_replay = False

    def __init__(self, host: str, port: int):
        self._ctx = zmq.Context()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(f"tcp://{host}:{port}")
        self._sub.setsockopt(zmq.SUBSCRIBE, b"quest_data")
        self._sub.setsockopt(zmq.RCVTIMEO, 100)
        self._lock = threading.Lock()
        self._latest: dict | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print(f"[QuestManager] Subscribed to quest relay at tcp://{host}:{port}")

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                parts = self._sub.recv_multipart()
            except zmq.Again:
                continue
            except zmq.ZMQError:
                break
            try:
                frame = _frame_from_snapshot(msgpack.unpackb(parts[1], raw=False))
            except Exception as e:
                print(f"[QuestManager] Bad quest_data message: {e}")
                continue
            # timestamp 0.0 means the relay has not received any Quest topic
            # yet — not usable for tracking or calibration.
            if frame["timestamp"] <= 0.0:
                continue
            with self._lock:
                self._latest = frame

    def get_frame(self) -> dict | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._sub.close()
        self._ctx.term()


class ReplaySource:
    """Replays a recorded NPZ trajectory with a synthetic rest-pose prefix.

    The prefix makes calibration on frame 0 always valid: the first frames put
    the (virtual) operator exactly in the pose the calibration expects — head
    at the recording's initial head pose, wrists where the robot's rest wrist
    links sit relative to the head — held for ``rest_hold_sec`` and then
    interpolated to the recording's real first frame over ``rest_interp_sec``.

    Playback is time-anchored (wall clock against recorded timestamps), loops
    at the end, and supports pause ('k') and single-frame stepping (arrows).
    """

    is_replay = True

    def __init__(
        self,
        path: str,
        robot_ref: RobotRestReference,
        rest_hold_sec: float = 1.0,
        rest_interp_sec: float = 1.0,
    ):
        raw = np.load(path, allow_pickle=True)
        n = raw["head_pos"].shape[0]
        if n < 2:
            raise ValueError(f"Recording {path} has fewer than 2 frames")
        ts = np.asarray(raw["timestamp"], dtype=np.float64)
        dt = float(np.median(np.diff(ts)))
        dt = dt if np.isfinite(dt) and dt > 1e-4 else 1.0 / 50.0

        frames = {
            k: np.asarray(raw[k])
            for k in (
                "head_pos",
                "head_quat",
                "left_wrist_pos",
                "left_wrist_quat",
                "right_wrist_pos",
                "right_wrist_quat",
                "left_landmarks",
                "right_landmarks",
                "left_tracked",
                "right_tracked",
            )
        }
        prefix = self._build_rest_prefix(frames, robot_ref, dt, rest_hold_sec, rest_interp_sec)
        self._data = {k: np.concatenate([prefix[k], frames[k]], axis=0) for k in frames}
        n_prefix = prefix["head_pos"].shape[0]
        self._ts = np.concatenate([np.arange(n_prefix) * dt, n_prefix * dt + (ts - ts[0])])
        self._n = self._ts.shape[0]
        self._duration = float(self._ts[-1]) + dt

        self._paused = False
        self._media_time = 0.0
        self._anchor = time.monotonic()
        print(
            f"[QuestManager] Replay {Path(path).name}: {n} frames + {n_prefix} rest-prefix "
            f"frames ({self._duration:.1f}s total, {1.0 / dt:.0f} fps, looping)"
        )

    @staticmethod
    def _build_rest_prefix(
        frames: dict,
        robot_ref: RobotRestReference,
        dt: float,
        hold_sec: float,
        interp_sec: float,
    ) -> dict:
        """Synthesize rest frames matching the robot FK rest pose, then blend to frame 0."""
        fk = robot_ref.compute(None)
        head_pos0 = np.asarray(frames["head_pos"][0], dtype=np.float64)
        head_quat0 = np.asarray(frames["head_quat"][0], dtype=np.float64)
        r0 = _rz(_yaw_of(head_quat0))
        head_ref = fk["torso_pos"] + QuestThreePointTracker._HEAD_OFFSET

        # Rest frame: virtual operator whose head matches the recording's first
        # head pose and whose wrists sit where the robot's rest wrist LINKS sit
        # relative to its own head reference, lifted into the Quest world by the
        # head heading. Calibrating on this frame maps it exactly onto FK rest.
        rest = {
            "head_pos": head_pos0,
            "head_quat": head_quat0,
            "left_landmarks": np.asarray(frames["left_landmarks"][0], dtype=np.float64),
            "right_landmarks": np.asarray(frames["right_landmarks"][0], dtype=np.float64),
        }
        for side in _SIDES:
            link_pos, link_rot = fk[side]
            rest[f"{side}_wrist_pos"] = head_pos0 + r0.apply(link_pos - head_ref)
            # The synthetic wrist quat must be what the QUEST would publish for
            # an operator physically holding the robot's rest pose: the link
            # orientation expressed in the physical hand convention (N), since
            # the Quest wrist TF is the physical frame, not the link frame.
            rest[f"{side}_wrist_quat"] = _quat(r0 * link_rot * robot_ref.hand_convention(side))

        n_hold = max(1, int(round(hold_sec / dt)))
        n_interp = max(0, int(round(interp_sec / dt)))
        n_total = n_hold + n_interp

        prefix: dict[str, np.ndarray] = {}
        for key in ("head_pos", "left_wrist_pos", "right_wrist_pos"):
            first = np.asarray(frames[key][0], dtype=np.float64)
            arr = np.tile(rest[key], (n_total, 1))
            for j in range(n_interp):
                a = (j + 1) / (n_interp + 1)
                arr[n_hold + j] = (1.0 - a) * rest[key] + a * first
            prefix[key] = arr
        for key in ("head_quat", "left_wrist_quat", "right_wrist_quat"):
            first = np.asarray(frames[key][0], dtype=np.float64)
            arr = np.tile(rest[key], (n_total, 1))
            for j in range(n_interp):
                a = (j + 1) / (n_interp + 1)
                arr[n_hold + j] = _slerp(rest[key], first, a)
            prefix[key] = arr
        for key in ("left_landmarks", "right_landmarks"):
            prefix[key] = np.tile(rest[key][None], (n_total, 1, 1))
        # No finger retargeting during the synthetic prefix.
        prefix["left_tracked"] = np.zeros(n_total, dtype=bool)
        prefix["right_tracked"] = np.zeros(n_total, dtype=bool)
        return prefix

    # -- playback controls --------------------------------------------------

    def _current_media_time(self) -> float:
        if self._paused:
            return self._media_time
        return (time.monotonic() - self._anchor) % self._duration

    def toggle_pause(self) -> None:
        if self._paused:
            self._anchor = time.monotonic() - self._media_time
            self._paused = False
            print("[QuestManager] Replay resumed")
        else:
            self._media_time = self._current_media_time()
            self._paused = True
            print(f"[QuestManager] Replay paused at t={self._media_time:.2f}s")

    def step(self, delta: int) -> None:
        """Step playback by delta frames; pauses the clock."""
        if not self._paused:
            self._media_time = self._current_media_time()
            self._paused = True
        idx = (self._current_index() + delta) % self._n
        self._media_time = float(self._ts[idx])
        print(f"[QuestManager] Replay step -> frame {idx}/{self._n - 1}")

    def restart(self) -> None:
        self._media_time = 0.0
        self._anchor = time.monotonic()

    def _current_index(self) -> int:
        idx = int(np.searchsorted(self._ts, self._current_media_time(), side="right")) - 1
        return int(np.clip(idx, 0, self._n - 1))

    def get_frame(self) -> dict:
        i = self._current_index()
        return {
            "head_pos": self._data["head_pos"][i],
            "head_quat": self._data["head_quat"][i],
            "left_wrist_pos": self._data["left_wrist_pos"][i],
            "left_wrist_quat": self._data["left_wrist_quat"][i],
            "right_wrist_pos": self._data["right_wrist_pos"][i],
            "right_wrist_quat": self._data["right_wrist_quat"][i],
            "left_landmarks": self._data["left_landmarks"][i],
            "right_landmarks": self._data["right_landmarks"][i],
            "left_tracked": bool(self._data["left_tracked"][i]),
            "right_tracked": bool(self._data["right_tracked"][i]),
            "timestamp": float(self._ts[i]),
        }

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Keyboard
# ---------------------------------------------------------------------------


class KeyboardListener:
    """cbreak single-key reader; arrow keys are decoded to 'LEFT' / 'RIGHT'."""

    def __init__(self) -> None:
        self.keys: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._old_term = None
        self._thread: threading.Thread | None = None

    def __enter__(self) -> "KeyboardListener":
        if self._fd is not None:
            self._old_term = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        else:
            print("[QuestManager] stdin is not a TTY — keyboard controls disabled")
        return self

    def _loop(self) -> None:
        while not self._stop.is_set():
            if not select.select([sys.stdin], [], [], 0.2)[0]:
                continue
            ch = sys.stdin.read(1)
            if ch != "\x1b":
                if ch:
                    self.keys.put(ch)
                continue
            # Escape sequence (arrow keys: ESC [ C / ESC [ D)
            seq = ""
            while select.select([sys.stdin], [], [], 0.01)[0] and len(seq) < 2:
                seq += sys.stdin.read(1)
            if seq == "[C":
                self.keys.put("RIGHT")
            elif seq == "[D":
                self.keys.put("LEFT")

    def __exit__(self, *exc) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._fd is not None and self._old_term is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)


# ---------------------------------------------------------------------------
# Manager loop
# ---------------------------------------------------------------------------


class QuestManager:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.robot_ref = RobotRestReference()
        self.tracker = QuestThreePointTracker(
            self.robot_ref, pos_scale=args.pos_scale, static_base=args.static_base
        )
        self.pose_filter = PoseLowPass(args.smooth_tau)
        self.retargeter = FingerRetargeting(force_np=args.np_retarget)
        self.feedback = RobotFeedback(args.feedback_host, args.feedback_port)

        if args.replay:
            self.source = ReplaySource(
                args.replay,
                self.robot_ref,
                rest_hold_sec=args.rest_hold_sec,
                rest_interp_sec=args.rest_interp_sec,
            )
        else:
            self.source = LiveQuestSource(args.relay_host, args.relay_port)

        self.ctx = zmq.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind(f"tcp://*:{args.port}")
        time.sleep(0.3)  # let subscribers connect before the first messages
        print(f"[QuestManager] ZMQ PUB bound to port {args.port}")

        self.stream_mode = STREAM_MODE_OFF
        self.phase = PHASE_OFF
        # Pre-teleop ramp state (live Quest): captured lazily once the first
        # robot feedback arrives after START, since body_q_measured is only
        # published while the deploy is in CONTROL.
        self.ramp_started = False
        self._ramp_warn_t = 0.0
        self.ramp_start_t = 0.0
        self.ramp_from_pos: np.ndarray | None = None
        self.ramp_from_quat: np.ndarray | None = None
        self.ramp_to_pos: np.ndarray | None = None
        self.ramp_to_quat: np.ndarray | None = None
        # Recalibration re-uses the ramp to ease the robot back to the reference
        # pose (instead of following live VR); `recalibrating` marks that the ramp
        # should auto-arm the recalib countdown on completion, `_recalib_armed`
        # ensures it is armed exactly once.
        self.recalibrating = False
        self._recalib_armed = False
        self.finger_tracking = True
        self.teleop_paused = False
        # monotonic time of the previous filtered compute(), for the pose filter dt
        self._last_compute_t: float | None = None
        # Locomotion (head-position walking). `_walking` adds hysteresis around
        # the speed deadband so the robot does not flap between IDLE and walk.
        self.walk_mode = _WALK_MODES[args.walk_mode]
        self._walking = False
        # Crouch (head-height squatting). `crouch_trim` is the keyboard offset in
        # metres added on top of the head drop; `_crouching` adds hysteresis around
        # the engage deadband; `_height_cmd` is the last height actually sent
        # (quantized), kept for the status line.
        self.crouch_trim = 0.0
        self._crouching = False
        self._height_cmd = HEIGHT_DEFAULT
        self.resume_ramp_start: float | None = None
        # Countdown state: (deadline, kind) with kind in {"start", "recalib"}
        self.pending_calib: tuple[float, str] | None = None
        self.last_countdown_print = 0
        # Scene-reset commands for the sim, shipped as monotonic counters in
        # manager_state (level-triggered: the sim acts on any observed increase,
        # so a late-joining or briefly-stalled subscriber never misses one).
        self.reset_box_count = 0
        self.reset_sim_count = 0
        # Set by '0'; consumed right after the manager_state publish that carries the
        # reset, so the deploy re-anchors on the already-reset robot. See run().
        self._reanchor_pending = False

        # Last commanded targets; reused while paused / blended on resume.
        self.frozen_pos: np.ndarray | None = None
        self.frozen_quat: np.ndarray | None = None
        self.frozen_yaw: float = 0.0
        self.frozen_drop: float = 0.0
        self.frozen_hands: dict[str, list[float] | None] = {"left": None, "right": None}
        self.last_hands: dict[str, list[float] | None] = {"left": None, "right": None}
        # facing_yaw = yaw_rel - yaw_offset: the offset rebases the operator's
        # heading after pauses/recalibrations so the commanded facing never jumps.
        self.yaw_offset: float = 0.0
        self.last_facing_yaw: float = 0.0
        self.resume_rebase_pending = False

    # -- calibration triggers -------------------------------------------------

    def _arm_calibration(self, kind: str) -> None:
        if self.source.is_replay:
            # The synthetic rest prefix stands in for the operator's rest pose,
            # so no countdown is needed: restart and calibrate on frame 0.
            if kind == "start":
                self.source.restart()
            self._do_calibration(kind, self.source.get_frame())
            return
        delay = self.args.calib_delay_sec
        self.pending_calib = (time.monotonic() + delay, kind)
        self.last_countdown_print = int(np.ceil(delay)) + 1
        print(
            f"[QuestManager] Assume the rest pose — calibrating in {delay:.0f}s "
            f"({'start' if kind == 'start' else 'recalibration'})"
        )

    def _do_calibration(self, kind: str, frame: dict | None) -> None:
        if frame is None:
            print("[QuestManager] WARNING: no Quest data — calibration aborted")
            return
        body_q = None
        if kind == "recalib":
            body_q = self.feedback.measured_body_q()
            if body_q is None:
                print(
                    "[QuestManager] WARNING: no g1_debug feedback — "
                    "recalibrating against the default rest pose instead"
                )
        self.tracker.calibrate(frame, body_q_29=body_q)
        self.pose_filter.reset()
        self._last_compute_t = None
        # The crouch reference (head z) was just re-captured, so the trim stacked on
        # top of it is stale — drop it, and stand up. The ramp that precedes this
        # already streams height=-1.0, so this is the hand-off point, not a jump.
        self.crouch_trim = 0.0
        self._crouching = False
        self.frozen_drop = 0.0
        if kind == "start":
            self.yaw_offset = 0.0
            self.last_facing_yaw = 0.0
            # START is already sent when the ramp begins (live); re-sending is a
            # no-op once the deploy is in CONTROL, and it is required for replay
            # (which skips the ramp), so send it here too.
            self.pub.send(build_command_message(start=True, stop=False, planner=True))
            self.stream_mode = STREAM_MODE_PLANNER_VR_3PT
            self.phase = PHASE_TELEOP
            self.recalibrating = False
            self._recalib_armed = False
            print("[QuestManager] Calibrated — entering live VR_3PT teleop")
        else:
            # Recalibration zeroes yaw_rel; keep the commanded facing continuous
            # so the robot does not turn back to its start heading.
            self.yaw_offset = _wrap(-self.last_facing_yaw)
            # The recalib ramp left us in PHASE_RAMP holding the reference pose;
            # re-enter live teleop and clear the recalib-ramp latches. (For replay,
            # which skips the ramp, phase is already TELEOP and the flags are unset.)
            self.phase = PHASE_TELEOP
            self.recalibrating = False
            self._recalib_armed = False
            print("[QuestManager] Recalibrated — resuming live VR_3PT teleop")

    def _begin_calib_ramp(self) -> None:
        """1st 's' (live): start the policy and ramp the robot to the calibration
        pose instead of snapping to live teleop targets.

        Sends START (deploy WAIT_FOR_CONTROL -> CONTROL) and enters PHASE_RAMP.
        The ramp endpoints are captured lazily in _run_ramp() once the first
        robot feedback arrives, because body_q_measured is only published while
        the deploy is in CONTROL.
        """
        self.pub.send(build_command_message(start=True, stop=False, planner=True))
        self.stream_mode = STREAM_MODE_PLANNER_VR_3PT
        self.phase = PHASE_RAMP
        self.ramp_started = False
        self._ramp_warn_t = 0.0
        print(
            "[QuestManager] Policy START sent — ramping to the calibration pose "
            f"over {self.args.calib_ramp_sec:.1f}s. Press 's' again once the robot "
            "is settled to calibrate and begin teleop."
        )

    def _begin_recalib_ramp(self) -> None:
        """'r' during teleop: stop following the operator and ease the robot back
        to the reference (calibration) pose, then auto-run the countdown and
        recalibrate.

        Re-enters PHASE_RAMP (so live VR targets stop streaming and _run_ramp
        drives the robot to the reference pose instead). _run_ramp arms the
        recalib countdown once the robot has settled at the reference pose; when
        the countdown fires, _do_calibration('recalib') re-enters teleop. The
        policy stays in CONTROL throughout — no START/STOP is sent.
        """
        self.phase = PHASE_RAMP
        self.ramp_started = False
        self._ramp_warn_t = 0.0
        self.recalibrating = True
        self._recalib_armed = False
        self.pending_calib = None  # (re)armed by _run_ramp once settled at reference
        self.teleop_paused = False
        print(
            "[QuestManager] Recalibrating — easing the robot back to the reference "
            f"pose over {self.args.calib_ramp_sec:.1f}s (teleop paused), then a "
            f"{self.args.calib_delay_sec:.0f}s countdown. Assume the rest pose."
        )

    def _run_ramp(self) -> bool:
        """Stream VR targets that ease the robot from its current pose to the
        calibration pose (FK of the default config), then hold there. Returns
        True when a planner message was sent this call.

        Runs every loop while in PHASE_RAMP. The ramp endpoints are captured on
        the first robot feedback after START (body_q_measured is only published
        once the deploy is in CONTROL). If feedback never arrives we CANNOT ramp
        safely — the robot's current pose is unknown, so any target we stream
        would snap it. In that case we send nothing (the deploy keeps holding its
        pose) and warn periodically; the 2nd 's' is likewise blocked until
        feedback is seen, so teleop can never engage with a jump either.
        """
        now = time.monotonic()
        if not self.ramp_started:
            body_q = self.feedback.measured_body_q()
            if body_q is None:
                if now - self._ramp_warn_t >= 2.0:
                    self._ramp_warn_t = now
                    print(
                        "[QuestManager] Waiting for g1_debug robot feedback "
                        f"({self.args.feedback_host}:{self.args.feedback_port}) before "
                        "ramping — the robot holds its current pose. Check the deploy "
                        "is publishing g1_debug, or press 'q' to stop."
                    )
                return False  # never stream a target we can't ramp from
            self.ramp_from_pos, self.ramp_from_quat = self.tracker.target_from_fk(body_q)
            self.ramp_to_pos, self.ramp_to_quat = self.tracker.target_from_fk(None)
            self.ramp_start_t = now
            self.ramp_started = True

        alpha = _smoothstep((now - self.ramp_start_t) / max(self.args.calib_ramp_sec, 1e-3))
        pos = (1.0 - alpha) * self.ramp_from_pos + alpha * self.ramp_to_pos
        quat = np.concatenate(
            [
                _slerp(
                    self.ramp_from_quat[4 * i : 4 * i + 4],
                    self.ramp_to_quat[4 * i : 4 * i + 4],
                    alpha,
                )
                for i in range(3)
            ]
        )
        self.pub.send(
            build_planner_message(
                mode=LOCOMOTION_IDLE,
                movement=[0.0, 0.0, 0.0],
                facing=[1.0, 0.0, 0.0],
                speed=-1.0,
                height=HEIGHT_DEFAULT,  # always ramp standing; crouch is reset at calibration
                vr_3pt_position=pos.tolist(),
                vr_3pt_orientation=quat.tolist(),
            )
        )
        # Recalibration path: once the robot has eased all the way back to the
        # reference pose, hold it there and start the calibration countdown (armed
        # exactly once). The robot keeps streaming this held reference pose through
        # the countdown, so it no longer follows the operator while they re-settle.
        if self.recalibrating and not self._recalib_armed and alpha >= 1.0:
            self._recalib_armed = True
            self._arm_calibration("recalib")
        return True

    # -- keyboard handling ------------------------------------------------------

    def _handle_key(self, key: str) -> tuple[bool, bool, bool]:
        """Returns (quit, toggle_data_collection, toggle_data_abort)."""
        toggle_dc = toggle_da = False
        if key == "q":
            return True, False, False
        elif key == "s":
            if self.source.is_replay:
                # Replay has no robot to ramp; single press starts teleop.
                if self.phase == PHASE_OFF:
                    self._arm_calibration("start")
                else:
                    print("[QuestManager] Already started ('q' to stop)")
            elif self.phase == PHASE_OFF:
                self._begin_calib_ramp()
            elif self.phase == PHASE_RAMP:
                if self.ramp_started:
                    self._arm_calibration("start")
                else:
                    print("[QuestManager] Waiting for robot feedback before calibrating...")
            else:
                print("[QuestManager] Already started ('r' to recalibrate, 'q' to stop)")
        elif key == "r":
            if self.source.is_replay:
                # Replay has no operator to re-settle; recalibrate immediately.
                if self.phase == PHASE_TELEOP:
                    self._arm_calibration("recalib")
                else:
                    print("[QuestManager] Recalibrate is only available during teleop")
            elif self.phase == PHASE_TELEOP:
                self._begin_recalib_ramp()
            elif self.phase == PHASE_RAMP and self.recalibrating:
                print("[QuestManager] Already recalibrating — easing back to the reference pose")
            else:
                print("[QuestManager] Recalibrate is only available during teleop")
        elif key == "f":
            self.finger_tracking = not self.finger_tracking
            print(f"[QuestManager] Finger retargeting {'ON' if self.finger_tracking else 'OFF'}")
        elif key == "p":
            if self.phase != PHASE_TELEOP:
                print("[QuestManager] Pause is only available during teleop")
                return False, toggle_dc, toggle_da
            self.teleop_paused = not self.teleop_paused
            if self.teleop_paused:
                self.frozen_pos = None  # captured from the next computed targets
                self.resume_ramp_start = None
                self.pose_filter.reset()  # re-seed from the live pose on resume
                self._last_compute_t = None
                print("[QuestManager] Teleop PAUSED — robot frozen at last pose")
            else:
                self.resume_ramp_start = time.monotonic()
                self.resume_rebase_pending = True
                print(
                    f"[QuestManager] Teleop RESUMED — easing to live pose over "
                    f"{self.args.resume_ramp_sec:.1f}s"
                )
        elif key == "c":
            toggle_dc = True
            print("[QuestManager] Data collection toggle sent")
        elif key == "x":
            toggle_da = True
            print("[QuestManager] Data abort toggle sent")
        elif key in ("-", "_", "=", "+"):
            # Trim added to the head-drop crouch; also the only crouch source for
            # replay / crouch-disabled. Same '-' down / '=' up convention as the
            # deploy's own keyboard_handler.
            step = self.args.crouch_step if key in ("-", "_") else -self.args.crouch_step
            self.crouch_trim = max(0.0, self.crouch_trim + step)
            print(f"[QuestManager] Crouch trim {self.crouch_trim:+.2f}m")
        elif key == "b":
            self.reset_box_count += 1
            print("[QuestManager] Box reset sent (sim teleports the box back to spawn)")
        elif key == "0":
            self.reset_sim_count += 1
            self._reanchor_pending = True
            print(
                "[QuestManager] FULL SIM RESET sent — robot snaps to default pose and the "
                "policy heading is re-anchored on it (pause teleop with 'p' first for a "
                "clean recovery)"
            )
        elif key == "k" and self.source.is_replay:
            self.source.toggle_pause()
        elif key == "LEFT" and self.source.is_replay:
            self.source.step(-1)
        elif key == "RIGHT" and self.source.is_replay:
            self.source.step(+1)
        return False, toggle_dc, toggle_da

    # -- per-loop helpers ---------------------------------------------------------

    def _maybe_send_reanchor(self) -> None:
        """Send the post-reset re-anchor command, after the reset it belongs to.

        Call this right after the manager_state publish carrying the incremented
        reset counter: one PUB socket delivers in order, so the sim has the reset
        before the deploy has the re-anchor, and the deploy then re-initializes its
        planner on the already-snapped robot.
        """
        if not self._reanchor_pending:
            return
        self._reanchor_pending = False
        if self.phase == PHASE_OFF:
            return
        self.pub.send(
            build_command_message(start=False, stop=False, planner=True, reanchor=True)
        )
        print("[QuestManager] Re-anchor sent — policy heading realigned with the reset pose")

    def _update_countdown(self, frame: dict | None) -> None:
        if self.pending_calib is None:
            return
        deadline, kind = self.pending_calib
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            self.pending_calib = None
            self._do_calibration(kind, frame)
        elif int(np.ceil(remaining)) < self.last_countdown_print:
            self.last_countdown_print = int(np.ceil(remaining))
            print(f"[QuestManager] ... {self.last_countdown_print}")

    def _compute_hands(self, frame: dict) -> dict[str, list[float] | None]:
        if not self.finger_tracking:
            return {"left": None, "right": None}
        hands: dict[str, list[float] | None] = {}
        for side in _SIDES:
            if frame[f"{side}_tracked"]:
                try:
                    self.last_hands[side] = self.retargeter(frame[f"{side}_landmarks"], side)
                except Exception as e:
                    print(f"[QuestManager] {side} retargeting error: {e}")
            hands[side] = self.last_hands[side]  # hold last command while untracked
        return hands

    def _apply_pause_resume(
        self, pos: np.ndarray, quat: np.ndarray, yaw_rel: float, hands: dict, drop: float
    ) -> tuple[np.ndarray, np.ndarray, float, dict, float]:
        facing_yaw = _wrap(yaw_rel - self.yaw_offset)

        if self.teleop_paused:
            if self.frozen_pos is None:
                self.frozen_pos = pos.copy()
                self.frozen_quat = quat.copy()
                self.frozen_yaw = facing_yaw
                self.frozen_drop = drop
                self.frozen_hands = dict(hands)
            return (
                self.frozen_pos,
                self.frozen_quat,
                self.frozen_yaw,
                self.frozen_hands,
                self.frozen_drop,
            )

        if self.resume_ramp_start is not None and self.frozen_pos is not None:
            if self.resume_rebase_pending:
                # Rebase heading: the operator may have turned while paused;
                # continue from the frozen facing instead of jumping.
                self.yaw_offset = _wrap(yaw_rel - self.frozen_yaw)
                self.resume_rebase_pending = False
            facing_yaw = _wrap(yaw_rel - self.yaw_offset)
            alpha = (time.monotonic() - self.resume_ramp_start) / max(
                self.args.resume_ramp_sec, 1e-3
            )
            if alpha >= 1.0:
                self.resume_ramp_start = None
                self.frozen_pos = None
            else:
                pos = (1.0 - alpha) * self.frozen_pos + alpha * pos
                quat = np.concatenate(
                    [
                        _slerp(self.frozen_quat[4 * i : 4 * i + 4], quat[4 * i : 4 * i + 4], alpha)
                        for i in range(3)
                    ]
                )
                drop = (1.0 - alpha) * self.frozen_drop + alpha * drop
        return pos, quat, facing_yaw, hands, drop

    def _walk_command(self, head_vel: np.ndarray) -> tuple[int, list[float], float]:
        """Map operator head planar velocity (R0 frame) to a planner movement.

        Returns (locomotion_mode, movement_direction[3], speed). While walking
        is disabled or teleop is paused — and below the speed deadband — this
        is IDLE with a zero movement vector, i.e. exactly the previous
        stationary behavior (the robot still turns via the facing command).
        """
        if self.args.disable_walk or self.args.static_base or self.teleop_paused:
            self._walking = False
            return LOCOMOTION_IDLE, [0.0, 0.0, 0.0], -1.0

        speed = float(np.hypot(head_vel[0], head_vel[1]))
        # Hysteresis: start above the deadband, keep going until well below it.
        if self._walking:
            self._walking = speed >= 0.5 * self.args.walk_deadband
        else:
            self._walking = speed >= self.args.walk_deadband
        if not self._walking:
            return LOCOMOTION_IDLE, [0.0, 0.0, 0.0], -1.0

        direction = np.asarray(head_vel[:2], dtype=np.float64) / max(speed, 1e-6)
        cmd_speed = float(
            np.clip(
                speed * self.args.walk_speed_scale,
                self.args.walk_min_speed,
                self.args.walk_max_speed,
            )
        )
        return self.walk_mode, [float(direction[0]), float(direction[1]), 0.0], cmd_speed

    def _crouch_command(self, head_drop: float) -> float:
        """Operator head drop (m, positive down) -> planner base-height command.

        Returns the target height in metres, or HEIGHT_DEFAULT (-1.0) meaning
        "stand". The keyboard trim is added to the head drop, so '-'/'=' work on
        their own (replay, mock) and as an offset on top of live head tracking.

        The result is quantized: the deploy replans the whole planner rollout on
        ANY height change (g1_deploy_onnx_ref.cpp height_changed), so a raw 50 Hz
        stream would force a replan every tick.
        """
        drop = max(0.0, head_drop) * self.args.crouch_scale if self.args.enable_crouch else 0.0
        drop += self.crouch_trim

        # Hysteresis so the operator does not flap in and out of squat at the
        # engage boundary (each transition is a planner replan).
        deadband = self.args.crouch_deadband
        self._crouching = drop >= (0.5 * deadband if self._crouching else deadband)
        if not self._crouching:
            return HEIGHT_DEFAULT

        height = np.clip(
            PLANNER_STAND_HEIGHT - drop, self.args.crouch_min_height, CROUCH_ENGAGE_HEIGHT
        )
        quantum = max(self.args.crouch_quantum, 1e-3)
        # round() again to kill the float noise, so equal buckets compare equal
        # (the deploy detects "height changed" by comparison).
        return round(round(height / quantum) * quantum, 4)

    # -- main loop ------------------------------------------------------------------

    def run(self) -> None:
        period = 1.0 / max(1, self.args.target_fps)
        print(
            "[QuestManager] Controls: s=ramp-to-calib / s-again=calibrate+teleop  "
            "r=recalibrate  f=fingers  p=pause  c=collect  x=abort  "
            "-/= crouch-deeper/stand  b=box-reset  0=FULL-sim-reset  q=quit"
            + ("  k=replay-pause  arrows=step" if self.source.is_replay else "")
        )
        last_report = time.time()
        sent = 0
        lat_sum, lat_max, lat_n = 0.0, 0.0, 0  # frame-age stats for --log-latency
        with KeyboardListener() as kb:
            while True:
                t_start = time.monotonic()
                quit_requested = toggle_dc = toggle_da = False
                while not kb.keys.empty():
                    q, dc, da = self._handle_key(kb.keys.get())
                    quit_requested |= q
                    toggle_dc |= dc
                    toggle_da |= da
                if quit_requested:
                    break

                frame = self.source.get_frame()
                # End-to-end frame age at the manager input. Only meaningful for
                # a live relay (its timestamp is relay wall-clock time.time());
                # replay timestamps are media-relative, so skip them.
                if (
                    self.args.log_latency
                    and frame is not None
                    and not self.source.is_replay
                ):
                    ts = float(frame.get("timestamp", 0.0))
                    if ts > 0.0:
                        age = time.time() - ts
                        lat_sum += age
                        lat_max = max(lat_max, age)
                        lat_n += 1
                self._update_countdown(frame)

                if self.phase == PHASE_RAMP:
                    if self._run_ramp():
                        sent += 1
                elif self.phase == PHASE_TELEOP and frame is not None:
                    pos, quat, yaw_rel, head_vel, head_drop = self.tracker.compute(frame)
                    dt = 0.0 if self._last_compute_t is None else t_start - self._last_compute_t
                    self._last_compute_t = t_start
                    pos, quat = self.pose_filter(pos, quat, dt)
                    hands = self._compute_hands(frame)
                    pos, quat, facing_yaw, hands, head_drop = self._apply_pause_resume(
                        pos, quat, yaw_rel, hands, head_drop
                    )
                    self.last_facing_yaw = facing_yaw
                    # Crouch wins over walking: IDEL_SQUAT is a static planner mode
                    # (the deploy forces speed=-1 for it), so the two cannot coexist.
                    height = self._crouch_command(head_drop)
                    self._height_cmd = height
                    if height >= 0.0:
                        mode, movement, speed = LOCOMOTION_SQUAT, [0.0, 0.0, 0.0], -1.0
                        self._walking = False
                    else:
                        mode, movement, speed = self._walk_command(head_vel)
                    # --static-base pins the base heading: never turn the robot.
                    facing = (
                        [1.0, 0.0, 0.0]
                        if self.args.static_base
                        else [float(np.cos(facing_yaw)), float(np.sin(facing_yaw)), 0.0]
                    )
                    self.pub.send(
                        build_planner_message(
                            mode=mode,
                            movement=movement,
                            facing=facing,
                            speed=speed,
                            height=height,
                            left_hand_position=hands["left"],
                            right_hand_position=hands["right"],
                            vr_3pt_position=pos.tolist(),
                            vr_3pt_orientation=quat.tolist(),
                        )
                    )
                    sent += 1

                self.pub.send(
                    pack_pose_message(
                        {
                            "stream_mode": np.array([self.stream_mode], dtype=np.int32),
                            "toggle_data_collection": np.array([toggle_dc], dtype=bool),
                            "toggle_data_abort": np.array([toggle_da], dtype=bool),
                            "reset_box_count": np.array([self.reset_box_count], dtype=np.int32),
                            "reset_sim_count": np.array([self.reset_sim_count], dtype=np.int32),
                        },
                        topic="manager_state",
                    )
                )
                self._maybe_send_reanchor()

                now = time.time()
                if now - last_report >= 5.0:
                    if self.phase == PHASE_OFF:
                        state = "OFF"
                    elif self.phase == PHASE_RAMP:
                        state = "RAMP"
                    elif self.teleop_paused:
                        state = "PAUSED"
                    else:
                        state = "VR_3PT"
                    data = "ok" if frame is not None else "waiting"
                    walk = (
                        "static"
                        if self.args.static_base
                        else (
                            "off"
                            if self.args.disable_walk
                            else ("walk" if self._walking else "idle")
                        )
                    )
                    crouch = "off" if self._height_cmd < 0.0 else f"{self._height_cmd:.2f}"
                    latency = (
                        f" | frame-age avg={1e3 * lat_sum / lat_n:.0f}ms "
                        f"max={1e3 * lat_max:.0f}ms"
                        if lat_n > 0
                        else ""
                    )
                    print(
                        f"[QuestManager] {state} | quest={data} | "
                        f"{sent / (now - last_report):.1f} planner msg/s | "
                        f"fingers={'on' if self.finger_tracking else 'off'} | "
                        f"walk={walk} | crouch={crouch}{latency}"
                    )
                    last_report = now
                    lat_sum, lat_max, lat_n = 0.0, 0.0, 0
                    sent = 0

                elapsed = time.monotonic() - t_start
                if elapsed < period:
                    time.sleep(period - elapsed)

        print("[QuestManager] Sending policy STOP...")
        self.pub.send(build_command_message(start=False, stop=True, planner=True))
        time.sleep(0.1)

    def close(self) -> None:
        self.source.close()
        self.pub.close()
        self.ctx.term()
        print("[QuestManager] Shutdown complete")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Quest teleop manager: relay/NPZ -> WBC policy ZMQ stream.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", type=int, default=5556, help="ZMQ PUB port (deploy subscribes)")
    parser.add_argument("--relay-host", default="localhost", help="quest_relay host")
    parser.add_argument("--relay-port", type=int, default=5559, help="quest_relay ZMQ port")
    parser.add_argument("--feedback-host", default="localhost", help="g1_debug feedback host")
    parser.add_argument("--feedback-port", type=int, default=5557, help="g1_debug feedback port")
    parser.add_argument("--target-fps", type=int, default=50, help="planner stream rate")
    parser.add_argument(
        "--pos-scale",
        type=float,
        default=0.7,
        help="scale on operator arm reach (orientation unaffected); 1.0 = 1:1 with "
        "operator motion. Lower under-reaches and feels sluggish.",
    )
    parser.add_argument(
        "--smooth-tau",
        type=float,
        default=0.02,
        help="EMA time constant (s) for the 3-point wrist/head pos+quat targets; "
        "higher = smoother but laggier, 0 disables. Lowered from 0.05 now that the "
        "upstream ZMQ queueing is fixed (conflate + publish-on-receipt), so less "
        "filtering is needed to hide jitter.",
    )
    parser.add_argument(
        "--calib-delay-sec",
        type=float,
        default=3.0,
        help="countdown after 's'/'r' before the calibration frame is captured (live only)",
    )
    parser.add_argument(
        "--calib-ramp-sec",
        type=float,
        default=3.0,
        help="duration of the ramp from the robot's current pose to the calibration "
        "pose, used by the 1st-'s' start ramp and the 'r' recalib ramp (live only)",
    )
    parser.add_argument(
        "--resume-ramp-sec",
        type=float,
        default=1.0,
        help="ease-in duration from frozen to live pose when resuming after pause",
    )
    parser.add_argument(
        "--log-latency",
        action="store_true",
        help="log end-to-end frame age (relay timestamp -> manager input) in the "
        "periodic status line; live relay only (replay timestamps are media-relative)",
    )
    parser.add_argument(
        "--disable-walk",
        action="store_true",
        help="disable head-position walking; robot only turns in place (facing) "
        "and never translates, reproducing the pre-walk behavior",
    )
    parser.add_argument(
        "--static-base",
        action="store_true",
        help="freeze the robot base: only arms/hands move, the head/torso stays at "
        "its reset pose (no walking AND no turn-in-place). Implies --disable-walk "
        "and forces facing to neutral. Operator should keep head motion limited.",
    )
    parser.add_argument(
        "--walk-mode",
        choices=tuple(_WALK_MODES),
        default="slow",
        help="locomotion mode used while the operator walks",
    )
    parser.add_argument(
        "--walk-deadband",
        type=float,
        default=0.08,
        help="operator head speed (m/s) above which a walk command is sent",
    )
    parser.add_argument(
        "--walk-speed-scale",
        type=float,
        default=1.0,
        help="scale from operator head speed to commanded robot walk speed",
    )
    parser.add_argument(
        "--walk-min-speed",
        type=float,
        default=0.2,
        help="lower clamp on commanded walk speed (m/s) once walking",
    )
    parser.add_argument(
        "--walk-max-speed",
        type=float,
        default=0.8,
        help="upper clamp on commanded walk speed (m/s)",
    )
    parser.add_argument(
        "--enable-crouch",
        action="store_true",
        help="let the operator's head height drive the base-height crouch. Off by "
        "default: it has never been validated on hardware, and an operator who ducks "
        "or leans would make the robot squat (squat also blocks walking). The "
        "'-'/'=' crouch trim works either way.",
    )
    parser.add_argument(
        "--crouch-scale",
        type=float,
        default=1.0,
        help="commanded base-height drop per metre of operator head drop",
    )
    parser.add_argument(
        "--crouch-deadband",
        type=float,
        default=0.07,
        help="operator head drop (m) below the calibration height at which the crouch "
        "engages; the default lands engagement right at the deploy's 0.72m squat "
        "boundary. Disengages at half this (hysteresis)",
    )
    parser.add_argument(
        "--crouch-min-height",
        type=float,
        default=0.5,
        help="lower clamp on the commanded base height (m). The deploy switches from "
        "IDEL_SQUAT to IDEL_KNEEL below 0.5, so the default keeps squat",
    )
    parser.add_argument(
        "--crouch-quantum",
        type=float,
        default=0.02,
        help="quantization (m) of the commanded height; every distinct value forces a "
        "planner replan",
    )
    parser.add_argument(
        "--crouch-step",
        type=float,
        default=0.05,
        help="height change (m) per '-'/'=' key press",
    )
    parser.add_argument(
        "--np-retarget",
        action="store_true",
        help="force the pure-numpy finger retargeter",
    )
    parser.add_argument(
        "--replay", default=None, help="NPZ trajectory to replay instead of live Quest"
    )
    parser.add_argument(
        "--rest-hold-sec",
        type=float,
        default=1.0,
        help="(replay) duration the synthetic rest pose is held before interpolation",
    )
    parser.add_argument(
        "--rest-interp-sec",
        type=float,
        default=1.0,
        help="(replay) duration of the blend from rest pose to the recording's first frame",
    )
    args = parser.parse_args()

    manager = QuestManager(args)
    try:
        manager.run()
    except KeyboardInterrupt:
        print("\n[QuestManager] Interrupted — sending policy STOP...")
        manager.pub.send(build_command_message(start=False, stop=True, planner=True))
        time.sleep(0.1)
    finally:
        manager.close()


if __name__ == "__main__":
    main()
