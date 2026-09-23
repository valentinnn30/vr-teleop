"""Live MediaPipe -> BrainCo retargeting that drives the *real* BrainCo hands.

This is the real-hardware counterpart of
``third_party/brainco-retargeting/demos/live_camera.py``. The camera input,
MediaPipe hand tracking, and BrainCo retargeting are identical; the only
difference is that the resulting 6 normalized motor values are streamed to the
physical BrainCo Revo2 hand(s) over Unitree DDS, exactly the way the C++
deploy binary (``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref``) does.

How the hand link works (see ``deployment_report.md`` Part A and
``gear_sonic_deploy/.../include/brainco_hands.hpp``):

  * The robot runs ``brainco_hand_service`` (the serial<->DDS bridge). It owns
    the RS485/Modbus link and exposes each hand on DDS topics
    ``rt/brainco/{left,right}/{cmd,state}`` using ``unitree_go::MotorCmds_`` /
    ``MotorStates_`` sized to 6 motors.
  * Anything on the same DDS domain / ``192.168.123.x`` network can publish
    ``rt/brainco/{side}/cmd`` and the bridge drives the fingers. This script is
    that publisher (the same role the deploy binary's ``BraincoHands`` class
    plays), so ``g1_deploy_onnx_ref`` must NOT be running at the same time or
    the two will fight over the command topic.

Value convention (normalized, identical to the bridge / firmware):
    q  : 0.0 = fully open, 1.0 = fully closed
    dq : 0.0 = stopped,    1.0 = full speed (recommended default)
    6 motors, order [thumb_metacarpal, thumb_proximal, index, middle, ring,
    pinky] -- which is exactly the BrainCoRetargeter output order.

Run it on the laptop (cabled to the robot, laptop on 192.168.123.x -- see
``deployment_setup.md`` Part 1). It uses the laptop camera and talks DDS to the
``brainco_hand_service`` running on the robot.

Usage (in .venv_teleop, which has both brainco_retargeting[live] and
unitree_sdk2py):

    python gear_sonic/scripts/brainco_retargeting_demo.py --network-interface enp0s31f6
    python gear_sonic/scripts/brainco_retargeting_demo.py -i eth0 --hand right
    python gear_sonic/scripts/brainco_retargeting_demo.py --dry-run        # no DDS, viz only

Safety:
    * Make sure ``brainco_hand_service`` is up first (otherwise commands go
      nowhere). Test with its ``test_brainco_hand_server`` before this.
    * The hands move as soon as a hand is detected. Keep clear of the fingers.
    * Per-tick delta-q smoothing (same idea as ``brainco_hands.hpp``) caps how
      fast the fingers move; tune with ``--max-delta-q`` / ``--command-rate``.
    * On exit (``q`` or Ctrl-C) the hands are commanded fully open.
"""

import argparse
import os
import sys
import threading
import time
from pathlib import Path

# Must be set before cv2/Qt initialises to prevent black windows on Linux+NVIDIA
os.environ.setdefault("QT_X11_NO_MITSHM", "1")

# Allow running straight from the repo even if the package was not pip-installed.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "third_party" / "brainco-retargeting"))

import cv2
import mediapipe as mp
import numpy as np

from brainco_retargeting import BrainCoRetargeter
from brainco_retargeting import np_retargeting as _np_retargeting
from brainco_retargeting._geometry import MIRRORED_INPUT
from brainco_retargeting._utils import (
    _MOTOR_RANGES,
    SapienHandRenderer,
    draw_hand_skeleton_mp21,
    draw_motor_bars,
    mp21_to_xr25,
    select_hand,
)

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmd_, MotorCmds_, MotorStates_

NUM_MOTORS = 6  # BrainCo Revo2: 6 DOF per hand


def _swap_thumb(q: np.ndarray) -> np.ndarray:
    """Return a copy of a 6-vector with thumb slots 0<->1 exchanged.

    The physical BrainCo hand wires motors 0/1 in the opposite order to our
    slot convention [thumb_metacarpal, thumb_proximal, index, middle, ring,
    pinky]. We swap only at the DDS wire (outgoing cmd + incoming state), so
    everything above stays in our convention. This mirrors the deploy binary's
    fix in gear_sonic_deploy/.../brainco_hands.hpp (setThumbSwap).
    """
    out = np.asarray(q, dtype=float).copy()
    out[[0, 1]] = out[[1, 0]]
    return out

# np_retargeting joint order, used only for the --np-retarget path (mirrors live_camera.py).
_NP_JOINT_ORDER = [
    "thumb_metacarpal", "thumb_proximal",
    "index_proximal", "middle_proximal", "ring_proximal", "pinky_proximal",
]


def parse_args():
    p = argparse.ArgumentParser(
        description="Live MediaPipe -> BrainCo retargeting driving the real hands over DDS.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # --- camera / retargeting (mirrors demos/live_camera.py) ---
    p.add_argument("--camera-id", type=int, default=0, help="OpenCV camera device index")
    p.add_argument(
        "--hand",
        choices=["left", "right", "auto"],
        default="auto",
        help="Which physical hand to drive. 'auto' follows whichever hand is detected.",
    )
    p.add_argument("--width", type=int, default=640, help="Camera capture width")
    p.add_argument("--height", type=int, default=480, help="Camera capture height")
    p.add_argument(
        "--np-retarget",
        action="store_true",
        help="Use the pure-numpy retargeter instead of the optimization-based one.",
    )
    p.add_argument(
        "--no-render",
        action="store_true",
        help="Skip the Sapien hand render panel (still shows the camera + motor bars). "
        "Useful on headless / weak-GPU machines.",
    )
    # --- DDS / hand link ---
    p.add_argument(
        "-i", "--network-interface",
        type=str,
        default=None,
        help="Network interface on the robot's 192.168.123.x subnet (e.g. enp0s31f6, eth0). "
        "Required to reach the robot's brainco_hand_service. Omit only for --dry-run.",
    )
    p.add_argument("--domain-id", type=int, default=0, help="DDS domain id (Unitree default is 0).")
    p.add_argument(
        "--command-rate", type=float, default=100.0,
        help="DDS publish rate (Hz) of the background command thread.",
    )
    p.add_argument(
        "--max-delta-q", type=float, default=0.15,
        help="Max the command may LEAD the measured position per tick, in normalized units "
        "(like brainco_hands.hpp)."
    )
    p.add_argument(
        "--speed", type=float, default=1.0,
        help="Normalized finger speed dq commanded to the bridge [0..1].",
    )
    p.add_argument(
        "--thumb-swap",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Swap thumb motors 0/1 at the DDS wire to match the physical BrainCo "
        "hand (on by default, like the deploy binary on real hardware). Use "
        "--no-thumb-swap if driving a hand/bridge that does not need it.",
    )
    p.add_argument(
        "--keep-on-loss",
        action="store_true",
        help="Hold the last command when no hand is detected (matches live_camera). "
        "Default is the safer behaviour: open the hand when tracking is lost.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Do everything except publish to DDS (no hardware needed). For testing the pipeline.",
    )
    return p.parse_args()


class BrainCoHandPublisher:
    """DDS client that drives the real BrainCo hands, mirroring brainco_hands.hpp.

    A background thread publishes ``rt/brainco/{left,right}/cmd`` at a fixed rate,
    stepping the published position toward the latest target with per-tick
    delta-q clamping. Like the C++ driver, the clamp is applied relative to the
    measured ``rt/brainco/{side}/state`` when it is available (falling back to
    the last published value otherwise), so the fingers cannot jump.
    """

    def __init__(self, rate_hz: float, max_delta_q: float, speed: float, thumb_swap: bool = True):
        self._rate_hz = rate_hz
        self._max_delta_q = float(max_delta_q)
        self._speed = float(np.clip(speed, 0.0, 1.0))
        # Swap thumb motors 0/1 at the DDS wire for real hardware (see
        # _swap_thumb). Everything above (targets, published, smoothing) stays
        # in our slot convention.
        self._thumb_swap = bool(thumb_swap)

        self._lock = threading.Lock()
        # Targets and last-published positions per side. Start fully open.
        self._target = {"left": np.zeros(NUM_MOTORS), "right": np.zeros(NUM_MOTORS)}
        self._published = {"left": np.zeros(NUM_MOTORS), "right": np.zeros(NUM_MOTORS)}
        self._measured = {"left": None, "right": None}

        self._pubs = {}
        self._subs = {}
        for side in ("left", "right"):
            pub = ChannelPublisher(f"rt/brainco/{side}/cmd", MotorCmds_)
            pub.Init()
            self._pubs[side] = pub
            sub = ChannelSubscriber(f"rt/brainco/{side}/state", MotorStates_)
            sub.Init(self._make_state_handler(side), 1)
            self._subs[side] = sub

        self._running = False
        self._thread = None

    def _make_state_handler(self, side: str):
        def handler(msg: MotorStates_):
            try:
                q = np.array([msg.states[i].q for i in range(NUM_MOTORS)], dtype=float)
            except (IndexError, AttributeError):
                return
            # Bring the hardware's motor 0/1 back into our slot convention so the
            # delta-q smoothing (target vs. measured) compares matching slots.
            if self._thumb_swap:
                q = _swap_thumb(q)
            with self._lock:
                self._measured[side] = q
        return handler

    def set_target(self, side: str, motors: np.ndarray) -> None:
        """Set the normalized [0,1] target for one hand (first 6 values used)."""
        q = np.clip(np.asarray(motors, dtype=float)[:NUM_MOTORS], 0.0, 1.0)
        with self._lock:
            self._target[side] = q

    def open(self, side: str) -> None:
        self.set_target(side, np.zeros(NUM_MOTORS))

    def measured(self, side: str):
        with self._lock:
            m = self._measured[side]
        return None if m is None else m.copy()

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, name="brainco-cmd", daemon=True)
        self._thread.start()

    def _build_cmd(self, q: np.ndarray) -> MotorCmds_:
        cmd = MotorCmds_()
        cmd.cmds = [
            MotorCmd_(mode=0, q=float(q[i]), dq=self._speed, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0])
            for i in range(NUM_MOTORS)
        ]
        return cmd

    def _step_once(self) -> None:
        for side in ("left", "right"):
            with self._lock:
                target = self._target[side].copy()
                reference = self._measured[side]
                if reference is None:
                    reference = self._published[side].copy()
            # Clamp how far the command may lead the reference (measured state if
            # available) this tick. Not a hard speed cap: the firmware drives to
            # the command at its own dq rate, so this only bounds the lead (see
            # --max-delta-q / brainco_hands.hpp), preventing sudden jumps.
            delta = np.clip(target - reference, -self._max_delta_q, self._max_delta_q)
            q = np.clip(reference + delta, 0.0, 1.0)
            # Swap thumb 0/1 only on the wire; keep _published in our convention.
            wire_q = _swap_thumb(q) if self._thumb_swap else q
            self._pubs[side].Write(self._build_cmd(wire_q))
            with self._lock:
                self._published[side] = q

    def _run(self) -> None:
        period = 1.0 / self._rate_hz
        while self._running:
            t0 = time.perf_counter()
            self._step_once()
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)

    def shutdown(self, open_first: bool = True) -> None:
        if open_first and self._running:
            # Ramp both hands open before stopping the thread.
            self.open("left")
            self.open("right")
            time.sleep(max(0.5, 1.5 / self._rate_hz * (1.0 / max(self._max_delta_q, 1e-3))))
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)


def main():
    args = parse_args()

    if not args.dry_run and not args.network_interface:
        sys.exit(
            "ERROR: --network-interface is required to reach the robot's brainco_hand_service.\n"
            "       Pass your robot-subnet ethernet interface (e.g. -i enp0s31f6), or use --dry-run."
        )

    panel_w, panel_h = args.width, args.height

    # Create the display window FIRST to lock in Qt's OpenGL context before any
    # EGL init (MediaPipe and Sapien both trigger EGL on NVIDIA GPUs).
    placeholder = np.zeros((panel_h, panel_w * 2, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Initializing...", (panel_w // 2 + 160, panel_h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
    cv2.imshow("BrainCo Retargeting (REAL HANDS) - press q to quit", placeholder)
    cv2.waitKey(1)

    retargeter = None
    if not args.np_retarget:
        print("Loading retargeter...")
        retargeter = BrainCoRetargeter()
    else:
        print("Using pure-numpy retargeter.")

    print("Loading MediaPipe...")
    mp_hands_mod = mp.solutions.hands
    hands = mp_hands_mod.Hands(
        static_image_mode=False,
        max_num_hands=2,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    active_side = args.hand if args.hand != "auto" else "right"

    renderer = None
    if not args.no_render:
        print(f"Loading Sapien renderer ({active_side} hand)...")
        renderer = SapienHandRenderer(active_side, panel_w, panel_h)

    # ---- DDS hand publisher ----
    publisher = None
    if not args.dry_run:
        print(f"Initializing DDS on interface '{args.network_interface}' (domain {args.domain_id})...")
        ChannelFactoryInitialize(args.domain_id, args.network_interface)
        publisher = BrainCoHandPublisher(
            args.command_rate, args.max_delta_q, args.speed, thumb_swap=args.thumb_swap
        )
        publisher.start()
        print("Publishing rt/brainco/{left,right}/cmd. Make sure brainco_hand_service is running.")
    else:
        print("DRY RUN: not publishing to DDS.")

    # Open camera after all EGL/GPU init has settled.
    backend = cv2.CAP_V4L2 if sys.platform == "linux" else cv2.CAP_ANY
    cap = cv2.VideoCapture(args.camera_id, backend)
    if not cap.isOpened():
        if publisher is not None:
            publisher.shutdown()
        sys.exit(f"Cannot open camera {args.camera_id}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    for _ in range(15):
        cap.read()

    motors = np.zeros(NUM_MOTORS)
    print("Press 'q' (window focused) or Ctrl-C to quit. Hands open on exit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            if MIRRORED_INPUT:
                frame = cv2.flip(frame, 1)

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_rgb.flags.writeable = False
            results = hands.process(frame_rgb)
            frame_rgb.flags.writeable = True

            cam_panel = cv2.resize(frame, (panel_w, panel_h))

            picked = select_hand(results, args.hand)
            detected = picked is not None

            if detected:
                target_idx, active_side = picked

                # np_retargeting works on image-space normalized landmarks;
                # BrainCoRetargeter needs metric world landmarks.
                if args.np_retarget:
                    src = results.multi_hand_landmarks[target_idx]
                else:
                    if not results.multi_hand_world_landmarks:
                        detected = False
                        src = None
                    else:
                        src = results.multi_hand_world_landmarks[target_idx]

                if src is not None:
                    mp21 = np.array([[lm.x, lm.y, lm.z] for lm in src.landmark], dtype=np.float64)

                    if renderer is not None and renderer.side != active_side:
                        print(f"Side changed to {active_side}, reloading Sapien renderer...")
                        renderer = SapienHandRenderer(active_side, panel_w, panel_h)

                    if args.np_retarget:
                        angles = _np_retargeting.retarget(mp21, active_side)
                        raw = np.array([angles[f"{active_side}_{k}_joint"] for k in _NP_JOINT_ORDER])
                        motors = np.array([(r - lo) / (hi - lo) for r, (lo, hi) in zip(raw, _MOTOR_RANGES)])
                    else:
                        xr25 = retargeter.canonicalize(mp21_to_xr25(mp21), active_side)
                        retarget_fn = (
                            retargeter.retarget_left if active_side == "left" else retargeter.retarget_right
                        )
                        motors = retarget_fn(xr25)

                    motors = np.clip(np.asarray(motors, dtype=float), 0.0, 1.0)
                    draw_hand_skeleton_mp21(cam_panel, results.multi_hand_landmarks[target_idx])

            # ---- drive the real hand(s) ----
            if publisher is not None:
                if detected:
                    publisher.set_target(active_side, motors)
                    # Keep the hand we are NOT tracking safely open.
                    other = "left" if active_side == "right" else "right"
                    if args.hand == "auto":
                        publisher.open(other)
                elif not args.keep_on_loss:
                    # Lost tracking: open the active hand (safe default).
                    publisher.open(active_side)
                    motors = np.zeros(NUM_MOTORS)

            # ---- visualization ----
            if renderer is not None:
                robot_panel = renderer.render(motors)
            else:
                robot_panel = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
            robot_panel = draw_motor_bars(robot_panel, motors)

            combined = np.hstack([cam_panel, robot_panel])
            cv2.putText(combined, "Camera", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.putText(combined, "BrainCo Hand", (panel_w + 10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.putText(combined, active_side.upper(), (panel_w + 10, 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 220, 100), 2)
            status = "DRY RUN" if args.dry_run else ("LIVE" if detected else "no hand")
            color = (60, 200, 255) if args.dry_run else ((100, 220, 100) if detected else (80, 80, 220))
            cv2.putText(combined, status, (panel_w + 10, panel_h - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            cv2.imshow("BrainCo Retargeting (REAL HANDS) - press q to quit", combined)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if publisher is not None:
            print("Opening hands and shutting down DDS...")
            publisher.shutdown(open_first=True)
        cap.release()
        cv2.destroyAllWindows()
        hands.close()


if __name__ == "__main__":
    main()
