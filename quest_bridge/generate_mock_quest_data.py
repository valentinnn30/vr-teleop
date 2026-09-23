#!/usr/bin/env python3
"""
Synthetic Quest data *generator*.

Produces ``.npz`` trajectory files in the exact same format that
``record_quest_data.py`` writes, but without any Quest, ROS, or Docker relay in
the loop — purely procedural, deterministic motion you can use to exercise
downstream teleop / data-processing code when no headset is available.

The output schema mirrors ``record_quest_data.save_trajectory`` one-for-one
(T = number of frames):

    timestamp          (T,)        float64  monotonically increasing wall-clock
    head_pos           (T, 3)      head_quat        (T, 4)  scalar-first [w,x,y,z]
    left_wrist_pos     (T, 3)      left_wrist_quat  (T, 4)
    right_wrist_pos    (T, 3)      right_wrist_quat (T, 4)
    left_landmarks     (T, 21, 3)  right_landmarks  (T, 21, 3)  MANO, world frame
    left_tracked       (T,) bool   right_tracked    (T,) bool
plus a JSON ``metadata`` string array.

Scales (head height, wrist offsets, finger bone lengths, ~90 Hz, world-frame
landmarks anchored at the wrist) were calibrated from the real recordings under
``data/quest/traj_*.npz`` so the synthetic streams sit in the same numeric range.

The 21 landmarks follow the MANO joint order used by the relay:
    0           wrist
    1..4        thumb   (CMC -> MCP -> IP -> TIP)
    5..8        index   (MCP -> PIP -> DIP -> TIP)
    9..12       middle
    13..16      ring
    17..20      pinky
Each hand is built as a rest skeleton with realistic per-finger bone lengths,
flexed by a per-frame curl signal, then placed so it is self-consistent with
``*_wrist_pos`` / ``*_wrist_quat`` under the SAME two-topic frame convention the
real recordings exhibit (see ``_R_TOPIC`` / ``_R_PALM``): the pose topic and the
landmarks topic live in world frames rotated +90° about Z from each other, so
``landmark[0] == R_TOPIC @ wrist_pos`` rather than ``== wrist_pos``.

Everything is deterministic given ``--seed``: trajectory ``i`` uses ``seed + i``,
so re-running reproduces byte-identical motion (modulo the filename timestamp).

Usage (from anywhere in the repo):
    python gear_sonic_deploy/docker/quest_relay/generate_mock_quest_data.py
    python gear_sonic_deploy/docker/quest_relay/generate_mock_quest_data.py \
        --output-dir data/quest --num-trajectories 5 --duration 17 --hz 90 --seed 0

    # Crouch test motion for the manager's head-height -> planner height command
    # (opt-in style; replay it with quest_manager_thread_server.py --replay):
    python gear_sonic_deploy/docker/quest_relay/generate_mock_quest_data.py \
        --styles crouch --num-trajectories 1 --duration 24 --output-dir data/quest/mock
"""

import argparse
import datetime as dt
import json
import time
from pathlib import Path

import numpy as np

# Mirror record_quest_data.py so a generated file is indistinguishable downstream.
_ARRAY_FIELDS = (
    "timestamp",
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

# --- Calibrated constants (means/ranges measured from data/quest/traj_*.npz) ---

# Head sits ~1.27 m up, a touch behind/left of the world origin.
_HEAD_BASE = np.array([-0.25, 0.03, 1.27])
# Resting near-identity head orientation with a slight forward/side tilt.
_HEAD_QUAT_BASE = np.array([0.991, 0.044, 0.111, -0.063])

# Wrists rest ~0.34 m below the head, one to each side, slightly forward.
_LEFT_WRIST_BASE = np.array([0.05, 0.16, 0.93])
_RIGHT_WRIST_BASE = np.array([-0.04, -0.20, 0.94])
# Resting palm-roughly-down wrist orientations (measured trajectory means).
_LEFT_WRIST_QUAT_BASE = np.array([-0.749, 0.272, 0.269, 0.235])
_RIGHT_WRIST_QUAT_BASE = np.array([0.783, 0.246, -0.301, 0.041])

# Frame conventions between the two ROS topics, measured by Kabsch/Procrustes
# fits against the real recordings (consistent across every data/quest/traj_*).
#
#   landmark_world = R_TOPIC @ ( wrist_pos + R(wrist_quat) @ R_PALM @ local )
#
# * R_TOPIC: the MANO-landmarks topic publishes in a world frame rotated +90°
#   about Z relative to the head/wrist-pose topic (fit RMSE ~1-10 mm, ~0 trans).
#   head_pos / *_wrist_pos stay in the pose frame; only landmarks are rotated.
# * R_PALM: fixed wrist-tracker -> MANO-palm offset (~180° flip about hand x),
#   so the rendered hand sits relative to wrist_quat exactly as in real data.
_R_TOPIC = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
_R_PALM = {
    "left": np.array(
        [[0.9995, -0.0148, -0.0282], [-0.018, -0.9929, -0.1173], [-0.0262, 0.1177, -0.9927]]
    ),
    "right": np.array(
        [[0.9995, 0.0152, -0.0288], [0.0185, -0.993, 0.1168], [-0.0268, -0.1172, -0.9927]]
    ),
}
# The left hand is the mirror image of the right: reflecting the canonical
# (right-chirality) skeleton across the palm's y=0 plane flips chirality (det -1)
# so the left hand has the correct handedness, exactly like the real recordings.
_REFLECT_Y = np.diag([1.0, -1.0, 1.0])

# Per-finger bone segment lengths [m], measured from the real recordings.
#   thumb : CMC->MCP, MCP->IP, IP->TIP, TIP (4 segments off the wrist)
# Index/middle/ring/pinky: wrist->MCP (palm), MCP->PIP, PIP->DIP, DIP->TIP.
_BONES = {
    "thumb": [0.0518, 0.0345, 0.0358, 0.0261],
    "index": [0.1050, 0.0402, 0.0258, 0.0237],
    "middle": [0.1014, 0.0455, 0.0292, 0.0265],
    "ring": [0.0961, 0.0413, 0.0282, 0.0258],
    "pinky": [0.0917, 0.0326, 0.0215, 0.0233],
}
# Landmark index of each finger's 4 joints (MANO order, wrist is index 0).
_FINGER_IDX = {
    "thumb": [1, 2, 3, 4],
    "index": [5, 6, 7, 8],
    "middle": [9, 10, 11, 12],
    "ring": [13, 14, 15, 16],
    "pinky": [17, 18, 19, 20],
}
# Fan angle of each finger's base direction about the palm normal (+z), radians.
# Thumb sticks out toward the palm side; the others fan out from index to pinky.
_FINGER_FAN = {
    "thumb": 0.95,
    "index": 0.26,
    "middle": 0.09,
    "ring": -0.09,
    "pinky": -0.28,
}


# ----------------------------- quaternion helpers ----------------------------
# Scalar-first [w, x, y, z], matching the recorder's convention.


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def _quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    h = 0.5 * angle
    return np.array([np.cos(h), *(np.sin(h) * axis)])


def _quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Intrinsic X(roll)->Y(pitch)->Z(yaw)."""
    qx = _quat_from_axis_angle(np.array([1.0, 0, 0]), roll)
    qy = _quat_from_axis_angle(np.array([0, 1.0, 0]), pitch)
    qz = _quat_from_axis_angle(np.array([0, 0, 1.0]), yaw)
    return _quat_normalize_mul(_quat_normalize_mul(qz, qy), qx)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w0, x0, y0, z0 = a
    w1, x1, y1, z1 = b
    return np.array(
        [
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        ]
    )


def _quat_normalize_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return _normalize_quat(_quat_mul(a, b))


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quat(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


# ------------------------------ hand skeleton --------------------------------


def _canonical_hand() -> dict[str, np.ndarray]:
    """Build a flat (uncurled) canonical right-chirality MANO skeleton.

    Local frame: +x points along the fingers (away from the wrist), +y across
    the palm (pinky -> index), +z is the back-of-hand normal. The left hand is
    produced by reflecting this skeleton (see ``_REFLECT_Y``). Returns the 21x3
    rest joint positions plus, per finger, the flexion axis to curl about.
    """
    joints = np.zeros((21, 3))
    curl_axes: dict[str, np.ndarray] = {}

    for name, bones in _BONES.items():
        fan = -_FINGER_FAN[name]
        # Base direction of the finger in the palm plane (rotated about +z).
        base_dir = np.array([np.cos(fan), np.sin(fan), 0.0])
        # Thumb starts lower on the palm and tips slightly out of plane.
        if name == "thumb":
            base_dir = base_dir + np.array([-0.15, 0.0, 0.25])
            base_dir /= np.linalg.norm(base_dir)
        pos = np.zeros(3)
        for seg_len, jidx in zip(bones, _FINGER_IDX[name]):
            pos = pos + seg_len * base_dir
            joints[jidx] = pos
        # Fingers flex toward the palm (-z): curl axis is in the palm plane,
        # perpendicular to the finger's base direction.
        curl_axes[name] = np.array([base_dir[1], -base_dir[0], 0.0])

    return {"joints": joints, "curl_axes": curl_axes}


def _curl_hand(rest: dict, curls: dict[str, float]) -> np.ndarray:
    """Apply per-finger flexion to the rest skeleton, returning 21x3 positions.

    ``curls[name]`` in [0, 1] drives progressive flexion: the MCP/PIP/DIP joints
    bend by an increasing fraction of the curl so a value near 1 makes a fist.
    """
    out = rest["joints"].copy()
    # Fraction of the curl applied at the three movable joints of each finger.
    weights = (0.45, 0.9, 0.75)
    max_flex = np.pi  # full curl ~= 180 deg distributed across the chain
    for name, idxs in _FINGER_IDX.items():
        axis = rest["curl_axes"][name]
        c = float(np.clip(curls[name], 0.0, 1.0))
        rest_pts = rest["joints"]
        # rest_pts[idxs[0]] is the MCP (fixed); flex the chain distal to it.
        prev = rest_pts[idxs[0]]
        out[idxs[0]] = prev
        racc = np.eye(3)
        for k in range(1, 4):
            racc = racc @ _quat_to_matrix(_quat_from_axis_angle(axis, c * weights[k - 1] * max_flex))
            bone = rest_pts[idxs[k]] - rest_pts[idxs[k - 1]]
            out[idxs[k]] = out[idxs[k - 1]] + racc @ bone
    return out


# ------------------------------ motion synthesis -----------------------------


def _smoothstep(x: np.ndarray) -> np.ndarray:
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def _crouch_profile(t: np.ndarray, depth: float) -> np.ndarray:
    """Repeating stand -> squat -> stand head-drop profile (m, positive down).

    One 10 s cycle: 2 s standing, 2 s down, 4 s held at ``depth``, 2 s back up.
    The manager engages its crouch ~0.07 m into the descent, so the held phase is
    what actually exercises a steady squat.
    """
    phase = np.mod(t, 10.0)
    down = _smoothstep((phase - 2.0) / 2.0)
    up = _smoothstep((phase - 8.0) / 2.0)
    return depth * (down - up)


def generate_trajectory(
    seed: int,
    duration: float,
    hz: float,
    style: str,
    start_time: float,
    crouch_depth: float = 0.25,
) -> dict[str, np.ndarray]:
    """Procedurally synthesize one trajectory dict of stacked arrays."""
    rng = np.random.default_rng(seed)
    n = max(2, int(round(duration * hz)))
    dt_nom = 1.0 / hz
    t = np.arange(n) * dt_nom

    # Timestamps: nominal cadence + small per-frame jitter (relay is not exact).
    jitter = rng.normal(0.0, 0.1 * dt_nom, size=n)
    ts = start_time + np.cumsum(np.maximum(1e-4, dt_nom + jitter))

    # Style-dependent motion gains.
    style_cfg = {
        "idle": dict(head_amp=0.02, reach=0.05, curl_amp=0.25, walk=0.0, wave=0.0),
        "reach": dict(head_amp=0.05, reach=0.30, curl_amp=0.6, walk=0.0, wave=0.0),
        "wave": dict(head_amp=0.04, reach=0.12, curl_amp=0.5, walk=0.0, wave=1.0),
        "walk": dict(head_amp=0.10, reach=0.20, curl_amp=0.4, walk=1.0, wave=0.0),
        "crouch": dict(head_amp=0.02, reach=0.10, curl_amp=0.4, walk=0.0, wave=0.0),
    }[style]
    # Styles whose head translates enough that the wrists must follow the body.
    follow_head = style in ("walk", "crouch")

    # Per-trajectory random phases/frequencies keep variety while staying smooth.
    f_head = rng.uniform(0.15, 0.4)
    f_arm = rng.uniform(0.2, 0.6)
    f_curl = rng.uniform(0.3, 0.8)
    ph = rng.uniform(0, 2 * np.pi, size=8)

    # --- head pose ---
    head_pos = np.tile(_HEAD_BASE, (n, 1)).astype(float)
    sway = style_cfg["head_amp"]
    head_pos[:, 0] += sway * np.sin(2 * np.pi * f_head * t + ph[0])
    head_pos[:, 1] += 0.6 * sway * np.sin(2 * np.pi * f_head * 1.3 * t + ph[1])
    head_pos[:, 2] += 0.4 * sway * np.sin(2 * np.pi * f_head * 2.0 * t + ph[2])
    if style == "walk":
        # Slow forward drift with a gentle turn, plus a vertical bob per "step".
        drift = _smoothstep(t / max(t[-1], 1e-6))
        head_pos[:, 0] += 0.4 * drift
        head_pos[:, 1] += 0.3 * drift * np.sin(0.5 * np.pi * drift)
        head_pos[:, 2] += 0.03 * np.sin(2 * np.pi * 1.8 * t)
    if style == "crouch":
        head_pos[:, 2] -= _crouch_profile(t, crouch_depth)

    head_quat = np.zeros((n, 4))
    yaw = 0.25 * sway * np.sin(2 * np.pi * f_head * 0.7 * t + ph[3])
    pitch = 0.15 * np.sin(2 * np.pi * f_head * 1.1 * t + ph[4])
    if style == "walk":
        yaw = yaw + 0.5 * _smoothstep(t / max(t[-1], 1e-6)) * np.sin(0.5 * np.pi * drift)
    for i in range(n):
        head_quat[i] = _quat_normalize_mul(
            _quat_from_euler(0.0, pitch[i], yaw[i]), _HEAD_QUAT_BASE
        )

    # --- wrists ---
    def wrist_track(base, base_quat, side_sign):
        pos = np.tile(base, (n, 1)).astype(float)
        reach = style_cfg["reach"]
        # Forward/up reaching arcs, mirrored per hand.
        pos[:, 0] += reach * (0.5 + 0.5 * np.sin(2 * np.pi * f_arm * t + ph[5]))
        pos[:, 1] += side_sign * 0.4 * reach * np.sin(2 * np.pi * f_arm * 0.8 * t + ph[6])
        pos[:, 2] += reach * 0.7 * np.sin(2 * np.pi * f_arm * 1.1 * t + ph[7])
        if follow_head:
            pos += head_pos - _HEAD_BASE  # wrists follow the body as it moves
        quat = np.zeros((n, 4))
        for i in range(n):
            roll = 0.3 * np.sin(2 * np.pi * f_arm * t[i] + ph[5])
            p = 0.25 * np.sin(2 * np.pi * f_arm * 0.9 * t[i] + ph[6])
            if style == "wave" and side_sign > 0:  # left hand waves side to side
                roll += 0.8 * np.sin(2 * np.pi * 1.5 * t[i])
            quat[i] = _quat_normalize_mul(_quat_from_euler(roll, p, 0.0), base_quat)
        return pos, quat

    left_wrist_pos, left_wrist_quat = wrist_track(_LEFT_WRIST_BASE, _LEFT_WRIST_QUAT_BASE, +1.0)
    right_wrist_pos, right_wrist_quat = wrist_track(_RIGHT_WRIST_BASE, _RIGHT_WRIST_QUAT_BASE, -1.0)

    # --- finger curls ---
    rest_canon = _canonical_hand()

    def curl_signal(side_phase):
        # Open/close cycle, fingers slightly staggered (pinky leads thumb).
        base = 0.5 + 0.5 * np.sin(2 * np.pi * f_curl * t + side_phase)
        amp = style_cfg["curl_amp"]
        per_finger = {}
        for j, name in enumerate(("thumb", "index", "middle", "ring", "pinky")):
            stagger = 0.15 * np.sin(2 * np.pi * f_curl * t + side_phase + 0.4 * j)
            per_finger[name] = 0.15 + amp * np.clip(base + stagger, 0.0, 1.0)
        return per_finger

    left_curls = curl_signal(ph[0])
    right_curls = curl_signal(ph[1])

    left_landmarks = np.zeros((n, 21, 3))
    right_landmarks = np.zeros((n, 21, 3))
    rpalm_l, rpalm_r = _R_PALM["left"], _R_PALM["right"]
    for i in range(n):
        lc = {k: v[i] for k, v in left_curls.items()}
        rc = {k: v[i] for k, v in right_curls.items()}
        # Right hand is the canonical skeleton; left hand is its y-reflection.
        ll = (_REFLECT_Y @ _curl_hand(rest_canon, lc).T).T
        rl = _curl_hand(rest_canon, rc)
        # Place the local hand at the wrist pose (apply the fixed wrist->palm
        # offset), then rotate the whole cloud into the landmarks-topic frame so
        # landmark[0] == R_TOPIC @ wrist_pos, matching the real recordings.
        lw = (_quat_to_matrix(left_wrist_quat[i]) @ rpalm_l @ ll.T).T + left_wrist_pos[i]
        rw = (_quat_to_matrix(right_wrist_quat[i]) @ rpalm_r @ rl.T).T + right_wrist_pos[i]
        left_landmarks[i] = (_R_TOPIC @ lw.T).T
        right_landmarks[i] = (_R_TOPIC @ rw.T).T

    # --- sensor-like noise (tracker jitter), then renormalize quats ---
    pos_sigma = 0.0015
    for arr in (head_pos, left_wrist_pos, right_wrist_pos, left_landmarks, right_landmarks):
        arr += rng.normal(0.0, pos_sigma, size=arr.shape)
    for q in (head_quat, left_wrist_quat, right_wrist_quat):
        q += rng.normal(0.0, 0.002, size=q.shape)
        q /= np.linalg.norm(q, axis=1, keepdims=True)

    # --- tracked flags: always tracked ---
    left_tracked = np.ones(n, dtype=bool)
    right_tracked = np.ones(n, dtype=bool)

    return {
        "timestamp": ts,
        "head_pos": head_pos,
        "head_quat": head_quat,
        "left_wrist_pos": left_wrist_pos,
        "left_wrist_quat": left_wrist_quat,
        "right_wrist_pos": right_wrist_pos,
        "right_wrist_quat": right_wrist_quat,
        "left_landmarks": left_landmarks,
        "right_landmarks": right_landmarks,
        "left_tracked": left_tracked,
        "right_tracked": right_tracked,
    }


def save_trajectory(
    traj: dict[str, np.ndarray], out_dir: Path, idx: int, hz: float, style: str, seed: int
) -> Path:
    """Write one trajectory dict to a compressed .npz (record_quest_data format)."""
    arrays = {field: traj[field] for field in _ARRAY_FIELDS}
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"traj_{stamp}_{idx:03d}.npz"
    meta = json.dumps(
        {
            "num_frames": len(traj["timestamp"]),
            "hz": hz,
            "head_topic": None,
            "left_hand_topic": None,
            "right_hand_topic": None,
            "saved_at": dt.datetime.now().isoformat(timespec="seconds"),
            "synthetic": True,
            "style": style,
            "seed": seed,
        }
    )
    np.savez_compressed(path, metadata=np.array(meta), **arrays)
    duration = traj["timestamp"][-1] - traj["timestamp"][0]
    print(
        f"[gen_quest] Saved {path.name}: {len(traj['timestamp'])} frames, "
        f"{duration:.1f}s, style={style}, seed={seed}."
    )
    return path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate synthetic Quest trajectory .npz files (record_quest_data format).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("data/quest"),
        help="Directory to write trajectory .npz files into (created if missing).",
    )
    parser.add_argument(
        "--num-trajectories", type=int, default=4, help="How many trajectory files to generate."
    )
    parser.add_argument("--duration", type=float, default=17.0, help="Seconds per trajectory.")
    parser.add_argument("--hz", type=float, default=90.0, help="Frame rate.")
    parser.add_argument("--seed", type=int, default=0, help="Base RNG seed (traj i uses seed+i).")
    parser.add_argument(
        "--start-index", type=int, default=0, help="Starting value for the file index suffix."
    )
    parser.add_argument(
        "--styles", nargs="+", default=["idle", "reach", "wave", "walk"],
        choices=["idle", "reach", "wave", "walk", "crouch"],
        help="Motion styles to cycle through across trajectories. 'crouch' is opt-in: "
        "it repeats a stand/squat/stand cycle to drive the manager's crouch command.",
    )
    parser.add_argument(
        "--crouch-depth", type=float, default=0.25,
        help="(crouch style) how far the head sinks below standing, in metres. "
        "0.25 gives a commanded base height of ~0.54m, inside the squat band.",
    )
    args = parser.parse_args()

    out_dir = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[gen_quest] Writing {args.num_trajectories} trajectories to {out_dir.resolve()}")

    base_time = time.time()
    for k in range(args.num_trajectories):
        seed = args.seed + k
        style = args.styles[k % len(args.styles)]
        traj = generate_trajectory(
            seed=seed, duration=args.duration, hz=args.hz, style=style,
            start_time=base_time + k * (args.duration + 1.0),
            crouch_depth=args.crouch_depth,
        )
        save_trajectory(traj, out_dir, args.start_index + k, args.hz, style, seed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
