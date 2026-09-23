#!/usr/bin/env python3
"""
Host-side Quest data *recorder*.

Same lifecycle as ``run_quest_relay.py`` (build + run the relay container, map
the ports, tear it down on exit), but instead of expecting a downstream teleop
manager to consume the ZMQ ``quest_data`` blob, this script subscribes to it and
records the raw tracking stream to disk as trajectories — for offline processing
and finetuning.

The relay already aggregates *all* Quest topics into a single ``quest_data``
msgpack snapshot (head pose, both wrist poses, both 21-joint MANO hands, the
per-hand tracked flags, and a timestamp), so subscribing to that one blob
captures every topic without needing any ROS install on the host.

Recording is toggled from the keyboard:
    SPACE  start / stop a trajectory (each on→off span is saved as one file)
    q      quit (a trajectory still recording is flushed first)

Data flow:
    Quest (Unity) --TCP:10000--> ros_tcp_endpoint --TCPROS--> relay --ZMQ:5559--> recorder

Each trajectory is written to ``--output-dir`` as a compressed ``.npz`` with
stacked arrays (T = number of frames):
    timestamp          (T,)      float64  relay-side time.time() per frame
    head_pos           (T, 3)    head_quat  (T, 4)   scalar-first [w,x,y,z]
    left_wrist_pos     (T, 3)    left_wrist_quat  (T, 4)
    right_wrist_pos    (T, 3)    right_wrist_quat (T, 4)
    left_landmarks     (T, 21, 3)   right_landmarks (T, 21, 3)
    left_tracked       (T,)  bool   right_tracked    (T,)  bool
plus string metadata (hz, topics, wall-clock start/stop).

Usage (from anywhere in the repo):
    python gear_sonic_deploy/docker/quest_relay/record_quest_data.py
    python gear_sonic_deploy/docker/quest_relay/record_quest_data.py --output-dir /data/quest --rebuild

Frames arrive at the relay ``--hz`` rate regardless of whether the underlying
topics changed, so consecutive frames may repeat values; pass ``--dedupe`` to
skip frames whose ``timestamp`` is unchanged from the previously recorded one.
"""

import argparse
import datetime as dt
import json
import queue
import select
import signal
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import msgpack
import numpy as np
import zmq

# Reuse the container lifecycle from the sibling launcher.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_quest_relay as rqr  # noqa: E402

# Fields stacked into per-trajectory arrays (everything else in the snapshot is
# metadata). Shapes are inferred by np.array, so variable-but-consistent fields
# (always 21 padded landmarks) stack cleanly.
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


def start_container(tag: str, args: argparse.Namespace) -> None:
    """Run the relay container detached, then wait for its ZMQ port to come up."""
    rqr.remove_stale_container(args.name)
    cmd = ["docker", "run", "-d", "--rm", "--name", args.name]
    cmd += [*rqr.port_mappings(args), tag, *rqr.relay_args(args)]
    result = rqr._run(cmd)
    if result.returncode != 0:
        sys.exit(f"[record_quest] ERROR: docker run failed (exit {result.returncode}).")
    print(f"[record_quest] Container '{args.name}' started. Waiting for ZMQ port {args.zmq_port}...")
    if not rqr.wait_for_zmq(args.zmq_port):
        rqr.stop_container(args.name)
        sys.exit(
            f"[record_quest] ERROR: ZMQ port {args.zmq_port} did not come up. "
            f"Check 'docker logs {args.name}'."
        )


def save_trajectory(frames: list[dict], out_dir: Path, idx: int, args: argparse.Namespace) -> Path:
    """Stack a list of frame dicts into arrays and write a compressed .npz."""
    arrays = {field: np.array([f[field] for f in frames]) for field in _ARRAY_FIELDS}
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"traj_{stamp}_{idx:03d}.npz"
    meta = json.dumps(
        {
            "num_frames": len(frames),
            "hz": args.hz,
            "head_topic": args.head_topic,
            "left_hand_topic": args.left_hand_topic,
            "right_hand_topic": args.right_hand_topic,
            "saved_at": dt.datetime.now().isoformat(timespec="seconds"),
        }
    )
    np.savez_compressed(path, metadata=np.array(meta), **arrays)
    duration = frames[-1]["timestamp"] - frames[0]["timestamp"] if len(frames) > 1 else 0.0
    print(f"[record_quest] Saved {path.name}: {len(frames)} frames, {duration:.1f}s of data.")
    return path


class KeyboardListener:
    """Background single-keypress reader (cbreak) that pushes chars onto a queue.

    Falls back to no-op if stdin is not a TTY (e.g. piped), in which case the
    recorder records one continuous trajectory until Ctrl-C.
    """

    def __init__(self) -> None:
        self.keys: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._old_term = None

    @property
    def enabled(self) -> bool:
        return self._fd is not None

    def __enter__(self) -> "KeyboardListener":
        if self.enabled:
            self._old_term = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        return self

    def _loop(self) -> None:
        while not self._stop.is_set():
            if select.select([sys.stdin], [], [], 0.2)[0]:
                ch = sys.stdin.read(1)
                if ch:
                    self.keys.put(ch)

    def __exit__(self, *exc) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.enabled and self._old_term is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)


def record_loop(args: argparse.Namespace, out_dir: Path) -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://localhost:{args.zmq_port}")
    sub.setsockopt(zmq.SUBSCRIBE, b"quest_data")
    sub.setsockopt(zmq.RCVTIMEO, 100)  # 100 ms so we stay responsive to keys
    print(f"[record_quest] Subscribed to relay at localhost:{args.zmq_port}")

    quit_flag = {"v": False}

    def _on_signal(signum, frame):
        quit_flag["v"] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    with KeyboardListener() as kb:
        if kb.enabled:
            print("\n[record_quest] SPACE = start/stop recording, q = quit.\n")
            recording = False
        else:
            print("\n[record_quest] stdin is not a TTY — recording continuously until Ctrl-C.\n")
            recording = True

        frames: list[dict] = []
        traj_idx = 0
        last_ts = None
        last_status = time.time()

        while not quit_flag["v"]:
            # Handle keypresses.
            while not kb.keys.empty():
                ch = kb.keys.get()
                if ch == " ":
                    recording = not recording
                    if recording:
                        frames = []
                        last_ts = None
                        print("[record_quest] ● RECORDING...")
                    else:
                        if frames:
                            save_trajectory(frames, out_dir, traj_idx, args)
                            traj_idx += 1
                        else:
                            print("[record_quest] ○ stopped (no frames captured).")
                elif ch in ("q", "\x03"):
                    quit_flag["v"] = True

            # Receive one frame.
            try:
                parts = sub.recv_multipart()
            except zmq.Again:
                continue
            frame = msgpack.unpackb(parts[1], raw=False)

            if recording:
                ts = frame["timestamp"]
                if args.dedupe and ts == last_ts:
                    continue
                last_ts = ts
                frames.append(frame)

            # Periodic status line.
            now = time.time()
            if now - last_status >= 2.0:
                state = f"REC ({len(frames)} frames)" if recording else "idle"
                hp = frame["head_pos"]
                print(
                    f"[record_quest] {state} | head=({hp[0]:+.3f}, {hp[1]:+.3f}, {hp[2]:+.3f}) "
                    f"| L={'ok' if frame['left_tracked'] else '-'} "
                    f"R={'ok' if frame['right_tracked'] else '-'}"
                )
                last_status = now

        # Flush an in-progress trajectory on quit.
        if recording and frames:
            print("\n[record_quest] Flushing in-progress trajectory...")
            save_trajectory(frames, out_dir, traj_idx, args)

    sub.close()
    ctx.term()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build/run the Quest relay container and record its data stream to disk.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("quest_recordings"),
        help="Directory to write trajectory .npz files into (created if missing).",
    )
    parser.add_argument(
        "--dedupe", action="store_true",
        help="Skip frames whose timestamp is unchanged from the previous recorded frame.",
    )
    # Container lifecycle args — mirror run_quest_relay.py so rqr helpers can reuse them.
    parser.add_argument("--image-tag", default="quest-relay:latest", help="Docker image tag.")
    parser.add_argument("--name", default="quest-relay", help="Container name.")
    parser.add_argument(
        "--tcp-port", type=int, default=10000,
        help="Host port for the ROS-TCP endpoint (Quest connects here).",
    )
    parser.add_argument(
        "--zmq-port", type=int, default=5559,
        help="Host port the relay publishes quest_data on (recorder subscribes here).",
    )
    parser.add_argument(
        "--rebuild", action="store_true", help="Force rebuild even if the image exists."
    )
    parser.add_argument(
        "--no-build", action="store_true", help="Never build; fail if the image is missing."
    )
    # Forwarded to relay.py inside the container.
    parser.add_argument("--head-topic", default=None, help="ROS1 head pose topic.")
    parser.add_argument("--left-hand-topic", default=None, help="ROS1 left ManoLandmarks topic.")
    parser.add_argument("--right-hand-topic", default=None, help="ROS1 right ManoLandmarks topic.")
    parser.add_argument("--hz", type=float, default=None, help="Relay publish rate in Hz.")
    args = parser.parse_args()

    rqr.preflight()

    have_image = rqr.image_exists(args.image_tag)
    if args.rebuild or not have_image:
        if args.no_build:
            if not have_image:
                sys.exit(
                    f"[record_quest] ERROR: image '{args.image_tag}' not found and --no-build was given."
                )
        else:
            rqr.build_image(args.image_tag)
    else:
        print(f"[record_quest] Reusing existing image '{args.image_tag}' (pass --rebuild to force).")

    out_dir = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[record_quest] Writing trajectories to {out_dir.resolve()}")

    start_container(args.image_tag, args)
    try:
        record_loop(args, out_dir)
    finally:
        rqr.stop_container(args.name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
