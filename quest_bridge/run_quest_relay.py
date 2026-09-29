#!/usr/bin/env python3
"""
Host-side launcher for the Quest → ZMQ relay container.

Automates the full lifecycle so you don't have to run docker by hand:
build the image, run the container (with the right port mappings), stream
its logs, and tear it down cleanly on Ctrl-C.

The container runs *both* the ROS-TCP endpoint (Quest connects here over the
Unity ROS-TCP-Connector protocol, launched via roslaunch on a ROS1/Noetic
master) and the ZMQ relay that republishes the tracking data as a single
msgpack ``quest_data`` blob. The host-side ``teleop_manager/quest_manager.py``
consumes that blob with ``--relay-host`` (no ROS needed on the host).

Data flow:
    Quest (Unity) --TCP:10000--> ros_tcp_endpoint --TCPROS--> relay --ZMQ:5559--> manager

Usage (from anywhere in the repo):
    python3 quest_bridge/run_quest_relay.py
    python3 quest_bridge/run_quest_relay.py --rebuild
    python3 quest_bridge/run_quest_relay.py --detach

Then, in another shell:
    python -m teleop_manager.quest_manager --relay-host localhost

Record / replay (off-robot retargeting + latency work):
    python3 quest_bridge/run_quest_relay.py --record-bag ./bags
    python3 quest_bridge/run_quest_relay.py --play-bag ./bags/quest_20260929_120000.bag

In --play-bag mode no Quest is involved: the bag supplies the ROS topics the
endpoint would have published, and the relay itself is unchanged, so the manager
sees a stream indistinguishable from live. Pair it with the manager's --no-robot
(there is no deploy to report robot feedback).
"""

import argparse
import hashlib
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

# quest_bridge -> <repo root>. This is the docker build context, and the
# Dockerfile COPYs from both third_party/ and quest_bridge/, so it must be the
# repo root and nothing shallower.
REPO_ROOT = Path(__file__).resolve().parents[1]
DOCKERFILE = Path(__file__).resolve().parent / "Dockerfile"
RELAY_PY = Path(__file__).resolve().parent / "relay.py"

# Container-internal ports (fixed; see Dockerfile EXPOSE and relay.py defaults).
CONTAINER_TCP_PORT = 10000
CONTAINER_ZMQ_PORT = 5559

# Everything baked into the image. Docker's own layer cache cannot help us decide
# whether to rebuild, because we only ever ask "does this tag exist?" — so an
# edited entrypoint.sh would otherwise sit unused behind an image that already
# exists. We stamp a fingerprint of these inputs as a label at build time and
# compare it before each run.
IMAGE_SOURCES = (
    "Dockerfile",
    "entrypoint.sh",
    "relay.py",
    "image_relay.py",
    "endpoint_no_adb.launch",
)
IMAGE_SUBMODULES = ("third_party/ROS-TCP-Endpoint", "third_party/vr_haptic_msgs")
FINGERPRINT_LABEL = "vr_teleop.fingerprint"


def image_fingerprint() -> str:
    """Short content hash of everything the image is built from.

    Hashes file CONTENT rather than mtimes: a `git pull` or a fresh clone rewrites
    mtimes without changing anything, and a needless rebuild here costs an apt
    install plus catkin_make on a Jetson. Submodules contribute their commit id
    instead of their contents, which is both cheaper and exactly as precise.
    """
    h = hashlib.sha256()
    here = Path(__file__).resolve().parent
    for name in IMAGE_SOURCES:
        path = here / name
        h.update(name.encode())
        h.update(path.read_bytes() if path.is_file() else b"<missing>")
    for sub in IMAGE_SUBMODULES:
        h.update(sub.encode())
        try:
            out = subprocess.run(
                ["git", "-C", str(REPO_ROOT / sub), "rev-parse", "HEAD"],
                capture_output=True, text=True, timeout=10,
            )
            h.update(out.stdout.strip().encode() if out.returncode == 0 else b"<nogit>")
        except Exception:
            h.update(b"<nogit>")
    return h.hexdigest()[:16]


def image_label(tag: str) -> str | None:
    """The fingerprint stamped into an existing image, or None if absent."""
    result = subprocess.run(
        ["docker", "image", "inspect", "-f",
         f'{{{{index .Config.Labels "{FINGERPRINT_LABEL}"}}}}', tag],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        return None
    value = result.stdout.strip()
    return value if value and value != "<no value>" else None


def _run(cmd: list[str], **kwargs) -> subprocess.CompletedProcess:
    """Run a command, echoing it first so the user sees exactly what ran."""
    print(f"\033[0;34m$ {' '.join(cmd)}\033[0m", flush=True)
    return subprocess.run(cmd, **kwargs)


def preflight() -> None:
    if shutil.which("docker") is None:
        sys.exit("[run_quest_relay] ERROR: 'docker' not found on PATH. Install Docker first.")
    if not DOCKERFILE.is_file():
        sys.exit(f"[run_quest_relay] ERROR: Dockerfile not found at {DOCKERFILE}")
    if not RELAY_PY.is_file():
        sys.exit(f"[run_quest_relay] ERROR: relay.py not found at {RELAY_PY}")


def image_exists(tag: str) -> bool:
    result = subprocess.run(
        ["docker", "image", "inspect", tag],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def build_image(tag: str, fingerprint: str) -> None:
    print(f"[run_quest_relay] Building image '{tag}' (context: {REPO_ROOT})...")
    result = _run(
        ["docker", "build", "-t", tag,
         "--label", f"{FINGERPRINT_LABEL}={fingerprint}",
         "-f", str(DOCKERFILE), str(REPO_ROOT)],
    )
    if result.returncode != 0:
        sys.exit(f"[run_quest_relay] ERROR: docker build failed (exit {result.returncode}).")


def remove_stale_container(name: str) -> None:
    # Best-effort: ignore failure (container may not exist).
    subprocess.run(
        ["docker", "rm", "-f", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def stop_container(name: str, timeout: int = 30) -> None:
    """Stop the container, giving the entrypoint time to shut its children down.

    `docker stop` defaults to a 10s grace period before SIGKILL. That is enough
    for the relay, but a rosbag being finalised (index write + .active rename)
    can need longer, and a SIGKILL there leaves an unreadable bag — so allow more.
    """
    print(f"\n[run_quest_relay] Stopping container '{name}' (up to {timeout}s)...")
    subprocess.run(
        ["docker", "stop", "--time", str(timeout), name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def relay_args(args: argparse.Namespace) -> list[str]:
    """Args forwarded to relay.py inside the container (via entrypoint.sh)."""
    # Container-internal ZMQ port stays fixed; --zmq-port only remaps the host side.
    forwarded = ["--zmq-port", str(CONTAINER_ZMQ_PORT)]
    if args.head_topic is not None:
        forwarded += ["--head-topic", args.head_topic]
    if args.left_hand_topic is not None:
        forwarded += ["--left-hand-topic", args.left_hand_topic]
    if args.right_hand_topic is not None:
        forwarded += ["--right-hand-topic", args.right_hand_topic]
    if args.hz is not None:
        forwarded += ["--hz", str(args.hz)]
    return forwarded


def port_mappings(args: argparse.Namespace) -> list[str]:
    return [
        "-p", f"{args.tcp_port}:{CONTAINER_TCP_PORT}",
        "-p", f"{args.zmq_port}:{CONTAINER_ZMQ_PORT}",
    ]


def network_args(args: argparse.Namespace) -> list[str]:
    """Docker networking: either host networking (no NAT hop — preferred on the
    robot/Jetson) or explicit port mapping. With --network host the container
    binds directly on the host's interfaces, so 10000/5559 avoid the Docker-NAT
    hop and ``localhost`` inside the container reaches host services (e.g. the
    on-board camera server)."""
    if getattr(args, "network_host", False):
        return ["--network", "host"]
    return port_mappings(args)


def env_args(args: argparse.Namespace) -> list[str]:
    """Docker ``-e`` env vars. CAMERA_HOST gates the optional image relay, and
    RECORD_BAG / PLAY_BAG select the rosbag mode (entrypoint.sh branches on all
    three; each is unset by default)."""
    # getattr throughout: record_quest_data.py reuses these helpers with a
    # narrower argument namespace that has neither the camera nor the bag flags.
    env: list[str] = []
    if getattr(args, "camera_host", None) is not None:
        env += ["-e", f"CAMERA_HOST={args.camera_host}", "-e", f"CAMERA_PORT={args.camera_port}"]
        if args.image_fps is not None:
            env += ["-e", f"IMAGE_RELAY_FPS={args.image_fps}"]
    if getattr(args, "record_bag", None) is not None:
        env += ["-e", "RECORD_BAG=1"]
        if args.bag_prefix is not None:
            env += ["-e", f"BAG_PREFIX={args.bag_prefix}"]
    if getattr(args, "play_bag", None) is not None:
        # Only the basename crosses into the container; the parent directory is
        # what gets bind-mounted at /bags (see bag_mount_args).
        env += ["-e", f"PLAY_BAG={Path(args.play_bag).name}"]
        if getattr(args, "loop", False):
            env += ["-e", "BAG_LOOP=1"]
    return env


def bag_mount_args(args: argparse.Namespace) -> list[str]:
    """Bind-mount the bag directory at /bags for whichever rosbag mode is active.

    Recording mounts the target directory (created if missing) read-write;
    playback mounts the bag's parent directory read-only, so a replay can never
    clobber the recording it is reading.
    """
    record_dir = getattr(args, "record_bag", None)
    play_bag = getattr(args, "play_bag", None)
    if record_dir is not None:
        host_dir = Path(record_dir).expanduser().resolve()
        host_dir.mkdir(parents=True, exist_ok=True)
        return ["-v", f"{host_dir}:/bags"]
    if play_bag is not None:
        bag_path = Path(play_bag).expanduser().resolve()
        if not bag_path.is_file():
            sys.exit(f"[run_quest_relay] ERROR: bag not found: {bag_path}")
        return ["-v", f"{bag_path.parent}:/bags:ro"]
    return []


def wait_for_zmq(port: int, timeout: float = 60.0) -> bool:
    """Poll the host ZMQ port until the relay is accepting connections."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=1.0):
                return True
        except OSError:
            time.sleep(0.5)
    return False


def print_next_steps(args: argparse.Namespace) -> None:
    playing = getattr(args, "play_bag", None) is not None
    print("\n" + "=" * 72)
    print("[run_quest_relay] Relay is up.")
    if playing:
        print(f"  - Bag mounted: {args.play_bag} (no Quest needed)")
        print("  - The manager starts playback on its 2nd 's'.")
    print("  - Start the teleop manager with:")
    print(
        "      python -m teleop_manager.quest_manager "
        f"--relay-host localhost --relay-port {args.zmq_port}"
        + ("  --no-robot" if playing else "")
    )
    print("=" * 72 + "\n")


def run_detached(tag: str, args: argparse.Namespace) -> int:
    cmd = ["docker", "run", "-d", "--rm", "--name", args.name]
    cmd += [*network_args(args), *env_args(args), *bag_mount_args(args), tag, *relay_args(args)]
    result = _run(cmd)
    if result.returncode != 0:
        return result.returncode
    print(
        f"[run_quest_relay] Container '{args.name}' started (detached). "
        f"Waiting for ZMQ port {args.zmq_port}..."
    )
    if wait_for_zmq(args.zmq_port):
        print_next_steps(args)
        print(f"[run_quest_relay] Follow logs with:  docker logs -f {args.name}")
        print(f"[run_quest_relay] Stop with:         docker stop {args.name}")
        return 0
    print(
        f"[run_quest_relay] WARNING: ZMQ port {args.zmq_port} did not come up in time. "
        f"Check 'docker logs {args.name}'."
    )
    return 1


def run_attached(tag: str, args: argparse.Namespace) -> int:
    cmd = ["docker", "run", "--rm", "--name", args.name]
    cmd += [*network_args(args), *env_args(args), *bag_mount_args(args), tag, *relay_args(args)]
    print(f"\033[0;34m$ {' '.join(cmd)}\033[0m", flush=True)
    print_next_steps(args)
    print("[run_quest_relay] Starting relay (Ctrl-C to stop)...\n")

    proc = subprocess.Popen(cmd)

    stopping = {"flag": False}

    def handle_signal(signum, frame):
        if stopping["flag"]:
            return
        stopping["flag"] = True
        stop_container(args.name)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        return proc.wait()
    finally:
        # Ensure teardown even if signal forwarding to `docker run` was flaky.
        if not stopping["flag"]:
            stop_container(args.name)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build and run the Quest → ZMQ relay container.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--image-tag", default="quest-relay:latest", help="Docker image tag.")
    parser.add_argument("--name", default="quest-relay", help="Container name.")
    parser.add_argument(
        "--tcp-port", type=int, default=10000,
        help="Host port for the ROS-TCP endpoint (Quest connects here).",
    )
    parser.add_argument(
        "--zmq-port", type=int, default=5559,
        help="Host port the manager subscribes to for relayed quest_data.",
    )
    parser.add_argument(
        "--network-host", action="store_true",
        help="Use Docker host networking instead of port mapping (preferred on the "
             "robot: no NAT hop, and 'localhost' reaches the on-board camera server).",
    )
    parser.add_argument(
        "--rebuild", action="store_true", help="Force rebuild even if the image exists."
    )
    parser.add_argument(
        "--no-build", action="store_true", help="Never build; fail if the image is missing."
    )
    parser.add_argument(
        "--detach", action="store_true",
        help="Run detached, wait for ZMQ readiness, then return.",
    )
    # Forwarded to relay.py inside the container.
    parser.add_argument(
        "--head-topic", default=None, help="ROS1 head pose topic (relay default if unset)."
    )
    parser.add_argument("--left-hand-topic", default=None, help="ROS1 left ManoLandmarks topic.")
    parser.add_argument("--right-hand-topic", default=None, help="ROS1 right ManoLandmarks topic.")
    parser.add_argument("--hz", type=float, default=None, help="Relay publish rate in Hz.")
    # Optional robot ego-view -> Quest image relay (see image_relay.py). Passing
    # --camera-host starts it inside the container; it must be an address the
    # container can reach (the robot's IP, or host.docker.internal for a server
    # on this same host).
    parser.add_argument(
        "--camera-host", default=None,
        help="Enable the ego-view image relay, subscribing to the camera ZMQ server at this host.",
    )
    parser.add_argument("--camera-port", type=int, default=5555, help="Camera server ZMQ port.")
    parser.add_argument(
        "--image-fps", type=float, default=None, help="Max image relay publish rate (default 30)."
    )
    # rosbag record / playback. Recording taps the four ROS topics the relay
    # consumes; playback feeds them back into an unmodified relay, so the manager
    # sees the same stream it would live. Mutually exclusive.
    bag = parser.add_mutually_exclusive_group()
    bag.add_argument(
        "--record-bag", default=None, metavar="DIR",
        help="Record the Quest ROS topics to a rosbag in this host directory "
             "(created if missing). Live mode only.",
    )
    bag.add_argument(
        "--play-bag", default=None, metavar="BAG",
        help="Replay this rosbag instead of starting the Quest endpoint (off-robot). "
             "Pair with the manager's --no-robot.",
    )
    parser.add_argument(
        "--bag-prefix", default=None,
        help="Filename prefix for --record-bag (default 'quest'); a timestamp is appended.",
    )
    parser.add_argument(
        "--loop", action="store_true", help="With --play-bag, loop the bag forever."
    )
    args = parser.parse_args()

    if args.loop and args.play_bag is None:
        parser.error("--loop only applies to --play-bag")
    if args.bag_prefix is not None and args.record_bag is None:
        parser.error("--bag-prefix only applies to --record-bag")
    # Check the bag up front: bag_mount_args() would not run until after the image
    # build, so a mistyped path would otherwise cost a full build first.
    if args.play_bag is not None and not Path(args.play_bag).expanduser().is_file():
        parser.error(f"--play-bag: no such file: {args.play_bag}")

    preflight()

    fingerprint = image_fingerprint()
    have_image = image_exists(args.image_tag)
    stale = have_image and image_label(args.image_tag) != fingerprint
    if args.rebuild or not have_image or stale:
        if args.no_build:
            if not have_image:
                sys.exit(
                    f"[run_quest_relay] ERROR: image '{args.image_tag}' not found and --no-build was given."
                )
            if stale:
                print(
                    "[run_quest_relay] WARNING: image is out of date with quest_bridge/ "
                    "or the ROS submodules, but --no-build was given. Running it anyway."
                )
        else:
            if stale and not args.rebuild:
                print(
                    "[run_quest_relay] Image is out of date (quest_bridge sources or the "
                    "ROS submodules changed since it was built) — rebuilding."
                )
            build_image(args.image_tag, fingerprint)
    else:
        print(
            f"[run_quest_relay] Reusing existing image '{args.image_tag}' (up to date)."
        )

    remove_stale_container(args.name)

    if args.detach:
        return run_detached(args.image_tag, args)
    return run_attached(args.image_tag, args)


if __name__ == "__main__":
    sys.exit(main())
