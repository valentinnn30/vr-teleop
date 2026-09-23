#!/usr/bin/env python3
"""
Host-side launcher for the Quest → ZMQ relay container.

Automates the full lifecycle so you don't have to run docker by hand:
build the image, run the container (with the right port mappings), stream
its logs, and tear it down cleanly on Ctrl-C.

The container runs *both* the ROS-TCP endpoint (Quest connects here over the
Unity ROS-TCP-Connector protocol, launched via roslaunch on a ROS1/Noetic
master) and the ZMQ relay that republishes the tracking data as a single
msgpack ``quest_data`` blob. The host-side ``quest_manager_thread_server.py``
consumes that blob with ``--zmq-relay-host`` (no ROS needed on the host).

Data flow:
    Quest (Unity) --TCP:10000--> ros_tcp_endpoint --TCPROS--> relay --ZMQ:5559--> manager

Usage (from anywhere in the repo):
    python gear_sonic_deploy/docker/quest_relay/run_quest_relay.py
    python gear_sonic_deploy/docker/quest_relay/run_quest_relay.py --rebuild
    python gear_sonic_deploy/docker/quest_relay/run_quest_relay.py --detach

Then, in another shell:
    python gear_sonic/scripts/quest_manager_thread_server.py --zmq-relay-host localhost
"""

import argparse
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

# quest_relay -> docker -> gear_sonic_deploy -> <repo root>
REPO_ROOT = Path(__file__).resolve().parents[3]
DOCKERFILE = Path(__file__).resolve().parent / "Dockerfile"
RELAY_PY = Path(__file__).resolve().parent / "relay.py"

# Container-internal ports (fixed; see Dockerfile EXPOSE and relay.py defaults).
CONTAINER_TCP_PORT = 10000
CONTAINER_ZMQ_PORT = 5559


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


def build_image(tag: str) -> None:
    print(f"[run_quest_relay] Building image '{tag}' (context: {REPO_ROOT})...")
    result = _run(
        ["docker", "build", "-t", tag, "-f", str(DOCKERFILE), str(REPO_ROOT)],
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


def stop_container(name: str) -> None:
    print(f"\n[run_quest_relay] Stopping container '{name}'...")
    subprocess.run(
        ["docker", "stop", name],
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
    """Docker ``-e`` env vars. CAMERA_HOST gates the optional image relay
    (entrypoint.sh starts image_relay.py only when it is set)."""
    if args.camera_host is None:
        return []
    env = ["-e", f"CAMERA_HOST={args.camera_host}", "-e", f"CAMERA_PORT={args.camera_port}"]
    if args.image_fps is not None:
        env += ["-e", f"IMAGE_RELAY_FPS={args.image_fps}"]
    return env


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
    print("\n" + "=" * 72)
    print("[run_quest_relay] Relay is up.")
    print(f"  - Point the Quest Unity app at:  <this-host>:{args.tcp_port}")
    print("  - Start the teleop manager with:")
    print(
        "      python gear_sonic/scripts/quest_manager_thread_server.py "
        f"--zmq-relay-host localhost --zmq-relay-port {args.zmq_port}"
    )
    print("=" * 72 + "\n")


def run_detached(tag: str, args: argparse.Namespace) -> int:
    cmd = ["docker", "run", "-d", "--rm", "--name", args.name]
    cmd += [*network_args(args), *env_args(args), tag, *relay_args(args)]
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
    cmd += [*network_args(args), *env_args(args), tag, *relay_args(args)]
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
    args = parser.parse_args()

    preflight()

    have_image = image_exists(args.image_tag)
    if args.rebuild or not have_image:
        if args.no_build:
            if not have_image:
                sys.exit(
                    f"[run_quest_relay] ERROR: image '{args.image_tag}' not found and --no-build was given."
                )
        else:
            build_image(args.image_tag)
    else:
        print(
            f"[run_quest_relay] Reusing existing image '{args.image_tag}' (pass --rebuild to force)."
        )

    remove_stale_container(args.name)

    if args.detach:
        return run_detached(args.image_tag, args)
    return run_attached(args.image_tag, args)


if __name__ == "__main__":
    sys.exit(main())
