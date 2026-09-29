# vr-teleop — working notes

VR teleoperation for the Unitree G1 + BrainCo hands: Meta Quest → ROS1 → ZMQ →
an external SONIC deploy process. This file records the decisions and gotchas that
are **not** derivable from the code.

## Scope boundary

This repo does **VR teleop and retargeting only**, and aims to stay robot-agnostic.
Everything robot- or policy-specific is called externally:

| Concern | Where it lives |
|---|---|
| Quest → ZMQ bridge, retargeting, teleop manager | here |
| SONIC deploy binary, policy, ONNX checkpoints | external GR00T checkout (`$GROOT_REPO`) |
| Robot kinematics (FK for calibration) | `gear_sonic` Python package, from that same checkout |
| ROS-TCP-Endpoint, `vr_haptic_msgs`, BrainCo hand service, brainco-retargeting | `third_party/` submodules |

`$GROOT_REPO` (default `$HOME/GR00T-WholeBodyControl`) is the single knob for the
external checkout — used by `scripts/launch_robot_side.sh` and
`install_scripts/install_teleop.sh`. It is **not** a pyproject dependency: PEP 508
URLs cannot interpolate env vars, so a `file://` path would only ever be valid on
one machine. The install script installs `gear_sonic[teleop]` from `$GROOT_REPO`
explicitly.

## Running it

```bash
bash install_scripts/install_teleop.sh      # .venv_teleop (robot and laptop)
./scripts/launch_robot_side.sh              # tmux: hand -> deploy -> relay -> manager
./scripts/launch_robot_side.sh --print      # dry run, prints every command
```

`ROBOT_IFACE` defaults to `enP8p1s0` (the Jetson's onboard NIC). It feeds the hand
service's DDS `-n` and **must match the interface the deploy binary resolves** —
a mismatch is the usual cause of "arms move, fingers dead", so it is pinned rather
than left to the SDK's pick. `ROBOT_IFACE=` (empty) opts back out.

Venvs are independent directories, so this repo's `.venv_teleop` and the GR00T
checkout's coexist; `TELEOP_VENV=<path>` switches between them for A/B testing.
Note that building this repo's own venv does **not** decouple from GR00T — the
manager still imports `gear_sonic` for FK. See the fork migration section.

The deploy pane starts the GR00T container, waits for the literal string
`Relays active` (its last line before the interactive shell), types the deploy
command in, and stops at `Proceed with deployment? [Y/n]:` — **the operator types
`y`**. That prompt is the last gate before the robot is commanded.

Off-robot, with no robot and no headset:

```bash
./scripts/launch_replay.sh --bag bags/quest_*.bag   # through the real relay
./scripts/launch_replay.sh --npz data/quest/*.npz   # straight into the manager
```

## Port map

| Port | Direction | Topics | Notes |
|---|---|---|---|
| 5556 | manager → deploy | `command`, `planner`, `manager_state` | manager **binds**, deploy connects SUB |
| 5557 | deploy → manager | `g1_debug` (`body_q_measured`) | deploy **binds**, manager polls |
| 5559 | relay → manager | `quest_data` (msgpack) | internal to this repo |
| 10000 | Quest → relay | ROS-TCP-Connector | the Unity app targets `<robot>:10000` |

`--host-net` on the deploy container is mandatory on the real robot: in bridge
mode only 5557 is port-mapped and **5556 never is**, so the deploy would never
receive commands.

## Latency: what is measurable and what is not

**The Quest → robot hop cannot be measured from a recording.** The Unity app sends
zero ROS header stamps; the leggedrobotics ROS-TCP-Endpoint fork works around that
(commit `0c70ce6`, "temp fix for headers being 0") by restamping with the *robot-side
receipt* time, and `relay.py` ignores header stamps entirely in favour of
`time.time()` at callback receipt. So every timestamp in the system begins at or
after arrival on the robot.

`--latency-csv` traces relay → manager → retarget → send per frame. Relay and
manager are always co-located (both on the Jetson live; both on the laptop under
bag playback), so those wall-clock deltas are free of clock skew. A small
`age_recv_ms` means the robot-side pipeline is healthy and the remaining lag is in
the Quest link — which needs a **Unity-side capture stamp** to see, outside this
repo.

## Replay modes, and which are safe on hardware

Three ways to run the manager without a live Quest. They differ in whether the
**pre-teleop ramp** happens, which is a hardware-safety property, not a
convenience:

| Mode | Source | Ramp | Safe on hardware? |
|---|---|---|---|
| `--replay <npz>` | `ReplaySource` | **skipped** | **No** |
| `--play-bag` relay + manager `--no-robot` | live relay, no deploy | skipped | n/a (no robot) |
| `--play-bag` relay + normal manager | live relay, real deploy | normal two-press | **Yes** |

The ramp exists because `body_q_measured` (5557, published only while the deploy
is in CONTROL) is the only way to know the robot's current pose; without it the
manager "cannot ramp safely — the robot's current pose is unknown, so any target
we stream would snap it" (`_run_ramp` docstring). `--replay` and `--no-robot` both
set `self.offline`, which bypasses that gate deliberately — correct off-robot,
dangerous with a real robot attached.

So: **drive hardware from a recording via `PLAY_BAG=<bag> ./scripts/launch_robot_side.sh`**,
which keeps the manager in its normal live mode with the deploy in the loop. Never
point `--replay` at a live robot.

One asymmetry to know: NPZ replay calls `source.restart()` when you calibrate, so
playback and calibration are synchronised and `rest_hold_sec`/`rest_interp_sec`
synthesise a rest prefix. **Bag playback cannot be rewound** — `rosbag play` is an
external process the manager has no handle on — so calibration samples whatever
frame happens to be playing. Time the second `s` to a rest-pose moment, and use
`BAG_LOOP=1` so you get repeated chances.

## Gotchas worth not rediscovering

**`set -m` in `quest_bridge/entrypoint.sh` is load-bearing.** POSIX has a
non-interactive shell start background jobs with SIGINT set to `SIG_IGN`, and bash
cannot trap a signal that was ignored at startup. Without job control, `kill -INT`
at shutdown is silently discarded by every backgrounded child — and
`rosbag record` finalises its file (index flush, `.bag.active` → `.bag` rename)
*only* from the SIGINT handler ROS installs. It does not handle SIGTERM, so
signalling TERM instead would truncate every recording. This silently corrupts
100% of bags if removed.

**The shutdown trap must `exit`.** A bash trap interrupts the current command and
then *resumes* the script. Without the explicit `exit`, a signal arriving during
startup tears the children down and then carries on to launch `rosbag record` and
the relay anyway, leaving a half-started container that ignores the stop it was
just given.

**Don't `wait` or `kill -0` in that trap.** `wait` inside a bash signal handler
hangs, and `kill -0` reports a zombie as alive, so process polling never
terminates early. The trap polls for `/bags/*.active` disappearing instead — the
condition actually cared about.

**`MANAGER_EXTRA="${MANAGER_EXTRA---static-base}"` has no colon on purpose.** With
`:-`, an explicitly empty value would silently re-apply the default; the launcher
needs `MANAGER_EXTRA=""` to mean "full motion, no flags".

**`third_party/vr_haptic_msgs` builds under catkin directly.** Upstream is
dual-build (`package.xml` format 3 with `condition="$ROS_VERSION == 1"`, and a
`CMakeLists.txt` that branches on `$ENV{ROS_VERSION}`), and the Dockerfile's
`. /opt/ros/noetic/setup.sh` sets `ROS_VERSION=1`. The repo previously carried a
hand-written catkin wrapper around vendored copies of the same four `.msg` files;
it was redundant and has been deleted.

## Deliberately out of scope right now

- **Camera** (ego-view server + the Quest image relay). The relay's image relay
  only starts when `CAMERA_HOST` is set, so omitting the flags disables it for
  free.
- **Quest wire link.** Shared WiFi works; there is no `wire` component.
- **Sim.** Not being sim-tested.
- **Robot-state / LeRobot dataset recording.** `data_collection/run_data_exporter.py:291`
  constructs `ComposedCameraClientSensor` unconditionally — there is a
  `--skip-robot-state` but no `--skip-camera`, so enabling this needs either the
  camera component back or a new skip path through the exporter and the LeRobot
  video schema.

## Planned: drop the GR00T fork

Intended direction, with the scope correctly sized: **the foundation model already
comes from upstream.** `download_from_hf.py` pulls the public HF repo
`nvidia/GEAR-SONIC`, so this is not a checkpoint swap. What is fork-local is:

- the deploy binary's BrainCo support — `hand_config.hpp` (`#define USE_BRAINCO_HANDS 1`)
  and `brainco_hands.hpp`
- `--input-type zmq_manager` and the 5556/5557 wire format in `g1_deploy_onnx_ref`
- `--brainco-thumb-swap` (applied automatically for the `real` target)
- the `gear_sonic` Python package
- the BrainCo kinematic patch: `gear_sonic/data/robot_model/model_data/g1/with_brainco/`
  (URDF/XML + `g1_supplemental_info.py`; no meshes needed) so that
  `instantiate_g1_robot_model(hand_type="brainco")` resolves

Dropping the fork means re-adding those to upstream. Its own phase.

## Known deviation from robot-agnosticism

`teleop_manager/quest_manager.py:251` hardcodes
`instantiate_g1_robot_model(hand_type="brainco")`, and the key-frame offsets come
from `teleop_manager/vis/vr3pt_pose_visualizer.py` (`G1_KEY_FRAME_OFFSETS`). That
is the seam to cut when a second robot or hand appears. `retargeting/hand/` is
already clean — it depends only on numpy and `brainco_retargeting`, with no
robot-body dependency, so a different hand is a new module there rather than a
manager rewrite.

## GR00T-side prerequisites this repo cannot do

1. `python download_from_hf.py` — ONNX checkpoints, not in git and not in LFS.
   They land in `gear_sonic_deploy/policy/release/` and
   `gear_sonic_deploy/planner/target_vel/V2/`, matching `deploy.sh`'s defaults.
   `deploy.sh` only *warns* about missing files; the binary then fails later.
2. `hand_config.hpp` with `#define USE_BRAINCO_HANDS 1`, then rebuild — otherwise
   the deploy ignores the hands entirely.
3. `unitree_sdk2` + CycloneDDS installed system-wide under `/usr/local` for the
   BrainCo hand service build (bare `link_libraries`, no `find_package`). This repo
   has no `unitree_sdk2` copy; the GR00T checkout vendors one.
