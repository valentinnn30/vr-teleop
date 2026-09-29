# G1 + BrainCo + Quest teleop — quickstart

**Body motion last ran on hardware in June; everything since has only been
tested off-hardware.**

Everything real-time runs on the robot: BrainCo hand service, the SONIC deploy
binary, the Quest relay, and the teleop manager. The Quest reaches the relay over
shared WiFi. A laptop is only needed to `ssh` in and drive the manager keyboard.

This repo does the VR side. The deploy binary lives in a **separate
GR00T-WholeBodyControl checkout**, reached over ZMQ and pointed at by
`$GROOT_REPO`. See `../CLAUDE.md` for the boundary and
`INTERFACE_CONTRACT.md` for the wire format.

## Prerequisites

### This repo (robot, and laptop if you want the replay loop)

```bash
git clone --recurse-submodules <this repo>
cd vr-teleop
export GROOT_REPO=$HOME/GR00T-WholeBodyControl   # wherever you cloned it
bash install_scripts/install_teleop.sh           # creates .venv_teleop
```

The install script verifies every import the manager makes at startup, so if it
finishes clean the manager will start.

Shortcut if the GR00T setup is already done on this machine: its `.venv_teleop`
already has `gear_sonic[teleop]` and `brainco-retargeting`, which is the complete
dependency set, so you can skip the install entirely with
`TELEOP_VENV=$GROOT_REPO/.venv_teleop`. The launcher `cd`s to this repo's root
before running `python -m teleop_manager.quest_manager`, so the packages here are
importable from the working directory without being pip-installed.

Nothing else needs installing. The deploy container and the Quest relay image both
build themselves on first use.

### BrainCo hand service (robot only, C++)

**If you already followed the GR00T robot setup, this is done** — it builds the
same submodule at
`$GROOT_REPO/gear_sonic_deploy/thirdparty/brainco_hand_service/bin`, and
`launch_robot_side.sh` falls back to that build automatically when this repo's
`third_party/.../bin` is empty. The submodule is the same commit in both repos, so
there is nothing to rebuild. `./scripts/launch_robot_side.sh hand --print` shows
which directory it picked.

To build it here instead: needs `unitree_sdk2` and CycloneDDS installed
**system-wide under `/usr/local`** (bare `link_libraries`, no `find_package`), plus
`libboost-all-dev libyaml-cpp-dev libfmt-dev libspdlog-dev`.

```bash
cd third_party/brainco_hand_service
mkdir -p build && cd build && cmake .. && make -j6   # binaries -> ../bin/
```

Check the hands before involving the policy:

```bash
cd <hand bin dir>
sudo ./brainco_hand_server -n <192.168.123.x iface>
sudo ./test_brainco_hand_server left
sudo ./test_brainco_hand_server right
```

### GR00T side (robot only)

Two things this repo cannot do for you:

```bash
cd $GROOT_REPO
uv run --no-project --with huggingface_hub python download_from_hf.py
# ONNX checkpoints -> gear_sonic_deploy/policy/release/ and planner/target_vel/V2/
# NOT in git, NOT in LFS. deploy.sh only WARNS if they are missing; the binary
# then fails later.
```

and `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/hand_config.hpp` must
have `#define USE_BRAINCO_HANDS 1` before the build, or the deploy ignores the
hands entirely.

## Run

```bash
./scripts/launch_robot_side.sh
```

One tmux session (`g1_robot`), four tiled panes, started in order:
**hand → deploy → relay → manager**.

The deploy pane brings up the GR00T container, waits for its `Relays active`
banner, types the deploy command in, and then stops at
`Proceed with deployment? [Y/n]:` — **type `y`**. That is the last gate before the
robot is commanded.

Point the Quest Unity app at `<robot-ip>:10000`.

Check the wiring first without starting anything:

```bash
./scripts/launch_robot_side.sh --print          # all four commands
./scripts/launch_robot_side.sh manager --print  # just one
```

### Manager keys

| Key | |
|---|---|
| `s` | 1st: start + ramp to calibration pose. 2nd: countdown → teleop |
| `r` | recalibrate (ramps back to the reference pose first) |
| `p` | pause / resume |
| `f` | fingers on/off |
| `c` / `x` | record episode start-stop / abort |
| `-` `=` | crouch / stand (works even with crouch tracking off) |
| `q` | stop |
| `b` `0` | **sim only — never on hardware** |

Stop with `q`, then Ctrl-C the deploy pane. **E-stop is `o` in the deploy pane.**

### Motion scope

The default is **arms and hands only** — no walking, no turn-in-place, no crouch.

```
(default)                        # arms/hands only  (--static-base)
MANAGER_EXTRA="--disable-walk"   # + turn in place
MANAGER_EXTRA=""                 # + walking
MANAGER_EXTRA="--enable-crouch"  # + head-driven crouch (full motion)
```

### Env vars

| Var | Default | |
|---|---|---|
| `GROOT_REPO` | `$HOME/GR00T-WholeBodyControl` | external deploy checkout |
| `ROBOT_IFACE` | `enP8p1s0` | hand service `-n` — must match deploy's. Empty = SDK default |
| `MANAGER_EXTRA` | `--static-base` | motion scope, see above |
| `DEPLOY_EXTRA` | — | e.g. `--cp policy/sonic_v1_1/model --obs-config policy/sonic_v1_1/observation_config.yaml` |
| `BAG_DIR` | — | set to record a Quest rosbag while teleoperating |
| `LATENCY_CSV` | — | set to write a per-frame latency trace |
| `HAND_DIR` | this repo's build, else the GR00T one | use a specific hand-service build |
| `TELEOP_VENV` | `.venv_teleop` | reuse another venv, e.g. `$GROOT_REPO/.venv_teleop` |
| `PLAY_BAG` / `BAG_LOOP` | — | drive the robot from a recording, no headset |
| `HAND_USE_SYSTEMD` | `0` | `1` = `systemctl restart brainco_hand.service` |

## Off-robot replay

Record while teleoperating:

```bash
BAG_DIR=$HOME/bags ./scripts/launch_robot_side.sh
```

Stop the relay pane with **Ctrl-C, not `docker kill`** — `rosbag record` finalises
its file only on SIGINT. A leftover `*.bag.active` means it was truncated.

Then, on a laptop with no robot and no headset:

```bash
./scripts/launch_replay.sh --bag ~/bags/quest_20260929_120000.bag [--loop]
```

This replays the bag through the **real relay container**, so the relay, its
msgpack encoding and the ZMQ hop are all exercised — the manager cannot tell it
from live. Press `s` to calibrate and start.

There is also a no-Docker path that enters at the manager's input:

```bash
python3 quest_bridge/generate_mock_quest_data.py      # synthetic trajectories
./scripts/launch_replay.sh --npz data/quest/traj_20260929_120000_000.npz
```

Both write a per-frame latency CSV (`outputs/latency_<timestamp>.csv` by default).

## Driving the robot from a recording (no headset)

Useful as a repeatable hardware test: the bag supplies the ROS topics the Quest
would have, so relay/msgpack/ZMQ stay in the loop and latency is representative.

```bash
PLAY_BAG=~/bags/quest_20260929_120000.bag BAG_LOOP=1 ./scripts/launch_robot_side.sh
```

The manager stays in **normal live mode** here — you still get the two-press ramp
from the robot's measured pose, and the deploy is fully in the loop. Time the
**second** `s` to a moment when the recording is at the operator's rest pose:
calibration samples whatever frame is playing, and `rosbag play` cannot be rewound
by the manager (`BAG_LOOP=1` gives you repeated chances).

**Do not use `--replay` (NPZ) against a real robot.** It sets the manager offline,
which skips the ramp — the robot would snap to the first commanded target from
whatever pose it is in. NPZ replay is for off-robot work only.

**What the CSV can and cannot tell you.** It covers
relay → manager → retarget → send. It does **not** include the Quest → robot hop:
the Unity app sends no capture timestamp and the ROS-TCP-Endpoint fork restamps on
arrival, so every timestamp starts at or after the robot. A small `age_recv_ms`
(single-digit ms) means the robot-side pipeline is fine and the remaining lag is in
the Quest link — which needs a Unity-side change to measure. See `../CLAUDE.md`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Arms move, fingers dead | hand service down, or on a different iface than deploy — set `ROBOT_IFACE` |
| Never leaves `WAIT_FOR_CONTROL` | press `s`; check the manager pane connected to the relay on 5559 |
| Manager sits at "Waiting for g1_debug robot feedback" | the deploy is not in CONTROL — answer `y` at its prompt. Off-robot, use `--no-robot` |
| `Missing file` at deploy start | `python download_from_hf.py` in `$GROOT_REPO` |
| Robot ignores DDS | the deploy banner's `Resolved interface:` must be the `192.168.123.x` port |
| `name already in use` | `docker rm -f g1-deploy-dev quest-relay` |
| Slow first start | TensorRT engine build — expected once |
| `GR00T checkout not usable` | `export GROOT_REPO=<path>` |
| Recorded bag is `*.bag.active` | the relay was killed, not Ctrl-C'd — the bag is truncated |
