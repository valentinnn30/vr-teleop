> **Note (repo split):** this doc was copied from the `GR00T-WholeBodyControl` fork's
> `robot-deploy` branch as a starting reference. Since that split, `install_scripts/`,
> `scripts/launch_*.sh`, and the teleop/data-collection Python entry points referenced below
> now live in **this** repo (`vr-teleop`) instead — see `pyproject.toml` for the editable
> `gear_sonic[teleop]`/`[data_collection]` dependency this repo takes on a GR00T checkout (with
> the BrainCo kinematic patch applied) for calibration and dataset schema. Everything about the
> SONIC deploy binary, the deploy container, and the BrainCo/hand_config C++ patch stays on the
> GR00T-repo side. This doc has not yet been fully re-split between the two repos — treat paths
> below as pointers to verify, not guaranteed-correct as written. See
> `docs/INTERFACE_CONTRACT.md` for the ZMQ contract between the two repos.

# G1 + BrainCo + Quest teleop — quickstart

**Body motion last ran on hardware in June; everything since is sim-tested only.**

**Test before in simulation the setup.**

Everything real-time runs on the robot. Quest is cabled to the robot
(`192.168.77.x`), laptop is cabled to the robot (`192.168.123.x`) and only
records datasets.

Upstream install docs:
`docs/source/getting_started/installation_deploy.md`, `download_models.md`.

## Get the code (laptop and robot)

```bash
git clone git@github.com:ronan-lebas/GR00T-WholeBodyControl.git
cd GR00T-WholeBodyControl
git checkout robot-deploy
git submodule update --init --recursive
git lfs pull
```

## Install — robot (Jetson)

```bash
# checkpoints (NOT in git, not in LFS)
uv run --no-project --with huggingface_hub python download_from_hf.py

# venvs
bash install_scripts/install_data_collection.sh   # .venv_data_collection
bash install_scripts/install_pico.sh              # .venv_teleop

# deploy container (CUDA/TensorRT/ONNX live inside it — nothing to install)
gear_sonic_deploy/docker/run-ros2-dev.sh --host-net    # exit once it comes up

# BrainCo hand bridge (needs unitree_sdk2 system-wide)
cd gear_sonic_deploy/thirdparty/brainco_hand_service
mkdir -p build && cd build && cmake .. && make -j6
```

Check the hands:

```bash
cd gear_sonic_deploy/thirdparty/brainco_hand_service/bin
sudo ./brainco_hand_server -n <192.168.123.x iface>
sudo ./test_brainco_hand_server left
sudo ./test_brainco_hand_server right
```

## Install — laptop

```bash
bash install_scripts/install_data_collection.sh

sudo nmcli connection add type ethernet ifname <iface> con-name robot \
    ipv4.method manual ipv4.addresses 192.168.123.222/24
sudo nmcli connection up robot
ping -c3 192.168.123.164
```

## Run

**Robot**, once per boot:

```bash
./scripts/launch_robot_side.sh wire      # Quest link: DHCP+NAT on 192.168.77.1
```

Quest app → `192.168.77.1:10000`.

**Robot**:

```bash
ROBOT_IFACE=<192.168.123.x iface> ./scripts/launch_robot_side.sh
```

tmux `g1_robot`: hand → camera → deploy → relay → manager. The deploy pane starts
the container and types the launch command, then stops at
`Proceed with deployment? [Y/n]:` — **type `y`**.

**Laptop**:

```bash
export ROBOT_IP=192.168.123.164
TASK_PROMPT="pick up the box" DATASET_NAME=session1 ./scripts/launch_laptop_side.sh
```


| Key | |
|---|---|
| `s` | 1st: start + ramp to calibration pose. 2nd: countdown → teleop |
| `r` | recalibrate (ramps back to reference pose first) |
| `p` | pause / resume |
| `f` | fingers on/off |
| `c` / `x` | record episode start-stop / abort |
| `-` `=` | crouch / stand (works even with crouch tracking off) |
| `q` | stop |
| `b` `0` | **sim only — never on hardware** |

Stop: `q`, then Ctrl-C the deploy pane.

**E-stop is "o" on the deploy pane**.  
**You need to Ctrl-C the recorder pane to save the data, killing the tmux window will discard the data.**

## Useful env vars

| Var | Default | |
|---|---|---|
| `ROBOT_IFACE` | auto | hand service iface — must match deploy's |
| `EGO_VIEW_CAMERA` | `realsense` | `oak` (+ `OAK_SERIAL`), `usb` |
| `DEPLOY_EXTRA` | — | `--cp policy/sonic_v1_1/model --obs-config policy/sonic_v1_1/observation_config.yaml` (for the other SONIC checkpoints) |
| `ROBOT_IP` | `192.168.123.164` | laptop side |

## First session

The default is **arms and hands only** — no walking, no turn-in-place, no crouch.

```
(default)                        # arms/hands only
MANAGER_EXTRA="--disable-walk"   # + turn in place
MANAGER_EXTRA=""                 # + walking
MANAGER_EXTRA="--enable-crouch"  # + head-driven crouch (full motion)
```

Test in sim first: `./scripts/launch_sim_setup.sh` (`--mock-quest` or
`--replay-quest` if no headset). (Needs to install `.venv_sim` with `bash install_scripts/install_mujoco_sim.sh`)

## To test the recordings

They are stored under `outputs/`.  
Visualize them with `source .venv_sim/bin/activate && python gear_sonic/scripts/visualize_robot_object_trajectory.py`.  
Options are described in `source .venv_sim/bin/activate && python gear_sonic/scripts/visualize_robot_object_trajectory.py --help`.

Check some joints recording with `python gear_sonic/scripts/visualize_recording.py`.


## Troubleshooting

| Symptom | Fix |
|---|---|
| Arms move, fingers dead | hand service down or on a different iface than deploy — set `ROBOT_IFACE` |
| Never leaves `WAIT_FOR_CONTROL` | press `s`; check the manager pane is connected to the relay |
| `Missing file` at deploy start | `python download_from_hf.py` |
| Robot ignores DDS | deploy banner `Resolved interface:` must be the `192.168.123.x` port, not the Quest wire |
| `name already in use` | `docker rm -f g1-deploy-dev quest-relay` |
| Slow first start | TensorRT engine build — expected once |
