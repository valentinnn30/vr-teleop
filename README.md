# vr-teleop

VR teleoperation stack for the G1 + BrainCo hands project: Meta Quest bridge, hand
retargeting, data collection/recording, and BrainCo hand control. No body-control or
simulation code — this repo talks to a separate GR00T-WholeBodyControl checkout's SONIC
deploy process over ZMQ.

```bash
export GROOT_REPO=$HOME/GR00T-WholeBodyControl
bash install_scripts/install_teleop.sh
./scripts/launch_robot_side.sh          # tmux: hand -> deploy -> relay -> manager
```

Docs: `CLAUDE.md` (boundary, port map, gotchas), `docs/ROBOT_TELEOP_QUICKSTART.md`
(setup/run), `docs/TESTING_SCRIPT.md` (post-install bring-up, in order),
`docs/INTERFACE_CONTRACT.md` (the ZMQ contract with the GR00T repo).

## Layout

- `quest_bridge/` — ROS1↔ZMQ relay between the Quest headset (via ROS-TCP-Endpoint) and
  `teleop_manager/`, packaged as a Docker image. Also rosbag record/replay. Robot-agnostic.
- `retargeting/hand/` — MANO-21 landmarks → normalized hand-motor commands. BrainCo today
  (`third_party/brainco-retargeting`); a different hand later is a new module here.
- `teleop_manager/` — core loop: Quest data in, retargeting, calibration against the robot's
  kinematic chain, ZMQ `command`/`planner` out to the GR00T deploy process.
- `data_collection/` — LeRobot-format dataset recorder. Self-contained and currently not
  wired into the launchers (it requires a camera server); delete this directory if data
  collection isn't needed.
- `hand_control/` — direct DDS publisher to `third_party/brainco_hand_service`, independent of
  the SONIC deploy binary.
- `scripts/` — `launch_robot_side.sh` (on-robot teleop), `launch_replay.sh` (off-robot replay).
- `install_scripts/` — `install_teleop.sh` creates `.venv_teleop`.
- `third_party/` — submodules: ROS-TCP-Endpoint, vr_haptic_msgs, brainco-retargeting,
  brainco_hand_service.

## Dependency on the GR00T repo

`$GROOT_REPO` (default `$HOME/GR00T-WholeBodyControl`) supplies two things:

1. **The SONIC deploy process** — launched as a black box by the `deploy` component and
   spoken to over ZMQ 5556/5557 only. It needs no code changes to work with this repo.
2. **The `gear_sonic` Python package** — `teleop_manager/quest_manager.py` and
   `data_collection/schema.py` import it for robot kinematics (forward-kinematics
   calibration, dataset joint schema) rather than duplicating robot geometry here.

That checkout needs the BrainCo kinematic patch (URDF/XML/`g1_supplemental_info.py`, no mesh
files required) even though it isn't upstream NVIDIA yet, and `hand_config.hpp` must have
`USE_BRAINCO_HANDS 1`. `gear_sonic` is installed by `install_scripts/install_teleop.sh` from
`$GROOT_REPO`, not declared in `pyproject.toml` — PEP 508 URLs can't interpolate env vars, so
a hardcoded path would only ever work on one machine.

See `CLAUDE.md` for the planned move off the fork (the foundation model already comes from
upstream HF; what's fork-local is BrainCo support and the `zmq_manager` input path).
