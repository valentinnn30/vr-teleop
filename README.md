# vr-teleop

VR teleoperation stack for the G1 + BrainCo hands project: Meta Quest bridge, hand
retargeting, data collection/recording, and BrainCo hand control. No body-control or
simulation code — this repo talks to a separate GR00T-WholeBodyControl checkout's SONIC
deploy process over ZMQ. See `docs/INTERFACE_CONTRACT.md` for that boundary and
`docs/ROBOT_TELEOP_QUICKSTART.md` for setup/run instructions.

## Layout

- `quest_bridge/` — ROS1↔ZMQ relay between the Quest headset (via ROS-TCP-Endpoint) and
  `teleop_manager/`. Robot-agnostic.
- `retargeting/hand/` — MANO-21 landmarks → normalized hand-motor commands. BrainCo today
  (`third_party/brainco-retargeting`); a different hand later is a new module here.
- `teleop_manager/` — core loop: Quest data in, retargeting, calibration against the robot's
  kinematic chain, ZMQ `command`/`planner` out to the GR00T deploy process.
- `data_collection/` — LeRobot-format dataset recorder. Self-contained; delete this directory
  if data collection isn't needed.
- `hand_control/` — direct DDS publisher to `third_party/brainco_hand_service`, independent of
  the SONIC deploy binary.
- `third_party/` — vendored submodules (ROS-TCP-Endpoint, brainco-retargeting,
  brainco_hand_service).

## Dependency on the GR00T repo

`teleop_manager/calibration` (inside `quest_manager.py`) and `data_collection/schema.py` /
`exporter.py` depend on `gear_sonic[teleop]` / `gear_sonic[data_collection]` from a GR00T
checkout (see `pyproject.toml`) for robot kinematics (forward-kinematics calibration, dataset
joint schema) — not duplicated here. That GR00T checkout needs the BrainCo kinematic patch
(URDF/XML/`g1_supplemental_info.py`, no mesh files required) even though it isn't upstream
NVIDIA yet. See `docs/INTERFACE_CONTRACT.md` for details.
