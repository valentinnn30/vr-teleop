# Interface contract with the GR00T repo

This repo (`vr-teleop`) and a GR00T-WholeBodyControl checkout (the SONIC deploy binary /
`gear_sonic_deploy`) run as two separate processes/repos that talk over ZMQ. This is the
complete surface area — nothing else needs to be shared between them, and the GR00T repo's
deploy binary needs **zero code changes** to work with this repo, as long as both sides keep
to the wire format below.

## Ports and topics

| Direction | Port | Topic(s) | Producer | Consumer | Format |
|---|---|---|---|---|---|
| teleop → GR00T | **5556** | `command`, `planner`, `manager_state` | this repo (`teleop_manager/quest_manager.py` via `teleop_manager/zmq_io/zmq_planner_sender.py`) | GR00T repo's SONIC deploy process (`g1_deploy_onnx_ref`) | topic-name prefix + JSON header (field names/dtypes/shapes) + binary payload |
| GR00T → teleop | **5557** | `g1_debug` — carries `body_q_measured` (polled continuously by `quest_manager.py` for wrist-FK calibration) and `robot_config` (polled once at startup by `data_collection/run_data_exporter.py`) | GR00T repo's deploy process | this repo | msgpack |
| quest_relay → teleop | 5559 | `quest_data` | this repo's own `quest_bridge/relay.py` (Docker/ROS1 container) | this repo (`teleop_manager/quest_manager.py`) | msgpack |

Port 5559 is entirely internal to this repo (relay → quest manager) — not part of the
GR00T-facing contract.

## Message field layout (5556, `planner` topic)

- `vr_position[9]` — rows = [L-wrist, R-wrist, head], KEY-FRAME points; local offsets
  (`[0.18, ∓0.025, 0]` for the wrists, `[0, 0, 0.35]` above the torso for the head) are already
  applied, rotated by the live commanded orientation.
- `vr_orientation[12]` — scalar-first quaternions for the same three rows.
- `left_hand_joints[7]` / `right_hand_joints[7]` — BrainCo: 6 normalized motors `[0=open,
  1=closed]` + one `0.0` padding slot.
- `mode` + `movement[3]` — locomotion command.
- `mode` + `height` — crouch command.

See `teleop_manager/quest_manager.py`'s module docstring for the full field-by-field
description, and `docs/frame_report.md` for the coordinate-frame conventions (ROS FLU:
X-forward, Y-left, Z-up; scalar-first quaternions) these fields use.

## Why the GR00T side needs no code changes

The GR00T repo's deploy binary is a pure ZMQ PUB/SUB peer with a fixed wire format — it has no
import-level or process-level dependency on this repo's Python code. The one thing that *does*
need to exist on the GR00T side is the small BrainCo kinematic patch (URDF/XML/
`g1_supplemental_info.py` under `gear_sonic/data/robot_model/model_data/g1/with_brainco/` — no
mesh files needed) so that `instantiate_g1_robot_model(hand_type="brainco")` resolves, since
this repo's calibration (`teleop_manager/quest_manager.py`) and data-collection schema
(`data_collection/schema.py`) both depend on `gear_sonic[teleop]`/`[data_collection]` directly
(see `pyproject.toml`) rather than duplicating robot geometry.

## Format-drift protection

Recommended follow-up (not yet implemented): a round-trip pack/unpack fixture test in this repo
that packs a `planner`/`command` message with `zmq_planner_sender.py` and asserts the byte
layout against a fixture captured from the current GR00T repo, so a future accidental format
change on either side is caught without needing the other repo present in CI.
