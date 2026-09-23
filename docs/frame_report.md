> **Note (repo split):** copied verbatim from the `GR00T-WholeBodyControl` fork as a reference —
> `run_data_exporter.py` now lives in this repo at `data_collection/run_data_exporter.py`;
> `process_contacts.py` and `visualize_robot_object_trajectory.py` (referenced below) stay on
> the GR00T-repo side as post-processing tools that consume datasets this repo produces. The
> coordinate-frame conventions described here are part of the wire-format contract between the
> two repos — see `docs/INTERFACE_CONTRACT.md`.

# Coordinate Frames in Recorded Trajectories — Reference Report

**Scope.** This document explains, from a coordinate-frame standpoint, exactly what
`gear_sonic/scripts/run_data_exporter.py` writes to disk, and how
`gear_sonic/scripts/process_contacts.py` and
`gear_sonic/scripts/visualize_robot_object_trajectory.py` read and reinterpret it. It is written
for someone who has never worked with coordinate frames, and is intended as the specification for
writing conversion scripts over recorded data.

**Configuration assumed:** Unitree G1 + BrainCo hands, MuJoCo sim, Meta Quest teleoperation.

**Everything in this document was verified against the real recording**
`outputs/2026-07-09-14-54-30` (episode 1, 815 frames @ 50 Hz) and against the live code, not just
docstrings. Verified numbers are marked ✅ and collected in [Part 7](#part-7--verified-numbers).

---

## Part 0 — A crash course in coordinate frames

Skip to [Part 1](#part-1--the-frames-in-this-system) if you are comfortable with SE(3).

### What a frame is

A **coordinate frame** is an origin plus three axes (X, Y, Z). A point has *no* absolute
coordinates — only coordinates *in some frame*. "The box is at (0.4, 0, 0.76)" is meaningless
until you say **in which frame**. This is the single most common source of bugs in this pipeline,
because a recording contains **at least six different frames**.

### Transforms as 4×4 matrices

A rigid transform (rotation + translation) is stored as a 4×4 matrix:

```
T = [ R  t ]        R = 3x3 rotation matrix
    [ 0  1 ]        t = 3x1 translation vector
```

To transform a point `p` (3 numbers) you append a 1, multiply, and drop the 1:

```python
p_new = T[:3, :3] @ p + T[:3, 3]      # equivalent to (T @ [p, 1])[:3]
```

### Naming convention: `ob_in_world`

This codebase uses the name `X_in_Y`, meaning **"the pose of X expressed in frame Y"**. The
mental model that makes every formula below fall out automatically:

> `T = ob_in_world` converts coordinates **from** the object's own frame **to** the world frame.
> `p_world = T @ p_object`

Read the name right-to-left as "**to** world **from** object". Then:

- **Compose** by cancelling the middle name. Reading each matrix as "to ← from", they chain when
  the left matrix's "from" matches the right matrix's "to":

  ```
  X_in_Z   =   Y_in_Z   @   X_in_Y
  (to Z←X)     (to Z←Y)     (to Y←X)
                    └── Y cancels ──┘
  ```

  Worked example used later in this document — the box's pose in the world, built from the box's
  pose in the reference-foot frame:

  ```
  obj_in_world = ref_in_world @ obj_in_ref
  ```

- **Invert** to flip the direction: `world_in_ob = inv(ob_in_world)`.

  A useful consequence, used in Recipe A: if you know `A_in_W` and `A_in_B`, you can recover
  `B_in_W = A_in_W @ inv(A_in_B)`. That is exactly how the robot's missing world pose is
  recovered from its foot.

### Quaternions — the #1 trap

A rotation can also be stored as 4 numbers (a quaternion). There are two orderings in the wild:

| Order | Also called | Used by |
|---|---|---|
| `(w, x, y, z)` | **scalar-first**, `wxyz` | **MuJoCo, and every quaternion stored in this repo's datasets** |
| `(x, y, z, w)` | scalar-last, `xyzw` | **scipy's `Rotation.from_quat()` default** |

**Every quaternion written to disk in this pipeline is `wxyz` (scalar-first).** With scipy you
must therefore always pass `scalar_first=True`:

```python
from scipy.spatial.transform import Rotation as R
Rot = R.from_quat(q_wxyz, scalar_first=True)     # correct
Rot = R.from_quat(q_wxyz)                        # SILENTLY WRONG — treats w as x
```

This fails silently and produces a plausible-looking but wrong rotation. If a conversion looks
"almost right but tilted", check this first.

### Flattened matrices are row-major

4×4 transforms stored in parquet are flattened to **16 floats, row-major** (C order). Recover
with `.reshape(4, 4)` — NumPy's default. `.reshape(4, 4, order='F')` would be wrong.

### Units

Meters and radians everywhere, with **one exception**: depth PNGs are `uint16` **millimeters**.

---

## Part 1 — The frames in this system

Six frames matter. This table is the key to the whole document.

| # | Frame | Where its origin is | Axis convention | Who uses it |
|---|---|---|---|---|
| 1 | **Sim world** `W_sim` | Fixed point in the MuJoCo scene | Z up | `object_gt/` (`ob_in_world`, `ref_in_world`) |
| 2 | **Replay world** `W_replay` | Reconstructed by the visualizer; feet-planted | Z up | `visualize_robot_object_trajectory.py`, `ob_in_world_filtered/` |
| 3 | **Pelvis / base** `B` | The robot's `pelvis` link | X fwd, Y left, Z up | `observation.eef_state`, all Pinocchio FK |
| 4 | **Head camera (MuJoCo/GL)** `C_gl` | `head_camera`, mounted on `torso_link` | X right, Y **up**, Z **backward** | MuJoCo camera FK |
| 5 | **Head camera (OpenCV)** `C_cv` | Same origin as `C_gl` | X right, Y **down**, Z **forward** | FoundationPose `ob_in_cam/` |
| 6 | **Object-local** `O` | The box's centre | Along box edges | `contacts/` (`contact_points`), `box.obj` |

Plus one bridging frame:

| # | Frame | Notes |
|---|---|---|
| 7 | **Reference body** `R` | `right_ankle_roll_link` (the right foot). Never a storage frame — it exists **only** to let you reconstruct the robot's world position. This is the linchpin of Recipe A below. |

### Notes on each

**1. Sim world (`W_sim`)** — a fixed, non-moving (inertial) frame. The box, table, and robot are
placed at explicit coordinates in it. This is the "real" ground-truth frame in simulation. It
does **not** exist on real hardware, which is why the whole pipeline can't simply use it.

**2. Replay world (`W_replay`)** — the visualizer must place the robot *somewhere*, but the
recording contains **no base world position** (see Part 3). So it invents a world:

- the pelvis translation at frame 0 is set to the scene's default standing position (`qpos0[0:3]`);
- the **initial yaw is subtracted** from the base orientation (frame 0 faces "yaw = 0");
- for every frame, base translation is solved so the **feet midpoint stays pinned** at its
  frame-0 location.

`W_replay` therefore differs from `W_sim` by a **near-constant rigid transform**, which the code
calls `gt_anchor`. ✅ Measured for the reference recording: a 1.296° yaw rotation and a 3.2 cm
translation.

**3. Pelvis (`B`)** — ⚠️ **The Pinocchio robot model is FIXED-BASE.** Verified:
`instantiate_g1_robot_model()` never passes `set_floating_base`, so it defaults to `False`. The
practical consequence, and probably the single most important fact in this report:

> **All Pinocchio FK — including `observation.eef_state` — is expressed in the PELVIS frame, not
> the world frame.** The Pinocchio `q` vector is 51 joint angles with **no floating-base DOFs**.

`RobotModel.frame_placement()`'s docstring says "in the world coordinate system"; that docstring
is **misleading for this configuration**. Because the model is fixed-base, Pinocchio's "world" *is*
the URDF root link = `pelvis`. ✅ Verified: FK of the wrist at `q = 0` returns
`(0.1998, ±0.1851, 0.1003)` — clearly pelvis-relative, not a world position.

**4 & 5. Head camera** — declared in the MJCF as a child of `torso_link`:

```xml
<camera name="head_camera" pos="0.06 0.0 0.45" euler="0 -0.8 -1.57"/>
```

It is **pitched down by 0.8 rad (~46°)**. This matters: a small base-pitch error levers the
camera-anchored object by several centimetres. The two camera frames share an origin but differ
in axis convention, related by a constant:

```python
GL_FROM_CV = np.diag([1.0, -1.0, -1.0, 1.0])   # flips Y and Z
```

**6. Object-local (`O`)** — origin at the **box centre**, axes along box edges. ✅ Verified:
`box.obj` spans exactly `[-0.025, +0.025]` on every axis with centroid exactly `(0,0,0)` — a 5 cm
cube, perfectly centred. Each face is colour-coded (`+x, -x, +y, -y, +z, -z`) so FoundationPose can
resolve orientation.

---

## Part 2 — On-disk layout

```
outputs/<timestamp>/
├── meta/
│   ├── info.json                       # fps, features schema, script_config
│   ├── modality.json                   # slice map into the feature vectors
│   ├── episodes.jsonl, tasks.jsonl
├── data/chunk-000/
│   └── episode_NNNNNN.parquet          # ROBOT: one row per frame @ 50 Hz  [LeRobot]
├── videos/chunk-000/                   # ego-view mp4
├── object_gt/
│   └── episode_NNNNNN.parquet          # OBJECT ground truth, sim world     [dense]
├── foundation_pose_data/
│   ├── box.obj                         # object mesh, object-local frame
│   └── episode_NNNNNN/
│       ├── cam_K.txt                   # 3x3 OpenCV intrinsics
│       ├── rgb/NNNNNN.png              # 8-bit RGB
│       ├── depth/NNNNNN.png            # uint16 MILLIMETERS
│       ├── masks/000000.png            # frame 0 ONLY
│       ├── frame_map.txt               # fp_frame -> proprio row
│       ├── ob_in_cam/NNNNNN.txt        # 4x4, OpenCV optical frame  [FoundationPose output]
│       └── ob_in_world_filtered/       # 4x4, REPLAY world          [filter_object_pose.py output]
└── contacts/
    ├── episode_NNNNNN.parquet          # CONTACTS, object-local frame
    └── meta.json                       # segment_names, threshold, conventions
```

**Three independent writers, three different frames.** The robot parquet, `object_gt/`, and
`contacts/` are written by three separate code paths and **do not share a frame**. Nothing on disk
pre-joins them: relating them in space is Part 4's job, and in time is Part 5's.

---

## Part 3 — Column-by-column frame reference

### 3.1 `data/chunk-000/episode_NNNNNN.parquet` — the robot

One row per frame at 50 Hz. Written by `run_data_exporter.py::_add_data_frame_sonic`.
Schema from `gear_sonic/data/features_sonic_vla.py`.

| Column | Shape | Frame | Meaning |
|---|---|---|---|
| `observation.state` | (51,) f64 | **joint space — no frame** | `whole_q`: measured joint angles, radians |
| `observation.eef_state` | (14,) f64 | ⚠️ **PELVIS** | `[l_pos(3), l_quat_wxyz(4), r_pos(3), r_quat_wxyz(4)]` |
| `action.wbc` | (51,) f64 | joint space | Commanded joint targets, same layout as state |
| `observation.root_orientation` | (4,) f64 | **world → base rotation** | Base orientation, `wxyz`. **The only robot orientation stored.** |
| `observation.projected_gravity` | (3,) f64 | **base** | World gravity `(0,0,-1)` rotated into the base frame |
| `observation.init_base_quat` | (4,) f64 | world | Base quat at controller init, `wxyz` |
| `observation.cpp_rotation_offset` | (4,) f64 | world | Controller's reference-motion root rotation, `wxyz` |
| `teleop.delta_heading` | (1,) f64 | — | Commanded yaw offset (rad) |
| `teleop.smpl_joints` | (72,) f32 | SMPL root, heading-normalised | 24 joints × 3 |
| `teleop.smpl_pose` | (63,) f32 | — | SMPL body pose (axis-angle, 21 joints × 3) |
| `teleop.body_quat_w` | (4,) f32 | VR world | SMPL root orientation, `wxyz` |
| `teleop.target_body_orientation` | (6,) f32 | yaw-normalised | rot6d |
| `teleop.vr_3pt_position` | (9,) f32 | head-relative | `[l_wrist, r_wrist, neck]` |
| `teleop.vr_3pt_orientation` | (18,) f32 | VR world | rot6d × 3 |
| `teleop.{left,right}_hand_joints` | (6,) f32 | joint space | Commanded BrainCo hand joints |
| `frame_index` | int64 | — | **Row index within the episode — the join key** |
| `timestamp`, `episode_index`, `index`, `task_index` | | | LeRobot bookkeeping |

#### 🔴 The single most important fact: the robot's world POSITION is never stored

The recording stores base **orientation** (`observation.root_orientation`) but **no base
translation** — not in any column, not anywhere. This is deliberate: on real hardware there is no
world position to record (the IMU gives gravity-aligned orientation with an arbitrary yaw origin;
there is no global localisation).

Consequences you must design around:

1. `observation.state` + `observation.eef_state` describe the robot **only up to an unknown rigid
   placement in the world**.
2. The visualizer *invents* the missing translation by pinning the feet (`W_replay`).
3. **This is exactly why `object_gt` stores `ref_in_world` alongside `ob_in_world`** — it is the
   one recorded quantity that ties the robot to the sim world. Recipe A below exploits this.

#### On `observation.root_orientation`

Sourced from the `g1_debug` ZMQ topic's `base_quat`, which the C++ deploy header documents as
"IMU base orientation (qw,qx,qy,qz)". In sim, the bridge feeds it directly from the MuJoCo pelvis
freejoint quaternion (`unitree_sdk2py_bridge.py:250`):

```python
self.low_state.imu_state.quaternion[:] = obs["floating_base_pose"][3:7]
```

So **in sim it is the exact pelvis orientation in `W_sim`**. On real hardware it is gravity-aligned
(pitch/roll absolute) with an **arbitrary yaw origin**. Portable code must never trust absolute
yaw — only yaw *differences*. This is precisely why `load_base_quats()` subtracts frame 0's yaw
but keeps the yaw *variation*:

```python
euler = R.from_quat(quats, scalar_first=True).as_euler("ZYX")
euler[:, 0] -= euler[0, 0]        # remove initial yaw; KEEP the variation
```

Keeping the variation is not cosmetic: when the robot turns, the head camera sweeps, and a
physically static object sweeps across the image with it. That rotation must be reproduced so it
cancels on reprojection — otherwise a static box appears to swing in azimuth.

#### ⚠️ `observation.state`: passive finger joints are stored as ZERO

The BrainCo fingers are **underactuated**. Each `*_distal_joint` is passive, driven by its
`*_proximal_joint` through a mimic coupling. Only the 6 actuated joints per hand are filled;
**every `*_distal_joint` is written as exactly 0.0**. ✅ Verified: `right_index_distal_joint` is
identically 0.000 across all 815 frames, while `right_index_proximal_joint` ranges to 0.360 rad.

The exact coupling, read from the MJCF's `mjEQ_JOINT` equality constraints:

| Joint | Driven by | Multiplier |
|---|---|---|
| `{side}_thumb_distal_joint` | `{side}_thumb_proximal_joint` | **1.0** |
| `{side}_{index,middle,ring,pinky}_distal_joint` | corresponding `_proximal_joint` | **1.155** |

**Any FK you run on `whole_q` must re-apply this coupling first**, or the fingers stay straight and
never close around the object. MuJoCo's `mj_forward` does *not* project `qpos` onto equality
constraints, which is why `TrajectoryReplay._apply_joint_couplings()` does it explicitly every
frame.

✅ Verified magnitude of skipping it — fingertip FK error at frame 400:

| Fingertip | Displacement if coupling is skipped |
|---|---|
| `right_thumb_tip` | **1.35 cm** |
| `right_index_tip` | **1.42 cm** |
| `right_ring_tip` | **2.55 cm** |

That is far larger than the 5 mm contact threshold — skipping this silently invalidates any
contact or fingertip analysis.

#### `whole_q` layout (BrainCo, 51 DOF)

Index → joint name, from `RobotModel.joint_names`:

```
 0- 5  left_leg    left_hip_{pitch,roll,yaw}, left_knee, left_ankle_{pitch,roll}
 6-11  right_leg   right_hip_{pitch,roll,yaw}, right_knee, right_ankle_{pitch,roll}
12-14  waist       waist_{yaw,roll,pitch}
15-21  left_arm    left_shoulder_{pitch,roll,yaw}, left_elbow, left_wrist_{roll,pitch,yaw}
22-32  left_hand   left_{index,middle,pinky,ring}_{proximal,distal}, left_thumb_{metacarpal,proximal,distal}
33-39  right_arm   right_shoulder_{pitch,roll,yaw}, right_elbow, right_wrist_{roll,pitch,yaw}
40-50  right_hand  right_{index,middle,pinky,ring}_{proximal,distal}, right_thumb_{metacarpal,proximal,distal}
```

Actuated subsets (the rest are the passive distals):
- body: `[0..21, 33..39]` (29 DOF)
- left hand: `[30, 31, 22, 24, 28, 26]` — ⚠️ **not sorted**; thumb first, then index/middle/ring/pinky
- right hand: `[48, 49, 40, 42, 46, 44]`

> Do not hardcode these. Derive them at runtime via `robot_model.get_joint_group_indices(...)`,
> `get_body_actuated_joint_indices()`, and `get_hand_actuated_joint_indices(side)`.

---

### 3.2 `object_gt/episode_NNNNNN.parquet` — the object (ground truth, sim only)

Written by `ObjectGtWriter`, gated behind `--record-object-gt`. **Dense: one row per recorded
robot frame.** ✅ Verified: 815 rows for 815 robot frames, `proprio_frame_index` = 0..814,
monotonic.

| Column | Type | Frame | Meaning |
|---|---|---|---|
| `proprio_frame_index` | int64 | — | Robot parquet row this pose pairs with |
| `timestamp` | f64 | — | Wall clock, from the sim publisher |
| `ob_in_world` | 16 × f64 | **`W_sim`** | Box pose, 4×4 **row-major** |
| `ref_in_world` | 16 × f64 | **`W_sim`** | `right_ankle_roll_link` pose, 4×4 row-major |

**Why the world frame, not a robot-relative frame?** Because expressing the box relative to a
*moving* link folds that link's motion into the stored pose, making a physically static cube appear
to drift and jitter on replay. Storing it absolutely means a static box has a genuinely constant
recorded pose. (An older schema stored `ob_in_ref` / `ob_in_cam` and suffered exactly this;
`convert_object_gt_to_world.py` migrates those recordings — see Part 6.)

**Why `ref_in_world` too?** It is the bridge back to the robot. Both poses are sampled from the
**same `mj_data`**, i.e. the same physics step, so they are perfectly time-consistent with each
other. `right_ankle_roll_link` was chosen because a planted foot is near-static. ✅ Verified: the
reference foot moved only `(2.2, 1.7, 0.4) mm` over the entire episode.

**Time alignment.** The box-GT stream comes straight from the sim and is *fresher* than the proprio
stream (which crosses the deploy pipeline). Naively pairing "latest with latest" would stamp each
robot row with a box pose from slightly *later*, making the replayed cube run ahead of the hand. So
the exporter keeps a short history and picks the box pose whose timestamp is **closest to the
proprio row's `ros_timestamp`** (`_select_box_gt`). The pairing is already baked into
`proprio_frame_index` — **trust that column; do not re-derive the alignment from timestamps.**

---

### 3.3 `foundation_pose_data/` — the object (vision estimate)

**Sparse**: FP frames are rendered at a reduced rate. ✅ Verified from `frame_map.txt`: FP frame
0→proprio row 1, 1→6, 2→10 — roughly **1 FP frame per 5 proprio frames** (~10 Hz vs 50 Hz).

| Path | Frame | Notes |
|---|---|---|
| `box.obj` | **object-local** | Metres, centred; per-face colours |
| `cam_K.txt` | — | 3×3 OpenCV pinhole intrinsics, row-major |
| `rgb/NNNNNN.png` | — | 8-bit. Written BGR-swapped by cv2; reads back as RGB |
| `depth/NNNNNN.png` | `C_cv` | ⚠️ `uint16` **millimetres** — divide by 1000 for metres |
| `masks/000000.png` | — | ⚠️ **frame 0 only** — FP initialises from it, then tracks |
| `frame_map.txt` | — | `# fp_frame proprio_frame_index timestamp` |
| `cam_extrinsics.txt` | — | 4×4 depth→colour. **Real RealSense only**; absent in sim |
| `ob_in_cam/NNNNNN.txt` | ⚠️ **`C_cv`** | 4×4 object-in-camera. FoundationPose's output |
| `ob_in_world_filtered/NNNNNN.txt` | ⚠️ **`W_replay`** | 4×4. `filter_object_pose.py`'s output |

⚠️ **`ob_in_cam` and `ob_in_world_filtered` are in different frames.** This trips people up
constantly. They are both 4×4 text files in sibling directories, but:

- `ob_in_cam` is **object-in-camera**, OpenCV optical convention → needs `GL_FROM_CV` **and** the
  camera FK to reach a world frame.
- `ob_in_world_filtered` is **already a world pose** in `W_replay` → must be placed **directly**.
  Re-multiplying it by the camera FK would re-inject the camera wobble the filter just removed
  (amplified by the ~1.8 m lever arm to the object), which is why a camera-frame round-trip leaves
  the replay looking unfiltered.

The `obj_in_world` flag in `TrajectoryReplay` selects between these two behaviours.

---

### 3.4 `contacts/episode_NNNNNN.parquet` — finger↔object contacts

Written by `process_contacts.py`. **Dense: one row per robot frame** (✅ verified 815 rows), keyed
by `proprio_frame_index`. Schema mirrors ConTrack's convention (5 mm proximity threshold by
default; **this recording used 10 mm** — always read it from `meta.json`).

| Column | Shape | Frame | Meaning |
|---|---|---|---|
| `proprio_frame_index` | int64 | — | Robot parquet row |
| `is_contact` | (32,) uint8 | — | Binary flag per finger segment |
| `contact_points` | (96,) f64 | ⚠️ **object-local** | 32 × 3, row-major xyz; **NaN** where not in contact |
| `contact_dist` | (32,) f64 | — | Surface gap in metres; **NaN** where not in contact |

Reshape `contact_points` to `(T, 32, 3)`. To match ConTrack's `(T, H, segments)` layout, reshape a
`(T, 32)` column to `(T, 2, 16)` — left hand first, then right.

✅ Verified: `is_contact` and "all three coordinates finite" agree on **every one of the 815 × 32
entries**. So you can use either as the mask; they never disagree.

#### Why object-local is the right choice

The contact point is stored **relative to the box**, so it "rides" on the box surface regardless of
where the box is placed. This makes contacts **invariant to the arbitrary choice of world origin** —
you can replay them against ground-truth, filtered, or raw FP poses and the dots stay glued to the
surface.

✅ Verified: all contact points lie on the box surface. Per-axis range `[-0.0298, +0.0299]` against
a box of half-extent 0.025, and `max |p| = 0.0472 m` vs. the theoretical corner bound
`sqrt(3)·0.025 + threshold/2 = 0.0483 m`. Consistent.

**Caveat worth understanding:** object-local contacts are invariant to the *world origin*, but they
still depend on the **hand↔box relative geometry** being correct — which depends on the object pose
source. Contacts computed from `--from-vision` inherit FoundationPose's error. Ground truth
(the default) is exact.

#### How contacts are computed (no physics stepping)

`process_contacts.py` reuses the visualizer's replay. Each frame it poses the robot from `whole_q`
(re-applying the mimic coupling) and places the box at its object pose, then builds the model with
the object made **collidable with `margin = threshold`**. MuJoCo's `mj_forward` runs its collision
phase even without `mj_step`, so it reports every object↔finger pair whose surface gap ≤ threshold.
The closest contact per segment is kept and its midpoint expressed in the object-local frame.

#### 🔴 `segment_names` are MJCF body names — they do NOT all resolve in Pinocchio

This is a **genuine, non-obvious trap** discovered while verifying this report. The 32
`segment_names` in `contacts/meta.json` are **MuJoCo body names**. The URDF (and therefore the
Pinocchio model) uses **different names for the right hand**:

| Segment | MJCF name (in `meta.json`) | URDF / Pinocchio frame name |
|---|---|---|
| Left, non-tip | `left_index_distal_Link` | `left_index_distal_Link` ✅ identical |
| Left, tip | `left_index_tip_Link` | `left_index_tip_Link` ✅ identical |
| **Right, non-tip** | `right_index_distal_Link` | `right_index_distal_link` ❌ **lowercase `l`** |
| **Right, tip** | `right_index_tip_Link` | `right_index_tip` ❌ **no `_Link` suffix** |

If you feed `meta.json`'s `segment_names` straight into `robot_model.frame_placement()`,
**11 of the 32 right-hand segments raise `ValueError`** — and since the right hand is the
manipulating hand in these recordings, that's exactly the half you care about. ✅ Verified: this
resolver maps all 32/32 correctly:

```python
pin_frames = {f.name for f in robot_model.pinocchio_wrapper.model.frames}

def mjcf_to_urdf(name: str) -> str | None:
    """MJCF body name (contacts/meta.json) -> Pinocchio frame name."""
    for cand in (name, name[:-4] + "link", name.replace("_tip_Link", "_tip")):
        if cand in pin_frames:
            return cand
    return None
```

The 32 segments are 16 per hand: for each of `thumb, index, middle, ring, pinky` →
`proximal, distal, tip`, plus `metacarpal` for the thumb only.

---

## Part 4 — How the robot is stored relative to the object

**It isn't.** There is no stored robot↔object relative pose. Nothing on disk directly answers
"where is the hand relative to the box". You must reconstruct it, and there are three ways.

The obstacle is always the same: `eef_state` is in the **pelvis** frame, `ob_in_world` is in
**`W_sim`**, and the transform between them — the robot's world pose — **is not recorded**.

```
    eef_state ──in pelvis──> [ ??? ] <──in W_sim── ob_in_world
                          the missing link
```

### 🥇 Recipe A — exact, replay-free (sim ground truth). **Recommended.**

`ref_in_world` *is* the missing link. It gives the right foot's pose in `W_sim`; Pinocchio FK gives
the same foot's pose in the pelvis frame. Compose them and the robot's world pose falls out:

```
pelvis_in_world(i) = ref_in_world(i) @ inv(ref_in_pelvis(i))
                     └─ recorded ─┘    └─ FK from whole_q ─┘
```

This requires **no MuJoCo, no viewer, and no feet-planting approximation**. It is essentially exact:
`ref_in_world` is the sim's true link pose, and the leg joints in `whole_q` are the measured
values, so FK reproduces the sim geometry.

```python
import numpy as np, pyarrow.parquet as pq
from scipy.spatial.transform import Rotation as R
from gear_sonic.data.features_sonic_vla import get_g1_robot_model

rm = get_g1_robot_model(hand_type="brainco")

def se3(placement):
    T = np.eye(4); T[:3, :3] = placement.rotation; T[:3, 3] = placement.translation
    return T

d   = pq.read_table("data/chunk-000/episode_000001.parquet")
q   = np.asarray(d.column("observation.state").to_pylist())      # (N, 51)
eef = np.asarray(d.column("observation.eef_state").to_pylist())  # (N, 14) pelvis frame

gt   = pq.read_table("object_gt/episode_000001.parquet")
ob   = np.asarray(gt.column("ob_in_world").to_pylist()).reshape(-1, 4, 4)
ref  = np.asarray(gt.column("ref_in_world").to_pylist()).reshape(-1, 4, 4)
gidx = np.asarray(gt.column("proprio_frame_index").to_pylist())

i = 400                                            # robot row
g = int(np.searchsorted(gidx, i, side="right")) - 1  # matching object row (hold-last)

rm.cache_forward_kinematics(q[i])                  # NOTE: apply mimic coupling first if you
                                                   #       need finger frames — see 3.1
ref_in_pelvis   = se3(rm.frame_placement("right_ankle_roll_link"))
pelvis_in_world = ref[g] @ np.linalg.inv(ref_in_pelvis)
obj_in_pelvis   = np.linalg.inv(pelvis_in_world) @ ob[g]

# Everything is now in ONE frame (the pelvis). Examples:
right_wrist_pos_pelvis = eef[i][7:10]
box_pos_pelvis         = obj_in_pelvis[:3, 3]

# Or flip it: the robot expressed in the OBJECT's frame (what a manipulation policy wants)
wrist_in_pelvis = np.eye(4)
wrist_in_pelvis[:3, :3] = R.from_quat(eef[i][10:14], scalar_first=True).as_matrix()
wrist_in_pelvis[:3, 3]  = eef[i][7:10]
wrist_in_object = np.linalg.inv(obj_in_pelvis) @ wrist_in_pelvis
```

✅ **Verified across the reference recording.** Recipe A yields:

| Check | Result | Interpretation |
|---|---|---|
| Pelvis height in `W_sim` | **0.7864 – 0.7870 m** | Correct for a standing G1 |
| Pelvis height stability | spread **0.65 mm** over 815 frames | No drift — the reconstruction is stable |
| Box in pelvis frame | `(0.37, 0.00, −0.03)` m | 37 cm in front, slightly below — a tabletop cube ✅ |
| Box ↔ right wrist distance | 0.16 – 0.29 m | Plausible reach for `wrist_yaw_link` |

**End-to-end chain validation.** Taking each of the **1220 recorded contacts**, mapping its
object-local point into the pelvis frame via Recipe A, and comparing against the contacting finger
segment's own FK origin:

> mean **2.95 cm**, median **2.79 cm**, max **4.92 cm** — all within one finger-segment length.

This is the expected magnitude (the contact lies on the segment's *surface*; FK returns its
*origin*) and confirms the whole chain — robot parquet, `object_gt`, `contacts/meta.json`, and the
name mapping — is mutually consistent. A frame error would show up as metres, not centimetres.

### 🥈 Recipe B — the replay world (what the visualizer does)

Use when you want to match the visualizer exactly, or need a full MuJoCo scene. `W_replay` is
reconstructed by feet-planting, then `W_sim` is mapped into it by a **single constant anchor built
at frame 0**:

```python
gt_anchor = replay_ref_pose(0) @ inv(ref_in_world[object_index(0)])   # built ONCE
obj_in_replay(i) = gt_anchor @ ob_in_world[object_index(i)]           # every frame
```

The anchor is deliberately **constant**. Recomputing it per frame would fold live robot motion back
into the cube — the exact bug the world-frame schema was introduced to fix.

✅ Verified: `gt_anchor` = 1.296° yaw + `(−0.0284, 0.0147, 0.0066)` m translation, and it stays
valid across the episode to **mean 0.88 mm / max 1.48 mm**.

### 🥉 Recipe C — the vision path (works on real hardware)

```python
# raw FoundationPose: object-in-camera (OpenCV) -> world, via camera FK
obj_in_world = cam_in_world(i) @ GL_FROM_CV @ ob_in_cam(j)

# filtered: ALREADY a world pose — place directly, do NOT re-apply the camera FK
obj_in_world = ob_in_world_filtered(j)
```

`cam_in_world` comes from the MuJoCo `head_camera` FK after the robot is posed in `W_replay`.

### Choosing a recipe

| Situation | Use |
|---|---|
| Sim recording, want exact numbers, no MuJoCo dependency | **A** ⭐ |
| Need to match the visualizer / build a MuJoCo scene | **B** |
| No `object_gt` (real hardware, or legacy recording) | **C** |

---

## Part 5 — Time alignment and join keys

Three timelines exist. **Never align by timestamp — the join keys are already on disk.**

| Stream | Rate | Join key |
|---|---|---|
| Robot parquet | 50 Hz | `frame_index` (row within episode) |
| `object_gt` | 50 Hz, **dense** | `proprio_frame_index` → robot row |
| `contacts` | 50 Hz, **dense** | `proprio_frame_index` → robot row |
| FoundationPose | ~10 Hz, **sparse** | `frame_map.txt`: `fp_frame → proprio_frame_index` |

For **dense** streams (`object_gt`, `contacts`) rows are 1:1 with robot rows and monotonic —
✅ verified 815/815, `proprio_frame_index` = 0..814. You can usually index directly, but
`searchsorted` is safer:

```python
g = int(np.searchsorted(gidx, i, side="right")) - 1   # hold-last, robust to gaps
```

For the **sparse** FP stream, the visualizer uses hold-last (`object_index()`): the latest object
frame whose proprio row is ≤ `i`. If `frame_map.txt` is missing (legacy recordings), it falls back
to uniform nearest-neighbour resampling with the robot as master timeline — approximate; prefer the
map when present.

To scatter a sparse/dense sidecar onto the robot timeline (as `load_contacts` does):

```python
out = np.full((n_robot_frames, s, 3), np.nan)
valid = (idx >= 0) & (idx < n_robot_frames)
out[idx[valid]] = pts[valid]
```

---

## Part 6 — Gotchas checklist

Ordered by how likely they are to silently corrupt a conversion:

1. **Quaternion order.** Everything on disk is `wxyz`. scipy defaults to `xyzw`. Always
   `scalar_first=True`. Fails silently.
2. **Pinocchio is fixed-base → `eef_state` is in the PELVIS frame, not world.** `frame_placement`'s
   "world coordinate system" docstring is misleading here.
3. **No base world position is recorded.** Ever. Use Recipe A (`ref_in_world`) to recover it.
4. **Passive distal finger joints are stored as 0.** Re-apply the mimic coupling (×1.155 fingers,
   ×1.0 thumb) before any finger FK. Costs 1.4–2.6 cm of fingertip error if skipped.
5. **`segment_names` are MJCF names; the right hand's URDF names differ** in case (`_Link` vs
   `_link`) and tip suffix (`_tip_Link` vs `_tip`). 11/32 fail without the mapping in §3.4.
6. **`ob_in_cam` ≠ `ob_in_world_filtered`.** Camera-optical vs. replay-world. Sibling folders,
   different frames.
7. **`GL_FROM_CV` is mandatory** for `ob_in_cam` (OpenCV: Y down, Z forward → MuJoCo: Y up,
   Z back).
8. **Depth is `uint16` millimetres**, not metres.
9. **Only frame 0 has a mask.** FP initialises from it and tracks thereafter.
10. **Flattened matrices are row-major** — plain `.reshape(4, 4)`.
11. **Hand actuated indices are not sorted** (`[30, 31, 22, 24, 28, 26]`). Derive at runtime.
12. **`contact_threshold` is per-recording.** Default 5 mm, but the reference recording used 10 mm.
    Read `meta.json`.
13. **Legacy `object_gt` schemas exist.** Older recordings have `ob_in_ref` or `ob_in_cam` instead
    of `ob_in_world`/`ref_in_world`. Detect by column name; migrate with
    `convert_object_gt_to_world.py`. Migrated data is *reconstructed*, not exact — the sim's link
    pose was never stored, so it is rebuilt via feet-planted FK.
14. **`gt_anchor` assumes a near-static stance.** ⚠️ See below.

### ⚠️ The standing-stance assumption (important design limitation)

The replay pins the feet at their frame-0 location, so **a walking robot is not reproduced** —
it walks in place. Meanwhile the box follows its true `W_sim` trajectory through a **constant**
anchor. If the robot translates significantly, `W_replay` drifts relative to `W_sim`, the frame-0
anchor goes stale, and **hand↔box relative geometry silently degrades**.

This is fine for the current manipulation setup (standing at a table). ✅ Verified for the
reference recording: the reference foot moved only ~2 mm and the anchor held to 1.48 mm.

**If you add locomotion, re-validate this** — and prefer **Recipe A**, which never uses the anchor
or the feet-planting hack and therefore does not carry this assumption.

---

## Part 7 — Verified numbers

All measured from `outputs/2026-07-09-14-54-30`, episode 1 (815 frames @ 50 Hz).

| Quantity | Value |
|---|---|
| Robot model | Fixed-base Pinocchio, 51 DOF, root = `pelvis` |
| Robot frames / object_gt rows / contacts rows | 815 / 815 / 815 (all dense, 1:1) |
| FP frame rate | ~1 per 5 proprio rows (~10 Hz) |
| Box half-extents | `(0.025, 0.025, 0.025)` — 5 cm cube, centred on object-local origin |
| Contact segments | 32 (16/hand) |
| Contact threshold (this recording) | **0.010 m** (default is 0.005) |
| Total contacts | 1220, over 292/815 frames — **right hand only** |
| Contact object-local per-axis range | `[−0.0298, +0.0299]` m |
| Contact `max \|p\|` | 0.0472 m (bound 0.0483 m) ✅ |
| `is_contact` vs finite-point agreement | **815 × 32 exact** ✅ |
| `gt_anchor` | 1.296° yaw, `(−0.0284, 0.0147, 0.0066)` m |
| `gt_anchor` validity over episode | mean 0.88 mm, max 1.48 mm |
| Sim reference foot drift | `(2.2, 1.7, 0.4)` mm |
| Recipe A pelvis height | 0.7864 – 0.7870 m (spread 0.65 mm) |
| Recipe A box in pelvis | `(0.37, 0.00, −0.03)` m |
| End-to-end contact ↔ finger-FK | mean 2.95 cm, median 2.79 cm, max 4.92 cm (n=1220) |
| Fingertip error if mimic coupling skipped | 1.35 / 1.42 / 2.55 cm (thumb / index / ring) |
| `head_camera` | `pos=(0.06, 0, 0.45)` on `torso_link`, `euler=(0, −0.8, −1.57)` — pitched down ~46° |
| Camera intrinsics | `fx = fy = 579.41`, `cx = 320`, `cy = 240` (640×480) |

---

## Part 8 — Quick reference: frame conversion cheat-sheet

```python
GL_FROM_CV = np.diag([1.0, -1.0, -1.0, 1.0])

# --- object poses into a common frame -------------------------------------
obj_in_replay = gt_anchor @ ob_in_world                    # GT      (Recipe B)
obj_in_world  = cam_in_world @ GL_FROM_CV @ ob_in_cam      # FP raw  (Recipe C)
obj_in_world  = ob_in_world_filtered                       # FP filtered — direct!

# --- the missing link: robot world pose (Recipe A, exact, sim only) -------
pelvis_in_world = ref_in_world @ inv(ref_in_pelvis)        # ref_in_pelvis = FK(whole_q)
obj_in_pelvis   = inv(pelvis_in_world) @ ob_in_world

# --- contacts -------------------------------------------------------------
p_world  = obj_in_world[:3, :3] @ p_local + obj_in_world[:3, 3]     # local -> world
p_pelvis = obj_in_pelvis[:3, :3] @ p_local + obj_in_pelvis[:3, 3]   # local -> pelvis

# --- rotations ------------------------------------------------------------
Rot = R.from_quat(q_wxyz, scalar_first=True)               # ALWAYS scalar_first
T   = flat16.reshape(4, 4)                                 # row-major
```

### Source-of-truth files

| Concern | File |
|---|---|
| What gets recorded | `gear_sonic/scripts/run_data_exporter.py` |
| Dataset schema | `gear_sonic/data/features_sonic_vla.py` |
| Object GT writer | `gear_sonic/utils/data_collection/object_gt_writer.py` |
| Object GT producer (frames!) | `gear_sonic/utils/mujoco_sim/base_sim.py::get_box_gt_poses` |
| FP scene writer | `gear_sonic/utils/data_collection/foundation_pose_writer.py` |
| Replay / anchor / couplings | `gear_sonic/scripts/visualize_robot_object_trajectory.py` |
| Contacts | `gear_sonic/scripts/process_contacts.py` |
| Legacy migration | `gear_sonic/scripts/convert_object_gt_to_world.py` |
| Robot model / FK | `gear_sonic/data/robot_model/robot_model.py` |
