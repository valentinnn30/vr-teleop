# Bring-up testing script

Run this after installation, in order, on the robot. Each step gates the next: if
one fails, fix it before moving on rather than continuing and debugging two things
at once.

**Steps 1–5 cannot move the robot.** No deploy process is running during them, so
nothing is subscribed to the manager's command port — the arms and hands are
physically incapable of moving no matter what the manager does. The robot moves for
the first time in step 7.

The "want to see" blocks below are real output from a passing run, so you can
compare directly.

```bash
export GROOT_REPO=$HOME/GR00T-WholeBodyControl   # needed by steps 4, 6, 7
cd ~/vr-teleop
```

---

## 1. Build the Quest relay image

```bash
docker build -t quest-relay -f quest_bridge/Dockerfile .
docker run --rm --entrypoint bash quest-relay -c \
  'source /catkin_ws/devel/setup.bash && rosmsg show vr_haptic_msgs/ManoLandmarks'
```

Want:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point[] landmarks
  float64 x
  float64 y
  float64 z
```

This proves `catkin_make` built both ROS packages and that
`third_party/vr_haptic_msgs` took its catkin branch (its `CMakeLists.txt` switches
on `$ROS_VERSION`, which `/opt/ros/noetic/setup.sh` sets to 1). A failure here is
almost always a missing apt package for that package's ROS1 `find_package`.

## 2. Relay alone — no Quest, no robot

```bash
./scripts/launch_robot_side.sh relay
```

Want `roscore up` → `ros_tcp_endpoint started` → `Starting ZMQ relay`, then a status
line every 5 s reading `head=waiting`. That confirms the container runs and binds
TCP 10000 and ZMQ 5559. Leave it running for step 3.

## 3. Quest → relay

Point the Quest Unity app at `<robot-ip>:10000`.

The relay's 5 s status line should flip from `head=waiting` to `head=ok` with real
position values and non-zero landmarks. This validates the entire VR input path —
headset, network, ROS-TCP endpoint, relay — with the robot completely uninvolved.

Ctrl-C when satisfied.

## 4. Manager against the relay, no robot

Relay running (step 2) and Quest connected (step 3). In a second shell:

```bash
cd ~/vr-teleop
$GROOT_REPO/.venv_teleop/bin/python -m teleop_manager.quest_manager \
    --relay-host localhost --no-robot --static-base --latency-csv /tmp/lat.csv
```

`--no-robot` skips the pre-teleop ramp and the `g1_debug` feedback wait; without it
the manager would sit forever waiting for a deploy that isn't running.

Press `s`, then `q` to stop. Want:

```
[QuestManager] Finger retargeting: optimization-based BrainCoRetargeter
[QuestManager] Subscribed to quest relay at tcp://localhost:5559
[QuestManager] --no-robot: skipping the ramp and robot feedback
[QuestManager] Head anchor LOCKED at calibration — head motion does not move the arms
[QuestManager] ZMQ PUB bound to port 5556
[QuestManager] Assume the rest pose — calibrating in 3s (start)
[QuestManager] Calibration captured (FK ref: default rest pose, head yaw +54.2 deg, locked head anchor)
  left:  head->wrist cal vector [-0.041, +0.415, -0.890]
  right: head->wrist cal vector [+0.038, -0.402, -0.885]
[QuestManager] Calibrated — entering live VR_3PT teleop
[QuestManager] VR_3PT | quest=ok | 49.8 planner msg/s | fingers=on | walk=static | crouch=off | anchor=locked
```

Check four things:

- **`optimization-based BrainCoRetargeter`**, not "pure-numpy fallback" — the
  fallback means `brainco-retargeting` didn't install properly.
- **`~49.8 planner msg/s`** against the 50 Hz target.
- **The two wrist vectors differ, with opposite-signed Y.** Identical vectors mean
  neither hand was tracked — both wrist slots are still at their `[0,0,0]` default,
  so `v_cal = R0⁻¹(p_wrist − p_head)` comes out the same for both sides. Get your
  hands into the headset's view and recalibrate with `r`.
- **`anchor=locked`**, from `--static-base`. That is all step 4 can show: the
  manager prints no per-frame target values and the visualizer is not wired into the
  live path, so the arms-do-not-follow-your-head check itself happens with the robot
  in step 7.

**Confirm the frame assumption once, here.** The anchor is only correct because the
Quest publishes wrist positions in the same fixed tracking frame as the head pose,
not relative to the head. On one frame, `left_wrist_pos` z should be ~0.9–1.0 while
`head_pos` z is ~1.27 — both floor-origin. Head-relative wrists would sit near
z≈−0.34, and the anchor would be achieving nothing. Cross-check: `left_landmarks[0]`
(the wrist joint) should equal `left_wrist_pos` rotated +90° about Z, which is the
`_R_TOPIC` convention in `quest_bridge/generate_mock_quest_data.py`. Same thing from
a bag: `header.frame_id` on `/tf` should be a world/origin frame, not `head`.

Then read the latency trace:

```bash
column -s, -t /tmp/lat.csv | head -5
awk -F, 'NR>1 && $4!="" {n++; s+=$4; if($4>m)m=$4} END {print "age_recv_ms mean",s/n,"max",m}' /tmp/lat.csv
```

`age_recv_ms` is relay-publish → manager-read. Single-digit ms means the robot-side
pipeline is healthy. Note this **cannot** include the Quest → robot hop — see the
latency section of `../CLAUDE.md` for why.

## 5. Record a rosbag

```bash
BAG_DIR=~/bags ./scripts/launch_robot_side.sh relay
# Quest connected, hands visible. Hold a rest pose ~5s, then move ~30s.
# Stop with Ctrl-C — NOT `docker kill`.
ls -la ~/bags/
```

**The filename is the test.** A `quest_<timestamp>.bag` means `rosbag record`
received SIGINT and finalised the file. A leftover `.bag.active` means it was
killed mid-write and the recording is truncated — see the `set -m` note in
`../CLAUDE.md`, which is what makes that signal reach it at all.

`rosbag` is ROS1 and exists only inside the relay image, so inspect through it:

```bash
docker run --rm -v ~/bags:/bags --entrypoint bash quest-relay -c \
  'source /catkin_ws/devel/setup.bash && rosbag info /bags/*.bag'
```

Want **all four** topics with non-zero counts:

```
topics:  /quest/pose/headset     928 msgs  : geometry_msgs/PoseStamped
         /tf                     932 msgs  : tf2_msgs/TFMessage
         /quest/hand_pose/left   ...       : vr_haptic_msgs/ManoLandmarks
         /quest/hand_pose/right  ...       : vr_haptic_msgs/ManoLandmarks
```

`rosbag info` only lists topics that actually received messages, so a missing
`/quest/hand_pose/*` means the headset published no finger landmarks — usually hand
tracking off or hands out of view. Such a bag is still a valid recording but is
useless for replay, since it would command static fingers.

To check which frames `/tf` actually carries:

```bash
docker run --rm -v ~/bags:/bags --entrypoint bash quest-relay -c \
  'source /catkin_ws/devel/setup.bash && rostopic echo -b /bags/<file>.bag -n 3 /tf'
```

`relay.py` accepts only `child_frame_id` of `hand_left` / `hand_right` and discards
everything else.

Bags are written by the container as root: `sudo chown -R $USER: ~/bags` if that
gets in the way.

## 6. Hand service alone

```bash
cd $GROOT_REPO/gear_sonic_deploy/thirdparty/brainco_hand_service/bin
sudo ./brainco_hand_server -n enP8p1s0
# second shell:
sudo ./test_brainco_hand_server left && sudo ./test_brainco_hand_server right
```

Fingers should fist and open on both hands. This confirms `enP8p1s0` is the right
interface — it must match the one the deploy binary resolves, and a mismatch is the
usual cause of "arms move, fingers dead".

**Stop it before step 7** — the launcher starts its own instance.

## 7. Full stack — first robot motion

```bash
./scripts/launch_robot_side.sh
```

(Add `TELEOP_VENV=$GROOT_REPO/.venv_teleop` if you have not built this repo's own
venv and your checkout predates the auto-fallback.)

One tmux session `g1_robot`, four tiled panes, started in order **hand → deploy →
relay → manager**.

1. The deploy pane brings up the GR00T container, waits for its `Relays active`
   banner, and types the deploy command in. It stops at
   `Proceed with deployment? [Y/n]:` — **type `y`**. That is the last gate before
   the robot is commanded.
2. Point the Quest at `<robot-ip>:10000`, hands in view.
3. In the manager pane press `s` — the robot ramps to the calibration pose.
4. Check the wrist vectors as in step 4, then press `s` again — countdown,
   calibration, live teleop.
5. Hands held still, **look around and lean** — the arms must not move. Head motion
   is anchored out at calibration, since the robot has no neck to reproduce it with.
   Then move your hands and confirm the arms follow.

Default is `--static-base`: arms and hands only, no walking, turning or crouching,
and the head anchor locked. Widen deliberately with `MANAGER_EXTRA` (see the
quickstart) — note that enabling walking necessarily unlocks the anchor, so head
motion drives the arms again.

If the arms drift over a long session, it is you rather than the robot: a locked
anchor no longer cancels the operator's own drift, so the status line reports
`anchor=locked drift 0.21m — press 'r' to re-anchor` once you are more than
`--anchor-drift-warn` (default 0.15 m) from where you calibrated. `r` re-anchors.

**`o` in the deploy pane is the e-stop.** Stop normally with `q` in the manager,
then Ctrl-C the deploy pane.

---

## Optional: drive the robot from a recording

Once step 7 works, the bag from step 5 can replace the headset entirely:

```bash
PLAY_BAG=~/bags/quest_<timestamp>.bag BAG_LOOP=1 ./scripts/launch_robot_side.sh
```

`PLAY_BAG` keeps the manager in **normal live mode** — the deploy is real, so you
keep the ramp against measured joints:

1. `y` at the deploy prompt.
2. **`s`** — ramps the robot to the calibration pose.
3. **`s`** again — starts the recording. Its first frame becomes the calibration
   reference, so the bag plays out relative to the pose the robot just ramped to.

That is the whole interaction. Calibration is deferred to the recording's first
frame because until playback starts the relay is publishing its all-zero default
snapshot, and anchoring to that would map the recording onto a meaningless
reference.

**Never use `--replay` (NPZ) against a real robot** — it skips the ramp. See the
replay-modes table in `../CLAUDE.md`.
