#!/bin/bash
set -e

# Job control, and it is load-bearing — not a convenience.
#
# POSIX has a non-interactive shell start background jobs with SIGINT (and
# SIGQUIT) set to SIG_IGN, and bash cannot trap a signal that was already ignored
# when the process started. So without `set -m`, `kill -INT` at shutdown is
# silently discarded by every `cmd &` below. That matters most for
# `rosbag record`, which finalises its file (index flush + .active rename) only
# from the SIGINT handler ROS installs — it does NOT handle SIGTERM, so the
# alternative of signalling TERM instead would truncate every recording.
# With job control each background job gets its own process group and the default
# disposition, and SIGINT is delivered as intended.
set -m

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Topics the relay consumes, and therefore the exact set worth recording: a bag
# of these replays into an unmodified relay (PLAY_BAG below), which is what makes
# off-robot retargeting/latency work possible.
BAG_TOPICS=(/quest/pose/headset /tf /quest/hand_pose/left /quest/hand_pose/right)

# PIDs we must shut down gracefully (rosbag record in particular — see below).
CHILD_PIDS=()

# rosbag record only finalises its file on SIGINT: it writes the index and
# renames <name>.bag.active -> <name>.bag. `docker stop` sends SIGTERM to PID 1
# only, so without forwarding it here every recorded bag is left truncated and
# unreadable. Hence: no `exec` for the relay when we have children to reap.
shutdown() {
    # Ignore further signals: without this a second SIGTERM (or the SIGINT we are
    # about to send, if we are in the same process group) re-enters the handler.
    trap '' INT TERM
    local pid i
    for pid in "${CHILD_PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    # Now block until the recording is actually finalised, polling the artefact
    # rather than the processes. Two bash traps make the obvious approaches wrong
    # here: `wait` inside a signal handler hangs (the SIGCHLD for a child reaped
    # during the interrupted `wait` is already consumed), and `kill -0` reports a
    # zombie as alive, so process polling never terminates early either.
    # `rosbag record` writes <name>.bag.active and renames it to <name>.bag once
    # the index is flushed, so "no .active files left" is the precise condition —
    # and it is trivially already true when we are not recording.
    for i in $(seq 1 200); do   # 20s, inside the launcher's 30s docker-stop budget
        set -- /bags/*.active
        [ -e "$1" ] || break
        sleep 0.1
    done
    set -- /bags/*.active
    if [ -e "$1" ]; then
        echo "[quest-relay] WARNING: $1 still present — bag may be truncated" >&2
    fi
    echo "[quest-relay] children stopped."
    # MUST exit here. A bash trap interrupts the current command and then RESUMES
    # the script, so without this a signal arriving during startup (e.g. inside
    # the `sleep 2` below) would tear the children down and then carry on to
    # launch rosbag record and the relay anyway — leaving a half-started
    # container that ignores the stop it was just given.
    exit 143  # 128 + SIGTERM, the conventional signal-terminated status
}
trap shutdown INT TERM

# ROS1 needs a master. Start roscore and wait until it answers before the relay
# (an rospy node) and the endpoint try to register.
roscore &
ROSCORE_PID=$!
until rosnode list >/dev/null 2>&1; do
    sleep 0.2
done
echo "[quest-relay] roscore up (PID $ROSCORE_PID)"

if [ -n "$PLAY_BAG" ]; then
    # ---- Bag playback --------------------------------------------------------
    # No Quest connects, so the ros_tcp_endpoint is pointless; the bag supplies
    # the same topics it would have published. The relay is byte-for-byte the live
    # one, so the manager cannot tell the difference.
    #
    # Playback is NOT started here. The manager kicks it off when teleop engages
    # (its second 's') via /start_bag.sh, so the recording's first frame is the
    # frame calibration reads — which is what makes the bag play out relative to
    # the calibration pose instead of from an arbitrary mid-recording anchor.
    if [ ! -f "/bags/$PLAY_BAG" ]; then
        echo "[quest-relay] ERROR: /bags/$PLAY_BAG not found (is the bag mounted?)" >&2
        exit 1
    fi
    PLAY_ARGS=()
    [ -n "$BAG_LOOP" ] && PLAY_ARGS+=(--loop)
    echo "[quest-relay] PLAY_BAG=$PLAY_BAG — the manager will start playback."

    cat > /start_bag.sh <<PLAYER
#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec rosbag play ${PLAY_ARGS[*]} "/bags/$PLAY_BAG"
PLAYER
    chmod +x /start_bag.sh
else
    # ---- Live Quest ----------------------------------------------------------
    # Start ros_tcp_endpoint in background (bridges Quest Unity TCP → ROS1 topics).
    # endpoint_no_adb.launch omits the adb_reverse node (Quest connects over TCP).
    roslaunch ros_tcp_endpoint endpoint_no_adb.launch tcp_ip:=0.0.0.0 tcp_port:=10000 &
    ROS_ENDPOINT_PID=$!
    CHILD_PIDS+=("$ROS_ENDPOINT_PID")

    # Give the endpoint a moment to initialize before the relay starts subscribing
    sleep 2

    echo "[quest-relay] ros_tcp_endpoint started (PID $ROS_ENDPOINT_PID)"

    if [ -n "$RECORD_BAG" ]; then
        BAG_NAME="${BAG_PREFIX:-quest}_$(date +%Y%m%d_%H%M%S)"
        echo "[quest-relay] RECORD_BAG set — recording ${BAG_TOPICS[*]} to /bags/$BAG_NAME.bag"
        rosbag record -O "/bags/$BAG_NAME.bag" "${BAG_TOPICS[@]}" &
        BAG_RECORD_PID=$!
        CHILD_PIDS+=("$BAG_RECORD_PID")
        echo "[quest-relay] rosbag record started (PID $BAG_RECORD_PID)"
    fi
fi

# Optional: robot ego-view camera → Quest. Started only when CAMERA_HOST is set,
# so the default relay is unchanged. Subscribes ZMQ to the composed camera server
# and republishes the ego_view JPEG as a ROS1 CompressedImage the Quest can show.
if [ -n "$CAMERA_HOST" ]; then
    echo "[quest-relay] CAMERA_HOST=$CAMERA_HOST — starting image relay..."
    python3 -u /image_relay.py --camera-host "$CAMERA_HOST" \
        ${CAMERA_PORT:+--camera-port "$CAMERA_PORT"} \
        ${IMAGE_RELAY_FPS:+--fps "$IMAGE_RELAY_FPS"} &
    IMAGE_RELAY_PID=$!
    CHILD_PIDS+=("$IMAGE_RELAY_PID")
    echo "[quest-relay] image relay started (PID $IMAGE_RELAY_PID)"
fi

echo "[quest-relay] Starting ZMQ relay..."

# -u = unbuffered stdout/stderr so relay logs stream live under `docker run`
# (a pipe, not a TTY) instead of being flushed all at once on exit.
#
# Run in the background and `wait` rather than `exec`: the trap above must stay
# installed so a `docker stop` can finalise a recording bag. `wait` returns as
# soon as a signal is handled, so shutdown() runs, then we wait out the relay.
python3 -u /relay.py "$@" &
RELAY_PID=$!
CHILD_PIDS+=("$RELAY_PID")

# `|| RELAY_RC=$?` because `set -e` would otherwise abort here the moment the
# relay exits non-zero, or a trapped signal makes `wait` return >128, skipping
# both the exit-code capture and the bag finalisation.
RELAY_RC=0
wait "$RELAY_PID" || RELAY_RC=$?
exit "$RELAY_RC"
