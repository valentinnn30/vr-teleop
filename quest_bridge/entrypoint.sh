#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# ROS1 needs a master. Start roscore and wait until it answers before the relay
# (an rospy node) and the endpoint try to register.
roscore &
ROSCORE_PID=$!
until rosnode list >/dev/null 2>&1; do
    sleep 0.2
done
echo "[quest-relay] roscore up (PID $ROSCORE_PID)"

# Start ros_tcp_endpoint in background (bridges Quest Unity TCP → ROS1 topics).
# endpoint_no_adb.launch omits the adb_reverse node (Quest connects over TCP).
roslaunch ros_tcp_endpoint endpoint_no_adb.launch tcp_ip:=0.0.0.0 tcp_port:=10000 &
ROS_ENDPOINT_PID=$!

# Give the endpoint a moment to initialize before the relay starts subscribing
sleep 2

echo "[quest-relay] ros_tcp_endpoint started (PID $ROS_ENDPOINT_PID)"

# Optional: robot ego-view camera → Quest. Started only when CAMERA_HOST is set,
# so the default relay is unchanged. Subscribes ZMQ to the composed camera server
# and republishes the ego_view JPEG as a ROS1 CompressedImage the Quest can show.
if [ -n "$CAMERA_HOST" ]; then
    echo "[quest-relay] CAMERA_HOST=$CAMERA_HOST — starting image relay..."
    python3 -u /image_relay.py --camera-host "$CAMERA_HOST" \
        ${CAMERA_PORT:+--camera-port "$CAMERA_PORT"} \
        ${IMAGE_RELAY_FPS:+--fps "$IMAGE_RELAY_FPS"} &
    echo "[quest-relay] image relay started (PID $!)"
fi

echo "[quest-relay] Starting ZMQ relay..."

# -u = unbuffered stdout/stderr so relay logs stream live under `docker run`
# (a pipe, not a TTY) instead of being flushed all at once on exit.
exec python3 -u /relay.py "$@"
