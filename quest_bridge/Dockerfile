# Quest ROS1 → ZMQ relay image.
#
# Build context must be the repo ROOT so we can COPY from both third_party/
# (the two ROS packages, as submodules) and quest_bridge/ (the relay itself).
#
# Preferred entry point — the launcher handles build, run, port mapping,
# log streaming, and teardown for you:
#   python3 quest_bridge/run_quest_relay.py
#
# Manual equivalent (from the repo root), if you'd rather drive docker yourself:
#   docker build -t quest-relay -f quest_bridge/Dockerfile .
#   docker run --rm -p 10000:10000 -p 5559:5559 quest-relay

FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Build tooling + message deps. ROS1 uses a ROS master + TCPROS (no DDS), so no
# CycloneDDS machinery is needed. geometry_msgs/tf2_msgs are required because the
# relay subscribes to the head pose and /tf; std_msgs/sensor_msgs/message_generation
# and libeigen3-dev are what third_party/vr_haptic_msgs asks for in its ROS1
# find_package branch. rosbag is used for the record/replay modes in entrypoint.sh.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-geometry-msgs \
    ros-noetic-tf2-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-message-generation \
    ros-noetic-rosbag \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Build the two vendored ROS packages in a catkin workspace. Both come from
# third_party/ submodules — this image is the only place they are needed, and
# nothing in this repo imports them.
WORKDIR /catkin_ws
COPY third_party/ROS-TCP-Endpoint/ src/ros_tcp_endpoint/
# Upstream vr_haptic_msgs is dual-build: its CMakeLists switches on $ROS_VERSION,
# which `source /opt/ros/noetic/setup.sh` below sets to 1, selecting the catkin
# branch. No repo-local packaging needed.
COPY third_party/vr_haptic_msgs/ src/vr_haptic_msgs/
# Container launch variant without the adb_reverse node (Quest connects over TCP).
COPY quest_bridge/endpoint_no_adb.launch src/ros_tcp_endpoint/launch/endpoint_no_adb.launch

RUN . /opt/ros/noetic/setup.sh \
    && catkin_make \
    && rm -rf build

# The base image is Python 3.8 (EOL). Newer pyzmq (>=26) no longer ships cp38
# aarch64 wheels, so pip would build from source and fail on a
# scikit-build-core/numpy incompatibility. Pin to a version that still has a
# prebuilt manylinux aarch64 cp38 wheel.
RUN pip3 install --no-cache-dir "pyzmq==25.1.2" msgpack

COPY quest_bridge/relay.py /relay.py
# Optional robot ego-view -> Quest image relay; started by entrypoint.sh when CAMERA_HOST is set.
COPY quest_bridge/image_relay.py /image_relay.py
COPY quest_bridge/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 10000 = ROS TCP Connector (Quest Unity app connects here)
# 5559  = ZMQ relay output (host quest_manager connects here)
EXPOSE 10000
EXPOSE 5559

ENTRYPOINT ["/entrypoint.sh"]
