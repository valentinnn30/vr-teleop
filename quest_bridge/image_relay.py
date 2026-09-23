"""Robot ego-view camera → Quest, as a ROS1 CompressedImage.

Subscribes (ZMQ SUB + CONFLATE) to the composed camera server's msgpack stream
(``gear_sonic/camera/sensor_server.py`` ``ImageMessageSchema``), pulls the
``ego_view`` JPEG **without decoding/re-encoding**, and republishes it as
``sensor_msgs/CompressedImage`` (``format="jpeg"``) on a topic ending in
``compressed``. The Unity app's ``ImageView`` auto-discovers any such topic and
can head-lock it, so the operator sees the robot's POV in the headset.

Runs inside the quest_relay container (ROS1 Noetic + roscore from entrypoint.sh),
started only when ``CAMERA_HOST`` is set. It talks ZMQ to the camera server, so
it needs neither the gear_sonic package nor any camera SDK — only the on-wire
msgpack contract, which is:

    msgpack({"timestamps": {key: float, ...},
             "images":     {key: <raw JPEG bytes | base64-JPEG str>, ...}})

Data flow:
    composed_camera server --ZMQ--> image_relay --ROS1--> ros_tcp_endpoint --> Quest
"""

import argparse
import base64
import time

import msgpack
import rospy
import zmq
from sensor_msgs.msg import CompressedImage


def extract_jpeg(data: dict, key: str):
    """Return the JPEG bytes for ``key`` from an unpacked ImageMessageSchema
    message, or None if absent/unsupported. Takes the encoded bytes as-is (raw
    bytes from an on-device encoder, or a legacy base64 string) — no cv2
    decode/re-encode.
    """
    jpeg = data.get("images", {}).get(key)
    if jpeg is None:
        return None
    if isinstance(jpeg, str):
        return base64.b64decode(jpeg)
    if isinstance(jpeg, (bytes, bytearray)):
        return bytes(jpeg)
    return None


def parse_args():
    p = argparse.ArgumentParser(
        description="Relay the robot ego-view JPEG from the camera ZMQ server to a "
        "ROS1 CompressedImage topic for the Quest.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--camera-host", required=True, help="Host/IP of the composed camera ZMQ server.")
    p.add_argument("--camera-port", type=int, default=5555, help="Camera server ZMQ port.")
    p.add_argument(
        "--image-key", default="ego_view",
        help="Key in the ImageMessageSchema 'images' dict to forward.",
    )
    p.add_argument(
        "--topic", default="/robot/ego_view/image/compressed",
        help="ROS1 CompressedImage topic (must end in 'compressed' for Unity auto-discovery).",
    )
    p.add_argument("--fps", type=float, default=30.0, help="Max publish rate; excess frames dropped.")
    return p.parse_args()


def main():
    args = parse_args()
    rospy.init_node("image_relay", anonymous=True, disable_signals=True)
    pub = rospy.Publisher(args.topic, CompressedImage, queue_size=1)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.CONFLATE, True)  # latest frame only; no backlog
    sub.setsockopt(zmq.RCVHWM, 1)
    sub.connect(f"tcp://{args.camera_host}:{args.camera_port}")
    rospy.loginfo(
        f"[image_relay] SUB tcp://{args.camera_host}:{args.camera_port} "
        f"key='{args.image_key}' -> {args.topic} @ <= {args.fps:.0f} Hz"
    )

    min_period = 1.0 / max(1.0, args.fps)
    last_pub = 0.0
    published = 0
    missing_logged = False
    last_log = time.time()

    try:
        while not rospy.is_shutdown():
            if not sub.poll(200):  # ms; loop back to re-check rospy shutdown
                continue
            packed = sub.recv()
            now = time.monotonic()
            if now - last_pub < min_period:  # rate cap: drop this frame
                continue

            data = msgpack.unpackb(packed, raw=False)
            jpeg = extract_jpeg(data, args.image_key)
            if jpeg is None:
                if not missing_logged:
                    rospy.logwarn(
                        f"[image_relay] key '{args.image_key}' not in frame; "
                        f"available: {list(data.get('images', {}).keys())}"
                    )
                    missing_logged = True
                continue

            msg = CompressedImage()
            ts = data.get("timestamps", {}).get(args.image_key)
            msg.header.stamp = rospy.Time.from_sec(ts) if ts else rospy.Time.now()
            msg.format = "jpeg"
            msg.data = bytes(jpeg)
            pub.publish(msg)
            last_pub = now
            published += 1

            wall = time.time()
            if wall - last_log >= 5.0:
                rospy.loginfo(f"[image_relay] {published / (wall - last_log):.1f} img/s")
                last_log = wall
                published = 0
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
