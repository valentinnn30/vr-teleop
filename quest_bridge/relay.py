"""
Quest ROS1 → ZMQ relay.

Subscribes to Meta Quest ROS1 topics published via ROS TCP Connector and
republishes the data as a single msgpack blob over a ZMQ PUB socket.

The host-side QuestReader (in quest_manager_thread_server.py) connects as
a ZMQ SUB to consume this data without needing any ROS installation.

ZMQ message format: two-part multipart
  part[0] = b"quest_data"
  part[1] = msgpack.packb({
      "head_pos":          [x, y, z],        # float64, FLU frame
      "head_quat":         [w, x, y, z],     # float64, scalar-first
      "left_wrist_pos":    [x, y, z],        # float64, FLU frame
      "left_wrist_quat":   [w, x, y, z],     # float64, scalar-first
      "right_wrist_pos":   [x, y, z],        # float64
      "right_wrist_quat":  [w, x, y, z],     # float64, scalar-first
      "left_landmarks":    [[x,y,z], ...],   # float64, 21 MANO joints
      "right_landmarks":   [[x,y,z], ...],   # float64, 21 MANO joints
      "left_tracked":      bool,
      "right_tracked":     bool,
      "timestamp":         float,            # time.time()
  })

Subscribed topics (all configurable via CLI):
  /quest/pose/headset        geometry_msgs/PoseStamped      head pose
  /tf                        tf2_msgs/TFMessage             wrist poses
  /quest/hand_pose/left      vr_haptic_msgs/ManoLandmarks   left hand (21 MANO joints)
  /quest/hand_pose/right     vr_haptic_msgs/ManoLandmarks   right hand (21 MANO joints)
"""

import argparse
import threading
import time

import msgpack
import rospy
import zmq
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from vr_haptic_msgs.msg import ManoLandmarks

_IDENTITY_QUAT = [1.0, 0.0, 0.0, 0.0]
_ZERO_POS = [0.0, 0.0, 0.0]
_ZERO_LANDMARKS = [[0.0, 0.0, 0.0]] * 21


class QuestRelayNode:
    """ROS1 node that subscribes to Quest topics and exposes a thread-safe snapshot."""

    def __init__(self, head_topic, left_hand_topic, right_hand_topic):
        self._lock = threading.Lock()
        # Set by the headset callback so the publish loop can wake and emit
        # immediately on a fresh pose instead of sampling on a fixed timer
        # (publish-on-receipt); the timer remains a fallback for stalled streams.
        self._new_pose = threading.Event()
        self._state = {
            "head_pos": list(_ZERO_POS),
            "head_quat": list(_IDENTITY_QUAT),
            "left_wrist_pos": list(_ZERO_POS),
            "left_wrist_quat": list(_IDENTITY_QUAT),
            "right_wrist_pos": list(_ZERO_POS),
            "right_wrist_quat": list(_IDENTITY_QUAT),
            "left_landmarks": list(_ZERO_LANDMARKS),
            "right_landmarks": list(_ZERO_LANDMARKS),
            "left_tracked": False,
            "right_tracked": False,
            "timestamp": 0.0,
        }

        rospy.Subscriber(head_topic, PoseStamped, self._on_headset, queue_size=1)
        rospy.Subscriber("/tf", TFMessage, self._on_tf, queue_size=1)
        rospy.Subscriber(left_hand_topic, ManoLandmarks, self._on_left_hand, queue_size=1)
        rospy.Subscriber(right_hand_topic, ManoLandmarks, self._on_right_hand, queue_size=1)

        rospy.loginfo(f"Subscribed to: {head_topic}, /tf, {left_hand_topic}, {right_hand_topic}")

    def _on_headset(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        with self._lock:
            self._state["head_pos"] = [p.x, p.y, p.z]
            # geometry_msgs quaternion is scalar-last [x,y,z,w]; convert to scalar-first [w,x,y,z]
            self._state["head_quat"] = [o.w, o.x, o.y, o.z]
            self._state["timestamp"] = time.time()
        self._new_pose.set()  # wake the publish loop (headset paces publishing)

    def wait_for_pose(self, timeout: float) -> bool:
        """Block until a fresh headset pose arrives or ``timeout`` s elapse.

        Returns True if woken by a pose (publish-on-receipt), False on the
        fallback timeout. Clears the flag so the next call blocks again.
        """
        fired = self._new_pose.wait(timeout)
        self._new_pose.clear()
        return fired

    def _on_tf(self, msg: TFMessage) -> None:
        updates = {}
        for t in msg.transforms:
            child = t.child_frame_id
            if child not in ("hand_left", "hand_right"):
                continue
            p = t.transform.translation
            o = t.transform.rotation
            pos = [p.x, p.y, p.z]
            if abs(o.x) + abs(o.y) + abs(o.z) + abs(o.w) < 1e-6:
                quat = list(_IDENTITY_QUAT)  # undetected hand → identity
            else:
                quat = [o.w, o.x, o.y, o.z]  # scalar-first
            if child == "hand_left":
                updates["left_wrist_pos"] = pos
                updates["left_wrist_quat"] = quat
            else:
                updates["right_wrist_pos"] = pos
                updates["right_wrist_quat"] = quat
        if updates:
            with self._lock:
                self._state.update(updates)
                self._state["timestamp"] = time.time()

    def _on_left_hand(self, msg: ManoLandmarks) -> None:
        landmarks = _mano_to_list(msg)
        with self._lock:
            self._state["left_landmarks"] = landmarks
            self._state["left_tracked"] = len(msg.landmarks) >= 21
            self._state["timestamp"] = time.time()

    def _on_right_hand(self, msg: ManoLandmarks) -> None:
        landmarks = _mano_to_list(msg)
        with self._lock:
            self._state["right_landmarks"] = landmarks
            self._state["right_tracked"] = len(msg.landmarks) >= 21
            self._state["timestamp"] = time.time()

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._state)


def _mano_to_list(msg: ManoLandmarks) -> list:
    """Convert ManoLandmarks to list of 21 [x,y,z] triples, padded if needed."""
    pts = msg.landmarks
    n = min(len(pts), 21)
    result = [[float(p.x), float(p.y), float(p.z)] for p in pts[:n]]
    result += [[0.0, 0.0, 0.0]] * (21 - n)
    return result


def main():
    parser = argparse.ArgumentParser(description="Quest ROS1 → ZMQ relay")
    parser.add_argument("--zmq-port", type=int, default=5559, help="ZMQ PUB port (default: 5559)")
    parser.add_argument(
        "--head-topic",
        default="/quest/pose/headset",
        help="ROS1 head pose topic (default: /quest/pose/headset)",
    )
    parser.add_argument(
        "--left-hand-topic",
        default="/quest/hand_pose/left",
        help="ROS1 left ManoLandmarks topic (default: /quest/hand_pose/left)",
    )
    parser.add_argument(
        "--right-hand-topic",
        default="/quest/hand_pose/right",
        help="ROS1 right ManoLandmarks topic (default: /quest/hand_pose/right)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=90.0,
        help="Fallback ZMQ publish rate in Hz when the headset stream stalls "
        "(default: 90). Normally the relay publishes on-receipt of each headset "
        "pose, so this only bounds the idle republish rate.",
    )
    args = parser.parse_args()

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://0.0.0.0:{args.zmq_port}")
    print(f"[relay] ZMQ PUB bound to port {args.zmq_port}")

    # disable_signals=True so our own KeyboardInterrupt/finally teardown below runs.
    rospy.init_node("quest_relay", anonymous=True, disable_signals=True)
    node = QuestRelayNode(
        head_topic=args.head_topic,
        left_hand_topic=args.left_hand_topic,
        right_hand_topic=args.right_hand_topic,
    )
    # In ROS1 rospy.Subscriber callbacks are serviced by background threads
    # automatically — no explicit spin thread is needed to receive messages.
    print("[relay] ROS1 node up. Waiting for Quest data...")

    # Fallback period: publish at least this often even if the headset stream
    # stalls. Normally we publish-on-receipt, driven by wait_for_pose() below.
    fallback_period = 1.0 / max(1.0, args.hz)
    topic_bytes = b"quest_data"
    last_log = time.time()
    published = 0

    try:
        while True:
            # Block until a fresh headset pose arrives (publish immediately, no
            # sample-and-hold quantization) or the fallback period elapses.
            node.wait_for_pose(fallback_period)

            snap = node.snapshot()
            pub.send_multipart([topic_bytes, msgpack.packb(snap, use_bin_type=True)])
            published += 1

            now = time.time()
            if now - last_log >= 5.0:
                print(
                    f"[relay] {published / (now - last_log):.1f} Hz | "
                    f"head={'ok' if snap['timestamp'] > 0.0 else 'waiting'} | "
                    f"L={'ok' if snap['left_tracked'] else '-'} "
                    f"R={'ok' if snap['right_tracked'] else '-'}"
                )
                # Sample a couple of values per topic so you can eyeball that real
                # data (not zeros) is flowing without flooding the log.
                hp = snap["head_pos"]
                lw = snap["left_wrist_pos"]
                rw = snap["right_wrist_pos"]
                print(
                    f"[relay]   sample | head_pos=({hp[0]:+.3f}, {hp[1]:+.3f}, {hp[2]:+.3f}) "
                    f"| L_wrist=({lw[0]:+.3f}, {lw[1]:+.3f}) "
                    f"L_lm0={tuple(round(v, 3) for v in snap['left_landmarks'][0])} "
                    f"| R_wrist=({rw[0]:+.3f}, {rw[1]:+.3f}) "
                    f"R_lm0={tuple(round(v, 3) for v in snap['right_landmarks'][0])}"
                )
                last_log = now
                published = 0

    except KeyboardInterrupt:
        print("\n[relay] Shutting down...")
    finally:
        pub.close()
        ctx.term()


if __name__ == "__main__":
    main()
