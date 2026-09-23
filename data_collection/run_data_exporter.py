"""
Sonic VLA data exporter for G1 -- NO ROS 2 DEPENDENCY.

All data sources use ZMQ:
  1. Robot state  -> ZMQ SUB on ``g1_debug`` topic (port 5557, from C++ zmq_output_handler)
  2. SMPL pose    -> ZMQ SUB on ``pose`` topic     (port 5556, from quest_manager.py)
  3. Camera       -> ZMQ/TCP via ComposedCameraClientSensor

Robot config (``script_config`` in info.json) is read from the ``robot_config``
ZMQ topic re-published every ~2 s by the C++ process.  If the config is not
received within the timeout the exporter exits with an error.

Virtual environment setup (run from repo root):
    bash install_scripts/install_data_collection.sh
    source .venv_data_collection/bin/activate

Usage (from repo root):
    python data_collection/run_data_exporter.py --task-prompt "pick up the cup"
    python data_collection/run_data_exporter.py --task-prompt "walk forward" --dataset-name my_session
"""

from collections import deque
from dataclasses import dataclass
from datetime import datetime
import time
from typing import Callable, Literal

import msgpack
import numpy as np
from scipy.spatial.transform import Rotation as R
import tyro
import zmq

from data_collection.exporter import Gr00tDataExporter
from data_collection.schema import (
    get_features_sonic_vla,
    get_g1_robot_model,
    get_modality_config_sonic_vla,
    get_wrist_camera_features,
    get_wrist_camera_modality_config,
)
from data_collection.camera.composed_camera import ComposedCameraClientSensor
from data_collection.writers.episode_state import EpisodeState
from data_collection.writers.keyboard_subscriber import ZMQKeyboardSubscriber
from data_collection.writers.telemetry import Telemetry
from data_collection.writers.text_to_speech import TextToSpeech
from data_collection.writers.transforms import compute_projected_gravity, quat_to_rot6d
from data_collection.writers.zmq_state_subscriber import (
    ZMQStateSubscriber,
    poll_robot_config_zmq,
)
from teleop_manager.zmq_io.zmq_planner_sender import unpack_pose_message

# object_gt_writer.py is sim-only and was dropped (see docs/ROBOT_TELEOP_QUICKSTART.md /
# plan section A.1); foundation_pose_writer.py is deferred, not yet wired up (kept as
# writers/foundation_pose_writer.py.deferred). Both resolve to None here so the code below
# no-ops instead of crashing; re-enable foundation pose by restoring the .py extension and
# removing this try/except.
ObjectGtWriter = None
try:
    from data_collection.writers.foundation_pose_writer import FoundationPoseWriter
except ImportError:
    FoundationPoseWriter = None

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass
class SonicDataExporterConfig:
    """CLI config for the ROS-free Sonic data exporter."""

    # Dataset
    dataset_name: str | None = None
    """Dataset name (auto-generated if creating new)."""

    task_prompt: str = "demo"
    """Language task prompt."""

    root_output_dir: str = "outputs"
    """Root output directory."""

    data_collection_frequency: int = 50
    """Data collection frequency (Hz)."""


    # Camera
    camera_host: str = "localhost"
    """Camera server host."""

    camera_port: int = 5555
    """Camera server port."""

    # ZMQ: Sonic / SMPL pose (from quest_manager.py)
    sonic_zmq_host: str = "localhost"
    """ZMQ host for Sonic SMPL pose messages."""

    sonic_zmq_port: int = 5556
    """ZMQ port for Sonic SMPL pose messages."""

    # ZMQ: Robot state (from C++ zmq_output_handler, g1_debug topic)
    state_zmq_host: str = "localhost"
    """ZMQ host for robot state (g1_debug topic from C++ deploy)."""

    state_zmq_port: int = 5557
    """ZMQ port for robot state (same socket as robot_config topic)."""

    # Robot config
    robot_config_timeout: float = 0
    """Seconds to wait for the ZMQ robot_config message at startup (0 = wait forever)."""

    record_wrist_cameras: bool = False
    """Record wrist camera streams (left_wrist, right_wrist). Requires cameras to be available."""

    text_to_speech: bool = True
    """Use text-to-speech voice feedback."""

    hand_type: Literal["dex3", "brainco"] = "dex3"
    """Hand type: 'dex3' (7 DOF, Unitree DEX3) or 'brainco' (6 DOF, BrainCo Revo2)."""

    skip_robot_state: bool = False
    """Skip waiting for robot proprioception (robot_config + g1_debug state); use zero-filled
    dummy state instead. For testing camera/depth recording without the robot's C++ deploy
    process running."""

    auto_record: bool = False
    """Skip keyboard/ZMQ recording toggle controls and start recording immediately at launch.
    Stop with Ctrl+C to save the episode."""

    # ZMQ: ground-truth box pose (from the sim's --record-box-gt publisher)
    record_object_gt: bool = False
    """Record the box's exact pose from the sim into a separate object_gt/ parquet, in
    parallel to FoundationPose (for comparison / direct use in replay). Sim-only."""

    box_gt_zmq_host: str = "localhost"
    """ZMQ host for the sim's ground-truth box-pose publisher (see --record-object-gt)."""

    box_gt_zmq_port: int = 5560
    """ZMQ port for the sim's ground-truth box-pose publisher."""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class TimeDeltaException(Exception):
    def __init__(self, failure_count: int, reset_timeout_sec: float):
        self.failure_count = failure_count
        self.reset_timeout_sec = reset_timeout_sec
        self.message = f"{self.failure_count} failures in {self.reset_timeout_sec} seconds"
        super().__init__(self.message)


class TimingThresholdMonitor:
    def __init__(self, max_failures=3, reset_timeout_sec=5, time_delta=0.2, raise_exception=False):
        self.max_failures = max_failures
        self.reset_timeout_sec = reset_timeout_sec
        self.failure_count = 0
        self.last_failure_time = 0
        self.time_delta = time_delta
        self.raise_exception = raise_exception

    def reset(self):
        self.failure_count = 0
        self.last_failure_time = 0

    def log_time_delta(self, time_delta_sec: float):
        time_delta = abs(time_delta_sec)
        if time_delta > self.time_delta:
            self.failure_count += 1
            self.last_failure_time = time.monotonic()

        if self.is_threshold_exceeded():
            print(
                f"Time delta exception: {self.failure_count} failures in "
                f"{self.reset_timeout_sec} seconds, time delta: {time_delta}"
            )
            if self.raise_exception:
                raise TimeDeltaException(self.failure_count, self.reset_timeout_sec)

    def is_threshold_exceeded(self):
        if self.failure_count >= self.max_failures:
            return True
        if time.monotonic() - self.last_failure_time > self.reset_timeout_sec:
            self.reset()
        return False


# ---------------------------------------------------------------------------
# Data Collector
# ---------------------------------------------------------------------------


class GrootDataCollector:
    """Collects data from G1 robot in Sonic CPP + SMPL mode -- no ROS 2.

    Data sources (all ZMQ):
      - ``g1_debug`` topic        -> proprio (body_q, hand_q, actions, base_quat, ...)
      - ``pose`` topic            -> SMPL pose (smpl_joints, body_quat_w, hand_joints, ...)
      - ``planner`` topic         -> planner commands (vr_position, vr_orientation, ...)
      - ``manager_state`` topic   -> current stream mode + toggle flags
      - Camera client             -> ego-view images
    """

    def __init__(
        self,
        camera_host: str,
        camera_port: int,
        exporter_factory: Callable[[], Gr00tDataExporter],
        robot_model,
        text_to_speech=None,
        frequency: int = 20,
        sonic_data_zmq_host: str = "localhost",
        sonic_data_zmq_port: int = 5556,
        state_zmq_host: str = "localhost",
        state_zmq_port: int = 5557,
        skip_robot_state: bool = False,
        auto_record: bool = False,
        record_object_gt: bool = False,
        box_gt_zmq_host: str = "localhost",
        box_gt_zmq_port: int = 5560,
        hand_type: str = "dex3",
    ):
        self.text_to_speech = text_to_speech
        self.frequency = frequency
        self.loop_period = 1.0 / frequency
        # The LeRobot dataset is created lazily on the first recording so that merely
        # launching the recorder does not litter an empty dataset folder on disk.
        self._exporter_factory = exporter_factory
        self.data_exporter: Gr00tDataExporter | None = None
        self.robot_model = robot_model
        self._skip_robot_state = skip_robot_state
        self._auto_record = auto_record

        # BrainCo hand STATE and COMMANDS arrive normalized [0, 1] (the sim bridge publishes
        # `(q - lower) / span`, and the deploy passes that straight through as *_hand_q /
        # *_hand_action). Left unmapped, whole_q would store fractions where the body joints are
        # radians, so any FK under-curls the fingers (~1.47x for the 1.466 rad finger range). We
        # map hand values back to radians here so observation.state / action.wbc are uniformly in
        # radians. Dex3 already reports radians, so this is a no-op there. teleop.*_hand_joints is
        # left as the raw [0,1] command on purpose (that IS the command representation).
        self._denorm_hands = hand_type == "brainco"
        self._hand_limits: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        if self._denorm_hands:
            lower = np.asarray(self.robot_model.lower_joint_limits, dtype=np.float64)
            upper = np.asarray(self.robot_model.upper_joint_limits, dtype=np.float64)
            for side in ("left", "right"):
                act = list(self.robot_model.get_hand_actuated_joint_indices(side))
                self._hand_limits[side] = (lower[act], upper[act])

        self._episode_state = EpisodeState()
        self._keyboard_listener = ZMQKeyboardSubscriber()

        # FoundationPose export: active only when the sim streams ego depth/seg
        # (i.e. run_sim_loop launched with --render-depth-seg). Depth/seg arrive at a
        # reduced rate, so track whether the stream exists at all and de-dupe by timestamp.
        # Created alongside the exporter in _ensure_exporter (needs the dataset root).
        self._fp_writer: FoundationPoseWriter | None = None
        self._fp_enabled = False
        self._fp_last_written_ts = None

        # Ground-truth box pose (sim-only): a separate parquet written in parallel to
        # FoundationPose. Writer is created alongside the exporter; poses are buffered by a
        # background subscriber to the sim's box-GT publisher and time-matched to each proprio
        # row (see _write_object_gt_frame). We keep a short *history* (not CONFLATE) so we can
        # pick the box pose contemporaneous with the proprio frame: the box-GT stream comes
        # straight from the sim and is fresher than proprio (which crosses the deploy pipeline),
        # so blindly taking "latest" would place the cube slightly ahead of the robot.
        self._record_object_gt = record_object_gt
        self._gt_writer: ObjectGtWriter | None = None
        self.latest_box_gt = None
        self._box_gt_buffer: deque = deque(maxlen=512)  # (timestamp, msg), oldest -> newest
        self._box_gt_socket = None
        self._box_gt_ctx = None
        if record_object_gt:
            try:
                self._box_gt_ctx = zmq.Context()
                self._box_gt_socket = self._box_gt_ctx.socket(zmq.SUB)
                self._box_gt_socket.connect(f"tcp://{box_gt_zmq_host}:{box_gt_zmq_port}")
                self._box_gt_socket.setsockopt(zmq.RCVTIMEO, 100)
                # Default RCVHWM (bounded) is fine — no CONFLATE, so a short history of poses
                # accumulates between polls; the loop drains it fully each iteration.
                self._box_gt_socket.setsockopt_string(zmq.SUBSCRIBE, "")
                print(f"[GT] Subscribed to box ground-truth at {box_gt_zmq_host}:{box_gt_zmq_port}")
            except Exception as e:
                print(f"[GT] Warning: Failed to initialize box-GT subscriber: {e}")
                self._box_gt_socket = None

        self._image_subscriber = ComposedCameraClientSensor(server_ip=camera_host, port=camera_port)

        self.obs_act_buffer = deque(maxlen=100)
        self.latest_image_msg = None
        self.latest_proprio_msg = None
        self.latest_sonic_msg = None
        self.latest_planner_msg = None

        self.current_stream_mode = 0

        self._manager_toggle_dc = False
        self._manager_toggle_da = False

        self._state_subscriber = ZMQStateSubscriber(
            host=state_zmq_host,
            port=state_zmq_port,
        )

        self._sonic_zmq_ctx = None
        self._sonic_zmq_socket = None
        try:
            self._sonic_zmq_ctx = zmq.Context()
            self._sonic_zmq_socket = self._sonic_zmq_ctx.socket(zmq.SUB)
            self._sonic_zmq_socket.connect(f"tcp://{sonic_data_zmq_host}:{sonic_data_zmq_port}")
            self._sonic_zmq_socket.setsockopt(zmq.RCVTIMEO, 100)
            self._sonic_zmq_socket.setsockopt(zmq.CONFLATE, 0)
            self._sonic_zmq_socket.setsockopt(zmq.RCVHWM, 20)
            self._sonic_zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "pose")
            self._sonic_zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "planner")
            self._sonic_zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "manager_state")
            time.sleep(0.5)
            print(f"[Sonic] Connected to ZMQ at {sonic_data_zmq_host}:{sonic_data_zmq_port}")
            print("[Sonic] Subscribed to: pose, planner, manager_state")
        except Exception as e:
            print(f"[Sonic] Warning: Failed to initialize ZMQ subscriber: {e}")
            self._sonic_zmq_socket = None

        self.telemetry = Telemetry(window_size=100)
        self.sonic_timing_monitor = TimingThresholdMonitor(
            max_failures=3, reset_timeout_sec=5, time_delta=0.1
        )

        self._last_latency_log_time = 0.0
        self._initial_yaw = None

        print("Data exporter ready — dataset folder is created on the first recording.")

        if self._auto_record:
            self._ensure_exporter()
            self._episode_state.change_state()  # IDLE -> RECORDING
            self._print_and_say(
                f"Auto-record enabled, started recording {self.current_episode_index}",
                blocking=False,
            )

    @property
    def current_episode_index(self):
        if self.data_exporter is None:
            return 0
        return self.data_exporter.episode_buffer["episode_index"]

    def _ensure_exporter(self) -> Gr00tDataExporter:
        """Create the LeRobot dataset (and FoundationPose writer) on first use.

        Deferred from __init__ so the dataset folder appears on disk only once
        recording actually starts, not merely when the recorder is launched.
        """
        if self.data_exporter is None:
            self.data_exporter = self._exporter_factory()
            if FoundationPoseWriter is not None:
                self._fp_writer = FoundationPoseWriter(self.data_exporter.meta.root)
            if self._record_object_gt and ObjectGtWriter is not None:
                self._gt_writer = ObjectGtWriter(self.data_exporter.meta.root)
            print(f"Recording to {self.data_exporter.meta.root}")
        return self.data_exporter

    def _print_and_say(self, message: str, say: bool = True, blocking: bool = False):
        if self.text_to_speech is not None:
            self.text_to_speech.print_and_say(message, say, blocking=blocking)
        else:
            print(message)

    def _poll_state_zmq(self):
        """Poll the ``g1_debug`` ZMQ topic for robot state (non-blocking)."""
        msg = self._state_subscriber.get_msg(clear=True)
        if msg is None:
            return

        if msg.get("ros_timestamp", 0.0) == 0.0:
            msg["ros_timestamp"] = time.time()

        self.latest_proprio_msg = msg

    def _check_recording_commands(self):
        """Check keyboard + ZMQ toggle flags for recording commands."""
        if self._auto_record:
            return
        key = self._keyboard_listener.read_msg()

        if self._manager_toggle_da:
            key = "x"
            self._manager_toggle_da = False
        elif self._manager_toggle_dc:
            key = "c"
            self._manager_toggle_dc = False

        if key == "c":
            # Entering RECORDING from IDLE: materialize the dataset on disk now.
            if self._episode_state.get_state() == self._episode_state.IDLE:
                self._ensure_exporter()
            self._episode_state.change_state()
            if self._episode_state.get_state() == self._episode_state.RECORDING:
                self._initial_yaw = None
                self._fp_last_written_ts = None
                self._print_and_say(
                    f"Started recording {self.current_episode_index}", blocking=False
                )
            elif self._episode_state.get_state() == self._episode_state.NEED_TO_SAVE:
                self._print_and_say("Stopping recording, preparing to save", blocking=False)
            elif self._episode_state.get_state() == self._episode_state.IDLE:
                self._print_and_say("Saved episode and back to idle state", blocking=False)
        elif key == "x":
            if self._episode_state.get_state() == self._episode_state.RECORDING:
                # True discard: delete the partial video + drop the buffer so no parquet/video is
                # written (the old save_episode_as_discarded saved everything and merely tagged it).
                self.data_exporter.discard_episode()
                if self._fp_writer is not None:
                    self._fp_writer.discard_episode()
                if self._gt_writer is not None:
                    self._gt_writer.discard_episode()
                self._episode_state.reset_state()
                self._initial_yaw = None
                self._print_and_say("Discarded episode", blocking=False)

    def _poll_sonic_zmq_messages(self):
        """Poll ZMQ for pose, planner, and manager_state messages (non-blocking)."""
        if self._sonic_zmq_socket is None:
            return

        max_polls = 20
        for _ in range(max_polls):
            try:
                raw = self._sonic_zmq_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                break

            if raw.startswith(b"manager_state"):
                self._handle_manager_state(raw)
            elif raw.startswith(b"planner"):
                self._handle_planner_message(raw)
            elif raw.startswith(b"pose"):
                self._handle_pose_message(raw)

    def _handle_manager_state(self, raw: bytes) -> None:
        try:
            data = unpack_pose_message(raw, topic="manager_state")
        except Exception:
            return

        if "stream_mode" in data:
            self.current_stream_mode = int(data["stream_mode"].flat[0])

        if self._extract_bool(data, "toggle_data_collection"):
            self._manager_toggle_dc = True
        if self._extract_bool(data, "toggle_data_abort"):
            self._manager_toggle_da = True

    def _handle_planner_message(self, raw: bytes) -> None:
        try:
            data = unpack_pose_message(raw, topic="planner")
        except Exception:
            return

        planner_mode = int(data["mode"].flat[0]) if "mode" in data else 0
        planner_movement = (
            data["movement"].flatten().astype(np.float32)
            if "movement" in data and data["movement"].size == 3
            else np.zeros(3, dtype=np.float32)
        )
        planner_facing = (
            data["facing"].flatten().astype(np.float32)
            if "facing" in data and data["facing"].size == 3
            else np.array([1.0, 0.0, 0.0], dtype=np.float32)
        )
        planner_speed = float(data["speed"].flat[0]) if "speed" in data else -1.0
        planner_height = float(data["height"].flat[0]) if "height" in data else -1.0

        vr_3pt_position = None
        if "vr_position" in data and data["vr_position"].size == 9:
            vr_3pt_position = data["vr_position"].flatten().astype(np.float32)
        vr_3pt_orientation = None
        if "vr_orientation" in data and data["vr_orientation"].size == 12:
            vr_3pt_orientation = data["vr_orientation"].flatten().astype(np.float32)

        self.latest_planner_msg = {
            "planner_mode": planner_mode,
            "planner_movement": planner_movement,
            "planner_facing": planner_facing,
            "planner_speed": planner_speed,
            "planner_height": planner_height,
            "vr_3pt_position": vr_3pt_position,
            "vr_3pt_orientation": vr_3pt_orientation,
            "left_hand_joints": self._extract_hand_joints(data, "left_hand_joints"),
            "right_hand_joints": self._extract_hand_joints(data, "right_hand_joints"),
            "receive_timestamp": time.time(),
        }

    def _handle_pose_message(self, raw: bytes) -> None:
        G1_L_WRIST_ROLL_IDX = 23
        G1_L_WRIST_PITCH_IDX = 25
        G1_L_WRIST_YAW_IDX = 27
        G1_R_WRIST_ROLL_IDX = 24
        G1_R_WRIST_PITCH_IDX = 26
        G1_R_WRIST_YAW_IDX = 28

        try:
            pose_data = unpack_pose_message(raw, topic="pose")
        except Exception as e:
            print(f"[Sonic] Error unpacking pose message: {e}")
            return

        try:
            if "smpl_joints" not in pose_data or len(pose_data["smpl_joints"].shape) != 3:
                return

            left_wrist_joints = None
            right_wrist_joints = None
            if "joint_pos" in pose_data and len(pose_data["joint_pos"].shape) == 2:
                joint_pos = pose_data["joint_pos"][0]
                left_wrist_joints = np.array(
                    [
                        joint_pos[G1_L_WRIST_ROLL_IDX],
                        joint_pos[G1_L_WRIST_PITCH_IDX],
                        joint_pos[G1_L_WRIST_YAW_IDX],
                    ],
                    dtype=np.float32,
                )
                right_wrist_joints = np.array(
                    [
                        joint_pos[G1_R_WRIST_ROLL_IDX],
                        joint_pos[G1_R_WRIST_PITCH_IDX],
                        joint_pos[G1_R_WRIST_YAW_IDX],
                    ],
                    dtype=np.float32,
                )

            frame_index = None
            if "frame_index" in pose_data:
                frame_index = np.array([pose_data["frame_index"].flat[0]], dtype=np.int64)

            smpl_pose = np.zeros(63, dtype=np.float32)
            if "smpl_pose" in pose_data:
                raw_pose = pose_data["smpl_pose"]
                if raw_pose.ndim == 3:
                    smpl_pose = raw_pose[0].flatten().astype(np.float32)
                elif raw_pose.ndim == 2:
                    smpl_pose = raw_pose.flatten().astype(np.float32)
                elif raw_pose.ndim == 1 and raw_pose.size == 63:
                    smpl_pose = raw_pose.astype(np.float32)

            left_hand_joints = self._extract_hand_joints(pose_data, "left_hand_joints")
            right_hand_joints = self._extract_hand_joints(pose_data, "right_hand_joints")

            vr_3pt_position = None
            if "vr_position" in pose_data and pose_data["vr_position"].size == 9:
                vr_3pt_position = pose_data["vr_position"].flatten().astype(np.float32)
            vr_3pt_orientation = None
            if "vr_orientation" in pose_data and pose_data["vr_orientation"].size == 12:
                vr_3pt_orientation = pose_data["vr_orientation"].flatten().astype(np.float32)

            self.latest_sonic_msg = {
                "smpl_joints": pose_data["smpl_joints"][0],
                "smpl_pose": smpl_pose,
                "body_quat_w": (
                    pose_data["body_quat_w"][0] if "body_quat_w" in pose_data else None
                ),
                "left_hand_joints": left_hand_joints,
                "right_hand_joints": right_hand_joints,
                "left_wrist_joints": left_wrist_joints,
                "right_wrist_joints": right_wrist_joints,
                "vr_3pt_position": vr_3pt_position,
                "vr_3pt_orientation": vr_3pt_orientation,
                "frame_index": frame_index,
                "receive_timestamp": time.time(),
            }
        except Exception as e:
            if not hasattr(self, "_sonic_error_count"):
                self._sonic_error_count = 0
            self._sonic_error_count += 1
            if self._sonic_error_count == 1 or self._sonic_error_count % 100 == 0:
                print(f"[Sonic] Error processing pose message: {e}")

    @staticmethod
    def _extract_hand_joints(pose_data: dict, key: str) -> np.ndarray:
        arr = pose_data.get(key)
        if arr is not None:
            if arr.ndim > 1:
                arr = arr[0]
            return arr.astype(np.float32)
        return np.zeros(7, dtype=np.float32)

    @staticmethod
    def _extract_bool(pose_data: dict, key: str) -> bool:
        val = pose_data.get(key)
        if val is None:
            return False
        if isinstance(val, np.ndarray):
            return bool(val.flat[0])
        return bool(val)

    def _log_latency_periodic(
        self,
        sonic_latency_ms: float | None = None,
    ):
        current_time = time.time()
        if current_time - self._last_latency_log_time >= 1.0:
            self._last_latency_log_time = current_time
            parts = []
            if sonic_latency_ms is not None:
                parts.append(f"Sonic Pose: {sonic_latency_ms:.1f}ms")
            if parts:
                print(f"[Latency] {', '.join(parts)}")

    def _poll_box_gt(self) -> None:
        """Drain all pending box-GT messages into the history buffer (non-blocking)."""
        if self._box_gt_socket is None:
            return
        for _ in range(1024):  # bounded drain: empty the socket each loop
            try:
                raw = self._box_gt_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception:
                break
            try:
                msg = msgpack.unpackb(raw, raw=False)
            except Exception:
                continue
            self.latest_box_gt = msg
            self._box_gt_buffer.append((float(msg.get("timestamp", time.time())), msg))

    def _select_box_gt(self):
        """Return the buffered box-GT message contemporaneous with the current proprio frame.

        The box-GT stream is published straight from the sim, so it is fresher than the proprio
        stream (which crosses the deploy pipeline). Pairing "latest with latest" would therefore
        stamp each robot row with a box pose from slightly *later*, making the replayed cube run
        ahead of the hand. We instead pick the box pose whose wall-clock timestamp is closest to
        the proprio message's ``ros_timestamp``, cancelling that constant offset. If no proprio
        timestamp is available we fall back to the latest pose (previous behaviour).
        """
        if not self._box_gt_buffer:
            return self.latest_box_gt
        proprio = self.latest_proprio_msg or {}
        ref_ts = float(proprio.get("ros_timestamp", 0.0) or 0.0)
        if ref_ts <= 0.0:
            return self._box_gt_buffer[-1][1]
        return min(self._box_gt_buffer, key=lambda item: abs(item[0] - ref_ts))[1]

    def _write_object_gt_frame(self) -> None:
        """Buffer one ground-truth box pose for the current parquet row (sim-only).

        Recorded densely (one row per recorded frame) with the same proprio-row index
        convention as FoundationPose, so ground truth and estimate can be compared at any
        FP frame's row. The pose is time-matched to the proprio row (see _select_box_gt).
        """
        if self._gt_writer is None:
            return
        gt = self._select_box_gt()
        if gt is None:
            return
        ob_in_world = gt.get("ob_in_world")
        ref_in_world = gt.get("ref_in_world")
        if ob_in_world is None or ref_in_world is None:
            return
        proprio_frame_index = max(0, self.data_exporter.episode_buffer.get("size", 1) - 1)
        self._gt_writer.write_frame(
            ob_in_world=ob_in_world,
            ref_in_world=ref_in_world,
            proprio_frame_index=proprio_frame_index,
            timestamp=gt.get("timestamp", time.time()),
            box_half_extents=gt.get("box_half_extents"),
            # Staged mesh asset dir when the sim object isn't a primitive box (--object-asset).
            object_mesh_dir=gt.get("object_mesh_dir"),
            object_name=gt.get("object_name"),
            # Reset-state fields (sim publisher >= this change); older publishers omit them and
            # the writer stores zeros. See new_data_collection_report.md.
            pelvis_in_world=gt.get("pelvis_in_world"),
            base_vel=gt.get("base_vel"),
            object_vel=gt.get("object_vel"),
            joint_vel=gt.get("joint_vel"),
        )

    def _write_foundation_pose_frame(self) -> None:
        """Save one FoundationPose frame (rgb+depth; on frame 0 also cam_K, the object
        mask, box.obj, and/or extrinsics — whichever the stream provides).

        Self-guards: only writes when the current message carries a *fresh* depth frame
        (on the sim renderer depth arrives at a reduced rate; on the real RealSense every
        frame has a matching depth frame, so this just de-dupes on timestamp).
        """
        if self.latest_image_msg is None:
            return
        images = self.latest_image_msg["images"]
        if "ego_view_depth" not in images:
            return

        ts = self.latest_image_msg.get("timestamps", {}).get("ego_view")
        if ts is not None and ts == self._fp_last_written_ts:
            return
        self._fp_last_written_ts = ts

        fp_meta = self.latest_image_msg.get("fp_meta") or {}
        # Row of the proprio/parquet frame added in this same loop iteration
        # (add_frame already incremented "size", so the just-added row is size-1).
        # FP frames may be sparser than proprio, so this is what links each object
        # pose to the exact robot state for camera FK downstream.
        proprio_frame_index = max(0, self.data_exporter.episode_buffer.get("size", 1) - 1)
        self._fp_writer.write_frame(
            rgb=images["ego_view"],
            depth=images["ego_view_depth"],
            cam_K=fp_meta.get("cam_K"),
            # Sim (--render-depth-seg): object mask + box mesh from the seg render.
            mask=images.get("ego_view_seg"),
            box_half_extents=fp_meta.get("box_half_extents"),
            # Real RealSense: depth->color transform (sim leaves these unset).
            cam_extrinsics_R=fp_meta.get("cam_extrinsics_R"),
            cam_extrinsics_t=fp_meta.get("cam_extrinsics_t"),
            proprio_frame_index=proprio_frame_index,
            timestamp=ts,
        )

    def _add_images_to_frame_data(self, frame_data: dict) -> None:
        if self.latest_image_msg is None:
            return
        images = self.latest_image_msg["images"]
        for feature_name, feature_info in self.data_exporter.features.items():
            if feature_info.get("dtype") in ["image", "video"]:
                image_key = feature_name.split(".")[-1]
                if image_key not in images:
                    raise ValueError(
                        f"Required image '{image_key}' for feature '{feature_name}' "
                        f"not found in image message. Available: {list(images.keys())}"
                    )
                frame_data[feature_name] = images[image_key]

    def _finalize_frame(self, t_start: float) -> bool:
        t_end = time.monotonic()
        if t_end - t_start > (1 / self.frequency):
            print(f"DataExporter Missed: {t_end - t_start} sec")

        if self._episode_state.get_state() == self._episode_state.NEED_TO_SAVE:
            buffer_size = self.data_exporter.episode_buffer.get("size", 0)
            if buffer_size > 0:
                self.data_exporter.save_episode()
                self.sonic_timing_monitor.reset()
                self._initial_yaw = None
                if self._fp_writer is not None:
                    self._fp_writer.close_episode()
                if self._gt_writer is not None:
                    self._gt_writer.close_episode()
                self._print_and_say("Finished saving episode")
            else:
                self._print_and_say("Skipping save: no frames collected", say=False)
            self._episode_state.change_state()
        return True

    def _make_dummy_proprio(self) -> dict:
        """Zero-filled proprio message used when --skip-robot-state bypasses the g1_debug stream."""
        n_body = len(self.robot_model.get_body_actuated_joint_indices())
        n_hand = len(self.robot_model.get_hand_actuated_joint_indices("left"))
        zeros_body = np.zeros(n_body, dtype=np.float64)
        zeros_hand = np.zeros(n_hand, dtype=np.float64)
        return {
            "body_q": zeros_body,
            "left_hand_q": zeros_hand,
            "right_hand_q": zeros_hand,
            "last_action": zeros_body,
            "last_left_hand_action": zeros_hand,
            "last_right_hand_action": zeros_hand,
            "base_quat": np.array([1.0, 0.0, 0.0, 0.0]),
            "ros_timestamp": time.time(),
        }

    def _add_data_frame(self):
        t_start = time.monotonic()

        if self._skip_robot_state and self.latest_proprio_msg is None:
            self.latest_proprio_msg = self._make_dummy_proprio()

        if self.latest_proprio_msg is None or self.latest_image_msg is None:
            self._print_and_say(
                f"Waiting for message. "
                f"Avail msg: proprio {self.latest_proprio_msg is not None} | "
                f"image {self.latest_image_msg is not None}",
                say=False,
            )
            return False

        if self._episode_state.get_state() != self._episode_state.RECORDING:
            return self._finalize_frame(t_start)

        return self._add_data_frame_sonic(t_start)

    def _denorm_hand(self, values, side: str) -> np.ndarray:
        """Map BrainCo hand values from normalized [0, 1] back to radians (no-op for Dex3).

        Uses the per-joint ``[lower, upper]`` limits in the actuated-joint order (the order in
        which ``get_configuration_from_actuated_joints`` consumes them), applying the same affine
        the bridge inverts: ``q_rad = lower + norm * (upper - lower)``.
        """
        v = np.asarray(values, dtype=np.float64)
        if not self._denorm_hands:
            return v
        lower, upper = self._hand_limits[side]
        n = min(v.shape[0], lower.shape[0])
        out = v.copy()
        out[:n] = lower[:n] + v[:n] * (upper[:n] - lower[:n])
        return out

    def _add_data_frame_sonic(self, t_start: float) -> bool:
        """Build one data frame in Sonic CPP + SMPL mode."""
        assert self.latest_proprio_msg is not None
        proprio = self.latest_proprio_msg

        whole_q = self.robot_model.get_configuration_from_actuated_joints(
            body_actuated_joint_values=proprio["body_q"],
            left_hand_actuated_joint_values=self._denorm_hand(proprio["left_hand_q"], "left"),
            right_hand_actuated_joint_values=self._denorm_hand(proprio["right_hand_q"], "right"),
        )
        whole_action_wbc = self.robot_model.get_configuration_from_actuated_joints(
            body_actuated_joint_values=proprio["last_action"],
            left_hand_actuated_joint_values=self._denorm_hand(proprio["last_left_hand_action"], "left"),
            right_hand_actuated_joint_values=self._denorm_hand(proprio["last_right_hand_action"], "right"),
        )

        self.robot_model.cache_forward_kinematics(whole_q)
        eef_parts = []
        for side in ["left", "right"]:
            placement = self.robot_model.frame_placement(
                self.robot_model.supplemental_info.hand_frame_names[side]
            )
            pos = placement.translation[:3]
            quat = R.from_matrix(placement.rotation).as_quat(scalar_first=True)
            eef_parts.append(np.concatenate([pos, quat]))
        observation_eef_state = np.concatenate(eef_parts)

        frame_data: dict = {
            "observation.state": whole_q,
            "observation.eef_state": observation_eef_state,
            "action.wbc": whole_action_wbc,
        }

        self._add_cpp_state_features(frame_data, proprio)

        sonic_latency_ms = self._add_sonic_pose_features(frame_data)

        self._add_images_to_frame_data(frame_data)

        self._log_latency_periodic(sonic_latency_ms)

        self.data_exporter.add_frame(frame_data)

        if self._fp_enabled and self._fp_writer is not None:
            if not self._fp_writer.is_active():
                self._fp_writer.start_episode(self.current_episode_index)
            self._write_foundation_pose_frame()

        if self._gt_writer is not None:
            if not self._gt_writer.is_active():
                self._gt_writer.start_episode(self.current_episode_index)
            self._write_object_gt_frame()

        return self._finalize_frame(t_start)

    def _add_cpp_state_features(self, frame_data: dict, proprio: dict) -> None:
        if "base_quat" in proprio:
            base_quat = np.asarray(proprio["base_quat"], dtype=np.float64)
            frame_data["observation.root_orientation"] = base_quat
            frame_data["observation.projected_gravity"] = compute_projected_gravity(
                base_quat
            ).astype(np.float64)

            if "init_ref_data_root_rot_array" in proprio:
                frame_data["observation.cpp_rotation_offset"] = np.asarray(
                    proprio["init_ref_data_root_rot_array"], dtype=np.float64
                )
            else:
                frame_data["observation.cpp_rotation_offset"] = np.array(
                    [1.0, 0.0, 0.0, 0.0], dtype=np.float64
                )
        else:
            frame_data["observation.root_orientation"] = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            )
            frame_data["observation.projected_gravity"] = np.array(
                [0.0, 0.0, -1.0], dtype=np.float64
            )
            frame_data["observation.cpp_rotation_offset"] = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            )

        if "init_base_quat" in proprio:
            frame_data["observation.init_base_quat"] = np.asarray(
                proprio["init_base_quat"], dtype=np.float64
            )
        else:
            frame_data["observation.init_base_quat"] = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float64
            )

        # Reference base translation from the planner-generated motion frame the WBC
        # is tracking. Its z is the reference PELVIS HEIGHT, i.e. the crouch as the
        # planner actually realized it (teleop.planner_height is only the request,
        # which the planner ramps toward). Unlike object_gt's pelvis_in_world this
        # exists on real hardware too, where nothing observes the true base height.
        if "base_trans_target" in proprio:
            frame_data["observation.base_trans_target"] = np.asarray(
                proprio["base_trans_target"], dtype=np.float64
            )
        else:
            frame_data["observation.base_trans_target"] = np.zeros(3, dtype=np.float64)

        if "delta_heading" in proprio:
            dh = proprio["delta_heading"]
            if isinstance(dh, np.ndarray):
                dh = dh.item() if dh.size == 1 else dh[0]
            frame_data["teleop.delta_heading"] = np.array([float(dh)], dtype=np.float64)
        else:
            frame_data["teleop.delta_heading"] = np.zeros(1, dtype=np.float64)

        if "token_state" in proprio:
            frame_data["action.motion_token"] = np.asarray(proprio["token_state"], dtype=np.float64)
        else:
            frame_data["action.motion_token"] = np.zeros(64, dtype=np.float64)

    def _add_sonic_pose_features(self, frame_data: dict) -> float | None:
        """Add teleop features based on current stream mode."""
        sonic_latency_ms = None

        frame_data["teleop.stream_mode"] = np.array([self.current_stream_mode], dtype=np.int32)

        smpl_msg = self.latest_sonic_msg
        use_smpl = False
        if self.current_stream_mode in (1, 4) and smpl_msg is not None:
            receive_ts = smpl_msg.get("receive_timestamp")
            if receive_ts is not None:
                age_sec = time.time() - receive_ts
                sonic_latency_ms = age_sec * 1000
                self.sonic_timing_monitor.log_time_delta(age_sec)
                if sonic_latency_ms <= 100.0:
                    use_smpl = True
                elif (self.sonic_timing_monitor.failure_count + 1) % 10 == 0:
                    self._print_and_say(
                        f"Sonic pose stale ({sonic_latency_ms:.1f}ms old), using zeros",
                        say=False,
                    )
            else:
                use_smpl = True

        planner_msg = self.latest_planner_msg
        use_planner = False
        if self.current_stream_mode == 5 and planner_msg is not None:
            receive_ts = planner_msg.get("receive_timestamp")
            if receive_ts is not None:
                age_sec = time.time() - receive_ts
                planner_latency_ms = age_sec * 1000
                if sonic_latency_ms is None:
                    sonic_latency_ms = planner_latency_ms
                if planner_latency_ms <= 200.0:
                    use_planner = True
            else:
                use_planner = True

        # SMPL features
        if use_smpl and smpl_msg.get("smpl_joints") is not None:
            joints = np.asarray(smpl_msg["smpl_joints"], dtype=np.float32)
            if joints.ndim == 2:
                joints = joints.flatten()
            frame_data["teleop.smpl_joints"] = np.ascontiguousarray(joints, dtype=np.float32)
        else:
            frame_data["teleop.smpl_joints"] = np.zeros(72, dtype=np.float32)

        if use_smpl and smpl_msg.get("smpl_pose") is not None:
            pose = np.asarray(smpl_msg["smpl_pose"], dtype=np.float32)
            if pose.ndim > 1:
                pose = pose.flatten()
            frame_data["teleop.smpl_pose"] = np.ascontiguousarray(pose, dtype=np.float32)
        else:
            frame_data["teleop.smpl_pose"] = np.zeros(63, dtype=np.float32)

        if use_smpl and smpl_msg.get("body_quat_w") is not None:
            body_quat_w = smpl_msg["body_quat_w"].astype(np.float32)
            frame_data["teleop.body_quat_w"] = body_quat_w
            frame_data["teleop.target_body_orientation"] = self._compute_target_body_orientation(
                body_quat_w, frame_data
            )
        else:
            frame_data["teleop.body_quat_w"] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            frame_data["teleop.target_body_orientation"] = quat_to_rot6d(
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            )

        frame_data["teleop.left_wrist_joints"] = (
            smpl_msg["left_wrist_joints"].astype(np.float32)
            if use_smpl and smpl_msg.get("left_wrist_joints") is not None
            else np.zeros(3, dtype=np.float32)
        )
        frame_data["teleop.right_wrist_joints"] = (
            smpl_msg["right_wrist_joints"].astype(np.float32)
            if use_smpl and smpl_msg.get("right_wrist_joints") is not None
            else np.zeros(3, dtype=np.float32)
        )

        frame_data["teleop.smpl_frame_index"] = (
            smpl_msg["frame_index"].astype(np.int64)
            if use_smpl and smpl_msg is not None and smpl_msg.get("frame_index") is not None
            else np.array([0], dtype=np.int64)
        )

        hand_msg = (
            smpl_msg if self.current_stream_mode in (1, 4) and smpl_msg is not None
            else planner_msg if planner_msg is not None
            else smpl_msg
        )
        num_hand_dof = len(self.robot_model.get_hand_actuated_joint_indices("left"))
        frame_data["teleop.left_hand_joints"] = (
            hand_msg["left_hand_joints"].astype(np.float32)[:num_hand_dof]
            if hand_msg is not None
            and hand_msg.get("left_hand_joints") is not None
            else np.zeros(num_hand_dof, dtype=np.float32)
        )
        frame_data["teleop.right_hand_joints"] = (
            hand_msg["right_hand_joints"].astype(np.float32)[:num_hand_dof]
            if hand_msg is not None
            and hand_msg.get("right_hand_joints") is not None
            else np.zeros(num_hand_dof, dtype=np.float32)
        )

        # Planner command fields
        frame_data["teleop.planner_mode"] = np.array(
            [planner_msg["planner_mode"]] if use_planner else [0],
            dtype=np.int32,
        )
        frame_data["teleop.planner_movement"] = (
            planner_msg["planner_movement"].copy()
            if use_planner and planner_msg.get("planner_movement") is not None
            else np.zeros(3, dtype=np.float32)
        )
        frame_data["teleop.planner_facing"] = (
            planner_msg["planner_facing"].copy()
            if use_planner and planner_msg.get("planner_facing") is not None
            else np.array([1.0, 0.0, 0.0], dtype=np.float32)
        )
        frame_data["teleop.planner_speed"] = np.array(
            [planner_msg["planner_speed"]] if use_planner else [-1.0],
            dtype=np.float32,
        )
        frame_data["teleop.planner_height"] = np.array(
            [planner_msg["planner_height"]] if use_planner else [-1.0],
            dtype=np.float32,
        )

        # VR 3-point pose
        frame_data["teleop.vr_3pt_position"] = (
            planner_msg["vr_3pt_position"].astype(np.float32)
            if use_planner and planner_msg.get("vr_3pt_position") is not None
            else np.zeros(9, dtype=np.float32)
        )
        if use_planner and planner_msg.get("vr_3pt_orientation") is not None:
            frame_data["teleop.vr_3pt_orientation"] = quat_to_rot6d(
                planner_msg["vr_3pt_orientation"].astype(np.float32)
            )
        else:
            frame_data["teleop.vr_3pt_orientation"] = np.zeros(18, dtype=np.float32)

        return sonic_latency_ms

    def _compute_target_body_orientation(
        self, body_quat_w: np.ndarray, frame_data: dict
    ) -> np.ndarray:
        """Compute yaw-normalised target body orientation as rot6d (6-dim)."""
        delta_heading = float(frame_data.get("teleop.delta_heading", [0.0])[0])

        body_rot = R.from_quat(body_quat_w, scalar_first=True)
        target_rot = R.from_euler("z", delta_heading, degrees=False) * body_rot

        euler = target_rot.as_euler("ZYX", degrees=False)
        current_yaw = euler[0]

        if self._initial_yaw is None:
            self._initial_yaw = current_yaw

        normalised_euler = np.array([current_yaw - self._initial_yaw, euler[1], euler[2]])
        target_quat = (
            R.from_euler("ZYX", normalised_euler, degrees=False)
            .as_quat(scalar_first=True)
            .astype(np.float32)
        )
        return quat_to_rot6d(target_quat)

    def save_and_cleanup(self):
        try:
            if self.data_exporter is not None:
                self._print_and_say("saving episode done", blocking=False)
                buffer_size = self.data_exporter.episode_buffer.get("size", 0)
                if buffer_size > 0:
                    self.data_exporter.save_episode()
                self._print_and_say(
                    f"Recording complete: {self.data_exporter.meta.root}", say=False, blocking=True
                )
        except Exception as e:
            self._print_and_say(f"Error saving episode: {e}", blocking=True)

        try:
            self._state_subscriber.close()
        except Exception:
            pass
        for sock in [self._sonic_zmq_socket, self._box_gt_socket]:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        for ctx in [self._sonic_zmq_ctx, self._box_gt_ctx]:
            if ctx is not None:
                try:
                    ctx.term()
                except Exception:
                    pass

        self._print_and_say("Shutting down data exporter...", say=False)

    def run(self):
        try:
            while True:
                t_start = time.monotonic()
                with self.telemetry.timer("total_loop"):
                    with self.telemetry.timer("poll_state"):
                        self._poll_state_zmq()

                    with self.telemetry.timer("poll_sonic"):
                        self._poll_sonic_zmq_messages()

                    self._poll_box_gt()

                    with self.telemetry.timer("poll_image"):
                        img_msg = self._image_subscriber.read()
                        if img_msg is not None:
                            self.latest_image_msg = img_msg
                            if "ego_view_depth" in img_msg.get("images", {}):
                                self._fp_enabled = True

                    with self.telemetry.timer("add_frame"):
                        self._add_data_frame()

                    with self.telemetry.timer("check_recording_commands"):
                        self._check_recording_commands()

                    end_time = time.monotonic()

                elapsed = time.monotonic() - t_start
                sleep_time = self.loop_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                if (end_time - t_start) > self.loop_period:
                    self.telemetry.log_timing_info(
                        context="Data Exporter Loop Missed", threshold=0.001
                    )

        except KeyboardInterrupt:
            print("Data exporter terminated by user")
            buffer_size = (
                self.data_exporter.episode_buffer.get("size", 0)
                if self.data_exporter is not None
                else 0
            )
            if buffer_size > 0:
                if self._auto_record:
                    # No keyboard control to stop-and-save; Ctrl+C is the only way out.
                    self.data_exporter.save_episode()
                    if self._fp_writer is not None:
                        self._fp_writer.close_episode()
                    if self._gt_writer is not None:
                        self._gt_writer.close_episode()
                else:
                    # Manual mode Ctrl+C mid-recording = abort: leave no parquet/video behind.
                    self.data_exporter.discard_episode()
                    if self._fp_writer is not None:
                        self._fp_writer.discard_episode()
                    if self._gt_writer is not None:
                        self._gt_writer.discard_episode()

        finally:
            self.save_and_cleanup()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(config: SonicDataExporterConfig):
    g1_rm = get_g1_robot_model(hand_type=config.hand_type)

    dataset_features = get_features_sonic_vla(g1_rm)
    modality_config = get_modality_config_sonic_vla(g1_rm)

    if config.record_wrist_cameras:
        print("[Camera] Wrist cameras enabled — adding to dataset schema")
        dataset_features.update(get_wrist_camera_features())
        wrist_modality = get_wrist_camera_modality_config()
        for key, value in wrist_modality.items():
            if key in modality_config:
                modality_config[key].update(value)
            else:
                modality_config[key] = value

    text_to_speech = TextToSpeech() if config.text_to_speech else None

    if config.skip_robot_state:
        print("[Config] --skip-robot-state set, not waiting for robot_config")
        robot_config = {}
    else:
        robot_config = poll_robot_config_zmq(
            config.state_zmq_host, config.state_zmq_port, config.robot_config_timeout
        )

    # Built lazily by the collector on the first recording so launching the recorder
    # (without ever toggling data collection) doesn't create an empty dataset folder.
    def make_exporter() -> Gr00tDataExporter:
        return Gr00tDataExporter.create(
            save_root=f"{config.root_output_dir}/{config.dataset_name}",
            fps=config.data_collection_frequency,
            features=dataset_features,
            modality_config=modality_config,
            task=config.task_prompt,
            script_config={**robot_config, "record_wrist_cameras": config.record_wrist_cameras},
        )

    data_collector = GrootDataCollector(
        frequency=config.data_collection_frequency,
        exporter_factory=make_exporter,
        robot_model=g1_rm,
        camera_host=config.camera_host,
        camera_port=config.camera_port,
        text_to_speech=text_to_speech,
        sonic_data_zmq_host=config.sonic_zmq_host,
        sonic_data_zmq_port=config.sonic_zmq_port,
        state_zmq_host=config.state_zmq_host,
        state_zmq_port=config.state_zmq_port,
        skip_robot_state=config.skip_robot_state,
        auto_record=config.auto_record,
        record_object_gt=config.record_object_gt,
        box_gt_zmq_host=config.box_gt_zmq_host,
        box_gt_zmq_port=config.box_gt_zmq_port,
        hand_type=config.hand_type,
    )
    data_collector.run()


if __name__ == "__main__":
    config = tyro.cli(SonicDataExporterConfig)

    if config.dataset_name is None:
        config.dataset_name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    main(config)
