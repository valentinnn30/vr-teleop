"""Intel RealSense camera driver.

Requires the ``pyrealsense2`` SDK — install with::

    pip install pyrealsense2

See https://github.com/IntelRealSense/librealsense for hardware-specific instructions.
"""

import time
from typing import Any

import numpy as np

try:
    import gymnasium as gym
except ImportError:
    gym = None  # type: ignore[assignment]

import pyrealsense2 as rs

from data_collection.camera.sensor import Sensor
from data_collection.camera.sensor_server import (
    CameraMountPosition,
    ImageMessageSchema,
    SensorServer,
)


class RealSenseConfig:
    """Configuration for the RealSense camera."""

    depth_image_dim: tuple[int, int] = (640, 480)
    color_image_dim: tuple[int, int] = (640, 480)
    fps: int = 30
    mount_position: str = CameraMountPosition.EGO_VIEW.value


class RealSenseSensor(Sensor, SensorServer):
    """Sensor for Intel RealSense depth cameras."""

    def __init__(
        self,
        run_as_server: bool = False,
        port: int = 5555,
        config: RealSenseConfig = RealSenseConfig(),
        id: int = 0,
        mount_position: str = CameraMountPosition.EGO_VIEW.value,
    ):
        devices = rs.context().query_devices()
        if len(devices) == 0:
            raise RuntimeError("No RealSense devices found")

        for device in devices:
            print(f"Device: {device.get_info(rs.camera_info.name)}")
            print(f"    Serial number: {device.get_info(rs.camera_info.serial_number)}")
            print(f"    Firmware version: {device.get_info(rs.camera_info.firmware_version)}")

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        devices = sorted(devices, key=lambda x: x.get_info(rs.camera_info.serial_number))
        self.config.enable_device(devices[id].get_info(rs.camera_info.serial_number))

        try:
            self.config.enable_stream(
                rs.stream.color,
                config.color_image_dim[0],
                config.color_image_dim[1],
                rs.format.rgb8,
                config.fps,
            )
            self.config.enable_stream(
                rs.stream.depth,
                config.depth_image_dim[0],
                config.depth_image_dim[1],
                rs.format.z16,
                config.fps,
            )
            profile = self.pipeline.start(self.config)
        except Exception as e:
            raise RuntimeError(f"Failed to start RealSense pipeline: {e}")

        self._realsense_config = config
        self._run_as_server = run_as_server
        self.mount_position = mount_position
        self._depth_scale_mm = profile.get_device().first_depth_sensor().get_depth_scale() * 1000.0
        self._calibration = self._read_calibration(profile)
        if self._run_as_server:
            self.start_server(port)
        print(
            f"Done initializing RealSense sensor: "
            f"{devices[id].get_info(rs.camera_info.serial_number)}"
        )

    def _read_calibration(self, profile) -> dict[str, Any]:
        """Color intrinsics + depth->color extrinsics (depth/color are separate sensors)."""
        color_intrinsics = (
            profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        )
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        extrinsics = depth_stream.get_extrinsics_to(profile.get_stream(rs.stream.color))

        cam_K = [
            [color_intrinsics.fx, 0.0, color_intrinsics.ppx],
            [0.0, color_intrinsics.fy, color_intrinsics.ppy],
            [0.0, 0.0, 1.0],
        ]
        # Plain lists, not ndarrays: msgpack.packb() (sensor_server.py) has no numpy hook.
        return {
            "cam_K": cam_K,
            "cam_extrinsics_R": list(extrinsics.rotation),
            "cam_extrinsics_t": list(extrinsics.translation),
        }

    def get_calibration(self) -> dict[str, Any]:
        """Static camera calibration: color intrinsics + depth->color extrinsics."""
        return self._calibration

    def read(self) -> dict[str, Any] | None:
        try:
            frames = self.pipeline.wait_for_frames()
        except Exception as e:
            print(f"ERROR! Failed to wait for frames: {e}")
            return None

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            print("WARNING! No color or depth frame")
            return None

        try:
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            if self._depth_scale_mm != 1.0:
                depth_image = (depth_image.astype(np.float32) * self._depth_scale_mm).astype(
                    np.uint16
                )
        except Exception as e:
            print(f"ERROR! Failed to convert frames to numpy arrays: {e}")
            return None

        if color_image.size == 0 or depth_image.size == 0:
            print("WARNING! Empty color or depth image")
            return None

        current_time = time.time()
        timestamps = {
            self.mount_position: current_time,
            f"{self.mount_position}_depth": current_time,
        }
        images = {
            self.mount_position: color_image,
            f"{self.mount_position}_depth": depth_image,
        }
        return {"timestamps": timestamps, "images": images}

    def serialize(self, data: dict[str, Any]) -> dict[str, Any]:
        serialized_msg = ImageMessageSchema(timestamps=data["timestamps"], images=data["images"])
        return serialized_msg.serialize()

    def observation_space(self):
        if gym is None:
            return None
        return gym.spaces.Dict(
            {
                "color_image": gym.spaces.Box(
                    low=0,
                    high=255,
                    shape=(
                        self._realsense_config.color_image_dim[1],
                        self._realsense_config.color_image_dim[0],
                        3,
                    ),
                    dtype=np.uint8,
                ),
                "depth_image": gym.spaces.Box(
                    low=0,
                    high=255,
                    shape=(
                        self._realsense_config.depth_image_dim[1],
                        self._realsense_config.depth_image_dim[0],
                        1,
                    ),
                    dtype=np.uint16,
                ),
            }
        )

    def close(self):
        if self._run_as_server:
            self.stop_server()
        self.pipeline.stop()

    def run_server(self):
        if not self._run_as_server:
            raise ValueError("run_as_server must be True to call run_server()")
        while True:
            read_result = self.read()
            if read_result is None:
                continue
            self.send_message({self.mount_position: self.serialize(read_result)})
