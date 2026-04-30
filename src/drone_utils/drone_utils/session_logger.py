"""ROS 2 node for logging mission session data to disk."""

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import rclpy
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from drone_utils.session_log_utils import (
    CSV_FIELDNAMES,
    build_session_dir,
    header_stamp_key,
    make_command_velocity_row,
    should_save_sample,
)


@dataclass
class LatestImage:
    msg: Image | CompressedImage
    sequence: int


class MissionSessionLogger(Node):
    """Log mission commands and image snapshots into one session folder."""

    def __init__(self) -> None:
        super().__init__("session_logger")

        self.declare_parameter("output_base", "~/cuasc_logs")
        self.declare_parameter("session_name", "mission")
        self.declare_parameter("image_interval_s", 1.0)
        self.declare_parameter("command_topic", "/mavros/setpoint_raw/local")
        self.declare_parameter("camera_topic", "/camera/image/compressed")
        self.declare_parameter("annotated_topic", "/target_cv/annotated")
        self.declare_parameter("mask_topic", "/target_cv/mask")
        self.declare_parameter("debug_topic_warn_after_s", 5.0)

        self._image_interval_s = max(
            float(self.get_parameter("image_interval_s").value),
            0.01,
        )
        self._debug_topic_warn_after_s = max(
            float(self.get_parameter("debug_topic_warn_after_s").value),
            0.0,
        )
        self._command_topic = str(self.get_parameter("command_topic").value)
        self._camera_topic = str(self.get_parameter("camera_topic").value)
        self._annotated_topic = str(self.get_parameter("annotated_topic").value)
        self._mask_topic = str(self.get_parameter("mask_topic").value)

        self._session_dir = self._create_session_dir()
        self._image_dirs = {
            "camera": self._session_dir / "images" / "camera",
            "annotated": self._session_dir / "images" / "annotated",
            "mask": self._session_dir / "images" / "mask",
        }
        for image_dir in self._image_dirs.values():
            image_dir.mkdir(parents=True, exist_ok=True)

        self._csv_file = (self._session_dir / "command_velocity.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDNAMES)
        self._csv_writer.writeheader()
        self._csv_file.flush()

        self._latest_images: dict[str, LatestImage | None] = {
            "camera": None,
            "annotated": None,
            "mask": None,
        }
        self._image_sequences = {
            "camera": 0,
            "annotated": 0,
            "mask": 0,
        }
        self._last_saved_sequences = {
            "camera": 0,
            "annotated": 0,
            "mask": 0,
        }
        self._save_counts = {
            "camera": 0,
            "annotated": 0,
            "mask": 0,
        }
        self._last_image_save_time_s: float | None = None
        self._started_monotonic_s = time.monotonic()
        self._missing_debug_warned = {
            "annotated": False,
            "mask": False,
        }

        self._write_metadata()
        self._create_subscriptions()
        self._image_timer = self.create_timer(
            min(self._image_interval_s, 0.25),
            self._on_image_timer,
        )

        self.get_logger().info(
            f"Mission session logger writing to {self._session_dir}"
        )

    def destroy_node(self) -> bool:
        if not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
        return super().destroy_node()

    def _create_session_dir(self) -> Path:
        output_base = self.get_parameter("output_base").value
        session_name = str(self.get_parameter("session_name").value)
        session_dir = build_session_dir(
            Path(str(output_base)),
            session_name,
            datetime.now().astimezone(),
        )
        candidate = session_dir
        suffix = 1
        while candidate.exists():
            candidate = session_dir.with_name(f"{session_dir.name}_{suffix:02d}")
            suffix += 1
        candidate.mkdir(parents=True, exist_ok=False)
        return candidate

    def _write_metadata(self) -> None:
        metadata = {
            "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
            "session_dir": str(self._session_dir),
            "image_interval_s": self._image_interval_s,
            "topics": {
                "command_velocity": self._command_topic,
                "camera": self._camera_topic,
                "annotated": self._annotated_topic,
                "mask": self._mask_topic,
            },
            "files": {
                "command_velocity_csv": str(
                    self._session_dir / "command_velocity.csv"
                ),
                "camera_images": str(self._image_dirs["camera"]),
                "annotated_images": str(self._image_dirs["annotated"]),
                "mask_images": str(self._image_dirs["mask"]),
            },
        }
        metadata_path = self._session_dir / "metadata.json"
        metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _create_subscriptions(self) -> None:
        self.create_subscription(
            PositionTarget,
            self._command_topic,
            self._on_command_velocity,
            10,
        )
        self.create_subscription(
            CompressedImage,
            self._camera_topic,
            self._store_camera_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self._annotated_topic,
            self._store_annotated_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self._mask_topic,
            self._store_mask_image,
            qos_profile_sensor_data,
        )

    def _on_command_velocity(self, msg: PositionTarget) -> None:
        row = make_command_velocity_row(
            msg,
            wall_time_iso=datetime.now(timezone.utc).astimezone().isoformat(
                timespec="milliseconds"
            ),
            ros_time_sec=self.get_clock().now().nanoseconds / 1e9,
        )
        self._csv_writer.writerow(row)
        self._csv_file.flush()

    def _store_camera_image(self, msg: CompressedImage) -> None:
        self._store_image("camera", msg)

    def _store_annotated_image(self, msg: Image) -> None:
        self._store_image("annotated", msg)

    def _store_mask_image(self, msg: Image) -> None:
        self._store_image("mask", msg)

    def _store_image(self, stream_name: str, msg: Image | CompressedImage) -> None:
        self._image_sequences[stream_name] += 1
        self._latest_images[stream_name] = LatestImage(
            msg=msg,
            sequence=self._image_sequences[stream_name],
        )

    def _on_image_timer(self) -> None:
        now_s = time.monotonic()
        if not should_save_sample(
            self._last_image_save_time_s,
            now_s,
            self._image_interval_s,
        ):
            self._maybe_warn_missing_debug_topics(now_s)
            return

        saved_any = False
        saved_any |= self._save_latest_image(
            stream_name="camera",
            extension="jpg",
            converter=self._compressed_image_to_bgr,
        )
        saved_any |= self._save_latest_image(
            stream_name="annotated",
            extension="jpg",
            converter=self._image_to_cv_array,
        )
        saved_any |= self._save_latest_image(
            stream_name="mask",
            extension="png",
            converter=self._image_to_cv_array,
        )
        if saved_any:
            self._last_image_save_time_s = now_s

        self._maybe_warn_missing_debug_topics(now_s)

    def _save_latest_image(
        self,
        *,
        stream_name: str,
        extension: str,
        converter: Callable[[Image | CompressedImage], np.ndarray],
    ) -> bool:
        latest = self._latest_images[stream_name]
        if latest is None or latest.sequence == self._last_saved_sequences[stream_name]:
            return False

        try:
            frame = converter(latest.msg)
            filename = self._next_image_filename(stream_name, latest.msg, extension)
            output_path = self._image_dirs[stream_name] / filename
            if not cv2.imwrite(str(output_path), frame):
                raise RuntimeError("cv2.imwrite returned false")
        except Exception as exc:
            self.get_logger().warn(f"Failed to save {stream_name} image: {exc}")
            return False

        self._last_saved_sequences[stream_name] = latest.sequence
        return True

    def _next_image_filename(
        self,
        stream_name: str,
        msg: Image | CompressedImage,
        extension: str,
    ) -> str:
        self._save_counts[stream_name] += 1
        stamp_sec, stamp_nsec = header_stamp_key(msg)
        return (
            f"{stream_name}_{self._save_counts[stream_name]:06d}_"
            f"{stamp_sec}_{stamp_nsec:09d}.{extension}"
        )

    def _maybe_warn_missing_debug_topics(self, now_s: float) -> None:
        if now_s - self._started_monotonic_s < self._debug_topic_warn_after_s:
            return

        for stream_name, topic in (
            ("annotated", self._annotated_topic),
            ("mask", self._mask_topic),
        ):
            if self._missing_debug_warned[stream_name]:
                continue
            if self._latest_images[stream_name] is not None:
                continue
            self.get_logger().warn(
                f"No {stream_name} images received on {topic}; "
                "continuing with available log streams"
            )
            self._missing_debug_warned[stream_name] = True

    @staticmethod
    def _compressed_image_to_bgr(msg: Image | CompressedImage) -> np.ndarray:
        if not isinstance(msg, CompressedImage):
            raise TypeError("expected sensor_msgs/CompressedImage")
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("compressed image decode failed")
        return frame

    @staticmethod
    def _image_to_cv_array(msg: Image | CompressedImage) -> np.ndarray:
        if not isinstance(msg, Image):
            raise TypeError("expected sensor_msgs/Image")

        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)

        if encoding == "mono8":
            rows = data.reshape((height, step))[:, :width]
            return np.ascontiguousarray(rows)

        if encoding in ("bgr8", "rgb8"):
            rows = data.reshape((height, step))[:, : width * 3]
            frame = rows.reshape((height, width, 3))
            if encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return np.ascontiguousarray(frame)

        if encoding in ("bgra8", "rgba8"):
            rows = data.reshape((height, step))[:, : width * 4]
            frame = rows.reshape((height, width, 4))
            code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
            return cv2.cvtColor(frame, code)

        raise ValueError(f"unsupported image encoding: {msg.encoding}")


def main() -> None:
    rclpy.init()
    node = MissionSessionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
