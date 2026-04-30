"""Pure helpers for mission session logging."""

from __future__ import annotations

import re
from datetime import datetime
from pathlib import Path
from typing import Any


CSV_FIELDNAMES = [
    "wall_time_iso",
    "ros_time_sec",
    "msg_stamp_sec",
    "frame_id",
    "coordinate_frame",
    "type_mask",
    "dx_mps",
    "dy_mps",
    "up_mps",
    "yaw_rad",
]


def sanitize_session_name(session_name: str) -> str:
    """Return a filesystem-safe session name prefix."""
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", session_name.strip())
    cleaned = cleaned.strip("._-")
    return cleaned or "mission"


def build_session_dir(
    base_dir: str | Path,
    session_name: str,
    created_at: datetime,
) -> Path:
    """Build the default timestamped session directory path."""
    timestamp = created_at.strftime("%Y%m%d_%H%M%S")
    return Path(base_dir).expanduser() / (
        f"{sanitize_session_name(session_name)}_{timestamp}"
    )


def stamp_to_seconds(stamp: Any) -> float:
    """Convert a ROS builtin_interfaces/Time-like stamp to seconds."""
    return float(stamp.sec) + (float(stamp.nanosec) / 1e9)


def header_stamp_key(msg: Any) -> tuple[int, int]:
    """Return a stable key for a ROS message header stamp."""
    stamp = msg.header.stamp
    return int(stamp.sec), int(stamp.nanosec)


def make_command_velocity_row(
    msg: Any,
    *,
    wall_time_iso: str,
    ros_time_sec: float,
) -> dict[str, str]:
    """Format a MAVROS PositionTarget message as a CSV row."""
    return {
        "wall_time_iso": wall_time_iso,
        "ros_time_sec": f"{ros_time_sec:.9f}",
        "msg_stamp_sec": f"{stamp_to_seconds(msg.header.stamp):.9f}",
        "frame_id": str(msg.header.frame_id),
        "coordinate_frame": str(int(msg.coordinate_frame)),
        "type_mask": str(int(msg.type_mask)),
        "dx_mps": f"{float(msg.velocity.x):.9f}",
        "dy_mps": f"{float(msg.velocity.y):.9f}",
        "up_mps": f"{float(msg.velocity.z):.9f}",
        "yaw_rad": f"{float(msg.yaw):.9f}",
    }


def should_save_sample(
    last_save_time_s: float | None,
    now_s: float,
    interval_s: float,
) -> bool:
    """Return true when a periodic sample should be saved."""
    if last_save_time_s is None:
        return True
    return (float(now_s) - float(last_save_time_s)) >= max(float(interval_s), 0.0)
