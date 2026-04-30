from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

from drone_utils.session_log_utils import (
    build_session_dir,
    make_command_velocity_row,
    sanitize_session_name,
    should_save_sample,
    stamp_to_seconds,
)


def _stamp(sec, nanosec):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_build_session_dir_sanitizes_name_and_adds_timestamp():
    created_at = datetime(2026, 4, 30, 12, 34, 56)

    session_dir = build_session_dir(
        Path("/tmp/cuasc_logs"),
        " mission/live drop ",
        created_at,
    )

    assert session_dir == Path("/tmp/cuasc_logs/mission_live_drop_20260430_123456")


def test_sanitize_session_name_falls_back_to_mission():
    assert sanitize_session_name(" /// ") == "mission"


def test_stamp_to_seconds_uses_nanoseconds():
    assert stamp_to_seconds(_stamp(12, 345_000_000)) == 12.345


def test_make_command_velocity_row_formats_position_target_fields():
    msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=_stamp(10, 250_000_000),
            frame_id="map",
        ),
        coordinate_frame=1,
        type_mask=3527,
        velocity=SimpleNamespace(x=1.25, y=-0.5, z=0.125),
        yaw=1.57079632679,
    )

    row = make_command_velocity_row(
        msg,
        wall_time_iso="2026-04-30T12:34:56.000-04:00",
        ros_time_sec=42.0,
    )

    assert row == {
        "wall_time_iso": "2026-04-30T12:34:56.000-04:00",
        "ros_time_sec": "42.000000000",
        "msg_stamp_sec": "10.250000000",
        "frame_id": "map",
        "coordinate_frame": "1",
        "type_mask": "3527",
        "dx_mps": "1.250000000",
        "dy_mps": "-0.500000000",
        "up_mps": "0.125000000",
        "yaw_rad": "1.570796327",
    }


def test_should_save_sample_obeys_interval():
    assert should_save_sample(None, 10.0, 1.0)
    assert not should_save_sample(10.0, 10.5, 1.0)
    assert should_save_sample(10.0, 11.0, 1.0)
