#!/usr/bin/env python3
"""Set live package-drop and package-delivery target GPS coordinates."""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parent
SOURCE_CONFIGS = (
    REPO_ROOT / "src/drone_mission_demo/config/sequences/package_drop_live.yaml",
    REPO_ROOT / "src/drone_mission_demo/config/sequences/package_delivery_live.yaml",
)
INSTALL_CONFIGS = (
    REPO_ROOT / "install/drone_mission_demo/share/drone_mission_demo/config/sequences/package_drop_live.yaml",
    REPO_ROOT / "install/drone_mission_demo/share/drone_mission_demo/config/sequences/package_delivery_live.yaml",
)


class CoordinateError(RuntimeError):
    """Raised when coordinates cannot be read or written safely."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fill target_latitude and target_longitude in the live drone_mission_demo "
            "sequence configs."
        )
    )
    parser.add_argument(
        "--mode",
        choices=("manual", "auto"),
        help="manual prompts for lat/lon; auto reads /mavros/global_position/global",
    )
    parser.add_argument("--latitude", type=float, help="Manual target latitude.")
    parser.add_argument("--longitude", type=float, help="Manual target longitude.")
    parser.add_argument(
        "--topic",
        default="/mavros/global_position/global",
        help="NavSatFix topic used by --mode auto.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for a valid GPS fix in --mode auto.",
    )
    parser.add_argument(
        "--also-install",
        action="store_true",
        help=(
            "Also update installed share configs if they exist. This is useful when "
            "you need to launch before rebuilding, but source configs are always updated."
        ),
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the confirmation prompt before writing files.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be updated without changing files.",
    )
    return parser.parse_args()


def prompt_choice(prompt: str, choices: Iterable[str]) -> str:
    allowed = tuple(choices)
    while True:
        value = input(prompt).strip().lower()
        if value in allowed:
            return value
        print(f"Enter one of: {', '.join(allowed)}")


def prompt_float(prompt: str) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            return float(raw)
        except ValueError:
            print("Enter a decimal number.")


def validate_coordinates(latitude: float, longitude: float) -> tuple[float, float]:
    if not -90.0 <= latitude <= 90.0:
        raise CoordinateError(f"Latitude out of range: {latitude}")
    if not -180.0 <= longitude <= 180.0:
        raise CoordinateError(f"Longitude out of range: {longitude}")
    if latitude == 0.0 and longitude == 0.0:
        raise CoordinateError("Refusing to write 0.0, 0.0 as a live mission target.")
    return latitude, longitude


def get_manual_coordinates(args: argparse.Namespace) -> tuple[float, float]:
    latitude = args.latitude if args.latitude is not None else prompt_float("Target latitude: ")
    longitude = args.longitude if args.longitude is not None else prompt_float("Target longitude: ")
    return validate_coordinates(latitude, longitude)


def get_auto_coordinates(topic: str, timeout_s: float) -> tuple[float, float]:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import NavSatFix
    except ImportError as exc:
        raise CoordinateError(
            "Auto mode needs a sourced ROS 2 environment with rclpy and sensor_msgs available."
        ) from exc

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    class GpsReader(Node):
        def __init__(self) -> None:
            super().__init__("live_mission_coordinate_reader")
            self.fix = None
            self.create_subscription(NavSatFix, topic, self._on_fix, qos)

        def _on_fix(self, msg: NavSatFix) -> None:
            if msg.status.status < 0:
                return
            if msg.latitude == 0.0 and msg.longitude == 0.0:
                return
            self.fix = msg

    rclpy.init(args=None)
    node = GpsReader()
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline and node.fix is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.fix is None:
            raise CoordinateError(f"No valid GPS fix received on {topic} within {timeout_s:.1f}s.")
        return validate_coordinates(float(node.fix.latitude), float(node.fix.longitude))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def update_coordinate_line(text: str, key: str, value: float) -> tuple[str, bool]:
    pattern = re.compile(rf"^(?P<indent>\s*{re.escape(key)}:\s*)(?P<old>[-+]?\d+(?:\.\d+)?)(?P<rest>\s*(?:#.*)?)$", re.MULTILINE)
    replacement = rf"\g<indent>{value:.7f}\g<rest>"
    updated, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise CoordinateError(f"Could not find exactly one {key} entry.")
    return updated, True


def update_config(path: Path, latitude: float, longitude: float, dry_run: bool) -> None:
    original = path.read_text(encoding="utf-8")
    updated, _ = update_coordinate_line(original, "target_latitude", latitude)
    updated, _ = update_coordinate_line(updated, "target_longitude", longitude)
    if updated == original:
        print(f"unchanged: {path}")
        return
    if dry_run:
        print(f"would update: {path}")
        return
    path.write_text(updated, encoding="utf-8")
    print(f"updated: {path}")


def existing_paths(paths: Iterable[Path]) -> list[Path]:
    return [path for path in paths if path.exists()]


def main() -> int:
    args = parse_args()
    mode = args.mode or prompt_choice("Coordinate source [manual/auto]: ", ("manual", "auto"))

    try:
        if mode == "manual":
            latitude, longitude = get_manual_coordinates(args)
        else:
            latitude, longitude = get_auto_coordinates(args.topic, args.timeout)

        print(f"Using target_latitude={latitude:.7f}, target_longitude={longitude:.7f}")
        if not args.yes:
            confirm = prompt_choice("Write these coordinates to the live configs? [yes/no]: ", ("yes", "no"))
            if confirm != "yes":
                print("Aborted without writing files.")
                return 1

        paths = list(SOURCE_CONFIGS)
        missing = [path for path in paths if not path.exists()]
        if missing:
            raise CoordinateError("Missing source config(s): " + ", ".join(str(path) for path in missing))

        if args.also_install:
            paths.extend(existing_paths(INSTALL_CONFIGS))

        for path in paths:
            update_config(path, latitude, longitude, args.dry_run)

        if not args.also_install:
            print("Rebuild or run with --also-install before launching from install/share configs.")
        return 0
    except (CoordinateError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
