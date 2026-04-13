"""Executor-owned context shared with missions."""

from __future__ import annotations

import math
from typing import Any, Callable, Optional

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix


class MissionContext:
    """Facade over shared telemetry, services, and managed setpoints."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._managed_local_setpoint: Optional[PoseStamped] = None

    @property
    def mavros_state(self) -> Optional[State]:
        return self._node._mavros_state

    @property
    def local_pose(self) -> Optional[PoseStamped]:
        return self._node._local_pose

    @property
    def global_gps(self) -> Optional[NavSatFix]:
        return self._node._global_gps

    @property
    def logger(self) -> RcutilsLogger:
        return self._node.get_logger()

    def now(self) -> Time:
        return self._node.get_clock().now()

    def seconds_since(self, start_time: Time) -> float:
        return (self.now() - start_time).nanoseconds / 1e9

    def takeoff_service_ready(self) -> bool:
        return self._node._takeoff_client.service_is_ready()

    def mode_service_ready(self) -> bool:
        return self._node._mode_client.service_is_ready()

    def request_takeoff(
        self,
        altitude_m: float,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = CommandTOL.Request()
        request.altitude = float(altitude_m)
        future = self._node._takeoff_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def request_mode_change(
        self,
        mode_name: str,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode_name
        future = self._node._mode_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def set_local_position_setpoint(
        self,
        east_m: float,
        north_m: float,
        up_m: float,
        yaw_deg: float = 90.0,
    ) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(east_m)
        msg.pose.position.y = float(north_m)
        msg.pose.position.z = float(up_m)

        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        self._managed_local_setpoint = msg

    def clear_local_position_setpoint(self) -> None:
        self._managed_local_setpoint = None

    def publish_managed_setpoints(self) -> None:
        if self._managed_local_setpoint is None:
            return
        self._managed_local_setpoint.header.stamp = self.now().to_msg()
        self._node._local_setpoint_pub.publish(self._managed_local_setpoint)

    def set_global_position_setpoint(self, *args: Any, **kwargs: Any) -> None:
        raise NotImplementedError("Global setpoints are reserved for a future mission.")

    def set_local_velocity_setpoint(self, *args: Any, **kwargs: Any) -> None:
        raise NotImplementedError("Local velocity setpoints are reserved for a future mission.")

    def point_gimbal(self, *args: Any, **kwargs: Any) -> None:
        raise NotImplementedError("Gimbal control is reserved for a future mission.")

    def actuate_servo(self, *args: Any, **kwargs: Any) -> None:
        raise NotImplementedError("Servo control is reserved for a future mission.")
