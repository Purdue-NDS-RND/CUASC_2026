"""Executor-owned context shared with missions."""

from __future__ import annotations

import math
from typing import Any, Callable, Optional

from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget, State
from mavros_msgs.srv import CommandLong, CommandTOL, GimbalManagerPitchyaw, SetMode
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
    def target_detection(self) -> Optional[PointStamped]:
        return self._node._target_detection

    @property
    def image_size(self) -> Optional[tuple[int, int]]:
        return self._node._image_size

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

    def command_service_ready(self) -> bool:
        return self._node._command_client.service_is_ready()

    def gimbal_service_ready(self) -> bool:
        return self._node._gimbal_client.service_is_ready()

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

    def point_gimbal(
        self,
        pitch_deg: float,
        yaw_deg: float,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = GimbalManagerPitchyaw.Request()
        request.pitch = float(pitch_deg)
        request.yaw = float(yaw_deg)
        request.pitch_rate = float("nan")
        request.yaw_rate = float("nan")
        request.flags = 0
        future = self._node._gimbal_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def actuate_servo(
        self,
        channel: int,
        pwm: int,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = CommandLong.Request()
        request.command = 183
        request.param1 = float(channel)
        request.param2 = float(pwm)
        future = self._node._command_client.call_async(request)
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
        self._node._managed_global_setpoint = None
        self._node._managed_local_velocity_setpoint = None
        self._managed_local_setpoint = msg

    def clear_local_position_setpoint(self) -> None:
        self._managed_local_setpoint = None

    def set_global_position_setpoint(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        yaw_deg: float = 90.0,
        lock_yaw: bool = True,
    ) -> None:
        msg = GlobalPositionTarget()
        msg.header.frame_id = "map"
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = float(altitude_m)
        msg.yaw = math.radians(yaw_deg)
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        if not lock_yaw:
            msg.type_mask |= GlobalPositionTarget.IGNORE_YAW
        self._managed_local_setpoint = None
        self._node._managed_local_velocity_setpoint = None
        self._node._managed_global_setpoint = msg

    def clear_global_position_setpoint(self) -> None:
        self._node._managed_global_setpoint = None

    def set_local_velocity_setpoint(
        self,
        east_mps: float,
        north_mps: float,
        up_mps: float = 0.0,
        yaw_deg: float = 90.0,
    ) -> None:
        msg = PositionTarget()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = float(east_mps)
        msg.velocity.y = float(north_mps)
        msg.velocity.z = float(up_mps)
        msg.yaw = math.radians(yaw_deg)
        self._managed_local_setpoint = None
        self._node._managed_global_setpoint = None
        self._node._managed_local_velocity_setpoint = msg

    def clear_local_velocity_setpoint(self) -> None:
        self._node._managed_local_velocity_setpoint = None

    def clear_all_setpoints(self) -> None:
        self.clear_local_position_setpoint()
        self.clear_global_position_setpoint()
        self.clear_local_velocity_setpoint()

    def publish_managed_setpoints(self) -> None:
        if self._managed_local_setpoint is None:
            local_velocity_setpoint = self._node._managed_local_velocity_setpoint
            if local_velocity_setpoint is not None:
                local_velocity_setpoint.header.stamp = self.now().to_msg()
                self._node._local_velocity_setpoint_pub.publish(local_velocity_setpoint)
                return

            global_setpoint = self._node._managed_global_setpoint
            if global_setpoint is not None:
                global_setpoint.header.stamp = self.now().to_msg()
                self._node._global_setpoint_pub.publish(global_setpoint)
            return

        self._managed_local_setpoint.header.stamp = self.now().to_msg()
        self._node._local_setpoint_pub.publish(self._managed_local_setpoint)
