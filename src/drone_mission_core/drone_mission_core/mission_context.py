"""Executor-owned context shared with missions."""

from __future__ import annotations

import math
from typing import Any, Callable, Optional

from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import (
    AttitudeTarget,
    ExtendedState,
    GlobalPositionTarget,
    PositionTarget,
    State,
)
from mavros_msgs.srv import CommandLong, CommandTOL, GimbalManagerPitchyaw, SetMode
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool


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
    def extended_state(self) -> Optional[ExtendedState]:
        return self._node._extended_state

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

    def landing_state_available(self) -> bool:
        extended_state = self.extended_state
        if extended_state is None:
            return False
        return extended_state.landed_state != ExtendedState.LANDED_STATE_UNDEFINED

    def vehicle_is_landed(self) -> bool:
        extended_state = self.extended_state
        if extended_state is None:
            return False
        return extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def takeoff_service_ready(self) -> bool:
        return self._node._takeoff_client.service_is_ready()

    def mode_service_ready(self) -> bool:
        return self._node._mode_client.service_is_ready()

    def command_service_ready(self) -> bool:
        return self._node._command_client.service_is_ready()

    def gimbal_service_ready(self) -> bool:
        return self._node._gimbal_client.service_is_ready()

    def target_cv_control_ready(self) -> bool:
        return self._node._target_cv_control_client.service_is_ready()

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

    def command_gripper(
        self,
        *,
        release: bool,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = CommandLong.Request()
        request.broadcast = False
        request.command = 211
        request.confirmation = 0
        request.param1 = 1.0
        request.param2 = 0.0 if release else 1.0
        future = self._node._command_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def command_sprayer(
        self,
        *,
        enable: bool,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = CommandLong.Request()
        request.broadcast = False
        request.command = 216
        request.confirmation = 0
        request.param1 = 1.0 if enable else 0.0
        request.param2 = 0.0
        future = self._node._command_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def set_target_cv_enabled(
        self,
        enabled: bool,
        done_callback: Callable[[Future[Any]], None] | None = None,
    ) -> Future[Any]:
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self._node._target_cv_control_client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def clear_target_tracking_state(self) -> None:
        self._node._target_detection = None
        self._node._image_size = None

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
        self._node._managed_attitude_setpoint = None
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
        self._node._managed_attitude_setpoint = None
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
        self._node._managed_attitude_setpoint = None
        self._node._managed_local_velocity_setpoint = msg

    def clear_local_velocity_setpoint(self) -> None:
        self._node._managed_local_velocity_setpoint = None

    def set_attitude_climb_rate_setpoint(
        self,
        climb_rate_mps: float,
        yaw_deg: float = 90.0,
        max_climb_rate_mps: float = 2.5,
    ) -> None:
        msg = AttitudeTarget()
        msg.header.frame_id = "map"
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )

        yaw_rad = math.radians(yaw_deg)
        msg.orientation.w = math.cos(yaw_rad / 2.0)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(yaw_rad / 2.0)

        capped_max_climb = max(float(max_climb_rate_mps), 0.01)
        normalized_climb = max(
            -1.0,
            min(float(climb_rate_mps) / capped_max_climb, 1.0),
        )
        msg.thrust = 0.5 + 0.5 * normalized_climb

        self._managed_local_setpoint = None
        self._node._managed_global_setpoint = None
        self._node._managed_local_velocity_setpoint = None
        self._node._managed_attitude_setpoint = msg

    def clear_attitude_setpoint(self) -> None:
        self._node._managed_attitude_setpoint = None

    def clear_all_setpoints(self) -> None:
        self.clear_local_position_setpoint()
        self.clear_global_position_setpoint()
        self.clear_local_velocity_setpoint()
        self.clear_attitude_setpoint()

    def publish_managed_setpoints(self) -> None:
        if self._managed_local_setpoint is None:
            attitude_setpoint = self._node._managed_attitude_setpoint
            if attitude_setpoint is not None:
                attitude_setpoint.header.stamp = self.now().to_msg()
                self._node._attitude_setpoint_pub.publish(attitude_setpoint)
                return

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
