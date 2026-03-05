from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.srv import GimbalManagerPitchyaw


class Gimble_PointService(Node):
    def __init__(self) -> None:
        super().__init__("gimbal_point_service")

        self.srv = self.create_service(GimbalManagerPitchyaw, "drone_utils/set_gimbal_point", self.set_gimbal_point_callback)

        self._gimbal_client = self.create_client(GimbalManagerPitchyaw, "/mavros/gimbal_control/manager/pitchyaw")

    def set_gimbal_point_callback(self, request: GimbalManagerPitchyaw.Request, response: GimbalManagerPitchyaw.Response) -> GimbalManagerPitchyaw.Response:
        self._set_gimbal(request.pitch, request.yaw)
        response.success = True
        response.result = 0
        return response


    def _on_timer(self) -> None:
        self._set_gimbal(-90.0)

    def _set_gimbal(self, pitch: float, yaw: float = 0.0) -> None:
        req = GimbalManagerPitchyaw.Request()
        req.pitch = pitch
        req.yaw = yaw
        req.pitch_rate = float('nan')
        req.yaw_rate = float('nan')
        req.flags = 0
        self._gimbal_client.call_async(req)


def main() -> None:
    rclpy.init()
    node = Gimble_PointService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()