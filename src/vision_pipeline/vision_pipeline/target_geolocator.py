"""
Target Geolocator
Converts local camera-frame coordinates to global GPS coordinates.
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


class TargetGeolocator(Node):
    def __init__(self):
        super().__init__("target_geolocator")

        # State variables for the VTOL's live telemetry
        self.current_lat = None
        self.current_lon = None
        self.current_heading_rad = None

        # Earth Radius in meters
        self.R_EARTH = 6378137.0

        # 1. Listen to the Raycaster
        self.local_sub = self.create_subscription(
            PointStamped, "/vision/target_world", self.target_callback, 10
        )

        # 2. Listen to MAVROS (Flight Controller Telemetry)
        self.gps_sub = self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_callback, 10
        )

        self.compass_sub = self.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self.compass_callback, 10
        )

        # 3. Publisher for the final GPS target
        self.global_pub = self.create_publisher(NavSatFix, "/vision/target_gps", 10)

        self.get_logger().info(
            "Geolocator initialized. Waiting for telemetry and targets..."
        )

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def compass_callback(self, msg: Float64):
        # MAVROS compass is usually in degrees (0=North, 90=East). Convert to radians.
        self.current_heading_rad = math.radians(msg.data)

    def target_callback(self, msg: PointStamped):
        # Ensure we have live telemetry before doing the math
        if self.current_lat is None or self.current_heading_rad is None:
            self.get_logger().warn(
                "Discarding target: No GPS/Compass telemetry from flight controller yet."
            )
            return

        # Local Coordinates (Assuming X is forward, Y is left)
        x_forward = msg.point.x
        y_left = msg.point.y

        # Step 1: Rotate to North and East
        # Standard rotation to align body frame with geographic frame
        d_north = (x_forward * math.cos(self.current_heading_rad)) - (
            y_left * math.sin(self.current_heading_rad)
        )
        d_east = (x_forward * math.sin(self.current_heading_rad)) + (
            y_left * math.cos(self.current_heading_rad)
        )

        # Step 2: Equirectangular Projection to Latitude/Longitude
        lat_offset = (d_north / self.R_EARTH) * (180.0 / math.pi)

        # Longitude scale shrinks as you move away from the equator
        lon_scale = math.cos(self.current_lat * math.pi / 180.0)
        lon_offset = (d_east / (self.R_EARTH * lon_scale)) * (180.0 / math.pi)

        target_lat = self.current_lat + lat_offset
        target_lon = self.current_lon + lon_offset

        # Publish the Global Target
        global_target = NavSatFix()
        global_target.header = msg.header  # Keep the original image timestamp
        global_target.latitude = target_lat
        global_target.longitude = target_lon
        global_target.altitude = 0.0  # Target is on the ground

        self.global_pub.publish(global_target)
        self.get_logger().info(
            f"Target Geolocation: Lat {target_lat:.7f}, Lon {target_lon:.7f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetGeolocator()
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
