import math

from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.registry import register_mission


@register_mission("lawnmower_survey")
class LawnmowerSurveyMission(BaseMission):
    def on_enter(self, context):
        """Called once when the mission starts."""
        self.altitude = self.spec.config.get("altitude_m", 30.0)
        self.spacing = self.spec.config.get("spacing_m", 8.0)
        self.corners = self.spec.config.get("corners", [])  # Expects 4 [lat, lon] pairs

        if len(self.corners) != 4:
            raise ValueError("You must provide exactly 4 GPS corners!")

        # Generate the zigzag waypoints
        self.waypoints = self._generate_lawnmower(self.corners, self.spacing)
        self.current_wp_idx = 0

        context.get_logger().info(f"Generated {len(self.waypoints)} survey waypoints.")

    def update(self, context):
        """Called at 20Hz by the executor to control the drone."""
        if self.current_wp_idx >= len(self.waypoints):
            context.get_logger().info("Survey complete!")
            return MissionStatus.SUCCESS

        target_lat, target_lon = self.waypoints[self.current_wp_idx]

        # Tell the framework to manage the global setpoint
        # (Assuming the context has a method like set_global_setpoint based on the README)
        context.set_global_setpoint(target_lat, target_lon, self.altitude)

        # Check if we have arrived (within ~2 meters)
        current_gps = context.get_global_gps()
        if current_gps is not None:
            dist = self._distance_m(
                current_gps.latitude, current_gps.longitude, target_lat, target_lon
            )
            if dist < 2.0:
                context.get_logger().info(
                    f"Reached waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}"
                )
                self.current_wp_idx += 1

        return MissionStatus.RUNNING

    def on_exit(self, context):
        """Clean up when done."""
        context.clear_all_setpoints()

    # --- Math Helpers ---
    def _generate_lawnmower(self, corners, spacing_m):
        """Simple bounding-box lawnmower generator."""
        lats = [c[0] for c in corners]
        lons = [c[1] for c in corners]

        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        # Convert spacing in meters to rough GPS degree steps
        # (1 degree lat ~= 111,320 meters)
        lat_step = spacing_m / 111320.0

        waypoints = []
        current_lat = min_lat
        going_east = True

        while current_lat <= max_lat:
            if going_east:
                waypoints.append([current_lat, min_lon])
                waypoints.append([current_lat, max_lon])
            else:
                waypoints.append([current_lat, max_lon])
                waypoints.append([current_lat, min_lon])

            going_east = not going_east
            current_lat += lat_step

        return waypoints

    def _distance_m(self, lat1, lon1, lat2, lon2):
        """Haversine distance in meters."""
        R = 6378137.0
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        a = (
            math.sin(d_lat / 2) ** 2
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(d_lon / 2) ** 2
        )
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
