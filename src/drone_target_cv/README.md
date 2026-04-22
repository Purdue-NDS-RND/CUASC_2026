# drone_target_cv

ROS2 Python package for the camera compression and target-tracking nodes used by
`drone_mission_demo`.

Current nodes:
- `compressed_grabber`
- `target_cv`

These were split from `vision_pipeline` so mission-demo launch files can depend
on a narrower CV package while the older vision stack remains available during
the transition.
