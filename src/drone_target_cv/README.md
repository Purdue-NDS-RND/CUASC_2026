# drone_target_cv

ROS2 Python package for the camera compression and target-tracking nodes used by
`drone_mission_demo`.

Current nodes:
- `compressed_grabber`
- `target_cv`
- `usb_grabber`

These were split from `vision_pipeline` so mission-demo launch files can depend
on a narrower CV package while the older vision stack remains available during
the transition.

`usb_grabber` is the simple OpenCV USB-camera path. It publishes:
- `/camera/image/compressed`
- `/camera/camera_info`

For stable camera selection on Linux, prefer `device_path` with a `/dev/v4l/by-id/...`
symlink. `usb_grabber` now uses `device_path` only.
It does not load or apply camera intrinsics, and it requests MJPG from the USB
camera to reduce bandwidth when the device supports it. The default transport is
JPEG-compressed output for higher publish rate. Raw `/camera/image` publishing is
available but disabled by default.
The default mode is the fastest lowest-resolution mode reported by your camera:
`1280x720 @ 30 fps`.
When available, `usb_grabber` uses a native Python GStreamer appsink path so it
behaves more like a working `gst-launch-1.0 v4l2src ... autovideosink` pipeline.

Launch it with:

```bash
ros2 launch drone_target_cv usb_grabber.launch.py
```

If a consumer needs raw images:

```bash
ros2 launch drone_target_cv usb_grabber.launch.py publish_raw:=true
```

For `target_cv`, use:

```yaml
target_cv:
  ros__parameters:
    image_topic: "camera/image/compressed"
    compressed_input: true
```

Default device path:

```bash
/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_12MP_SN0001-video-index0
```

Override example:

```bash
ros2 launch drone_target_cv usb_grabber.launch.py \
  device_path:=/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_12MP_SN0001-video-index0
```
