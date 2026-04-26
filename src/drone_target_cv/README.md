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

## USB Camera Prerequisites

On a Jetson or other Ubuntu/Linux ROS2 machine, install the basic USB camera
capture dependencies with:

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy v4l-utils
```

These provide:
- `python3-opencv` for `cv2.VideoCapture(...)`
- `python3-numpy` for image buffer handling
- `v4l-utils` for tools like `v4l2-ctl` when checking formats and devices

`usb_grabber` is the simple OpenCV USB-camera path. It publishes:
- `/camera/image/compressed`
- `/camera/camera_info`

For stable camera selection on Linux, use `camera_type` for known team cameras
or `device_path` with a `/dev/v4l/by-id/...` symlink for an explicit override.
It does not load or apply camera intrinsics, and it requests MJPG from the USB
camera to reduce bandwidth when the device supports it. The default transport is
JPEG-compressed output for higher publish rate. Raw `/camera/image` publishing is
available but disabled by default.
The default mode is `1280x720 @ 30 fps`.
`usb_grabber` opens USB cameras through the V4L2 backend so requested capture
modes are applied through the Linux camera driver.

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
    sim_hsv: false
```

Set `sim_hsv: true` for the broader sim red threshold and `sim_hsv: false`
for the stricter live/outdoor red threshold.

Mask cleanup tuning:
- `hsv_blur_kernel_px` smooths the HSV image before thresholding
- `morph_kernel_px` removes small islands and fills small holes
- `mask_blur_kernel_px` optionally smooths the final binary mask edges before contour detection

Camera type presets:

```text
rolling -> /dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_12MP_SN0001-video-index0
global  -> /dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9782_USB_Camera_UC852-video-index0
```

Global-shutter camera example:

```bash
ros2 launch drone_target_cv usb_grabber.launch.py camera_type:=global
```

Default rolling-shutter device path:

```bash
/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_12MP_SN0001-video-index0
```

Override example:

```bash
ros2 launch drone_target_cv usb_grabber.launch.py \
  device_path:=/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_12MP_SN0001-video-index0
```
