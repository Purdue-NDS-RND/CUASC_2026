# drone_target_cv

ROS2 Python package for the USB camera and target-tracking nodes used by
`drone_mission_demo`.

Current nodes:
- `target_cv`
- `usb_grabber`
- `mipi_grabber`

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

For more consistent target colors, the USB grabber locks white balance through
OpenCV by default for the global-shutter camera profile while leaving exposure
automatic:

```yaml
usb_grabber:
  ros__parameters:
    lock_white_balance: true
    manual_white_balance: 4500
```

Image controls are also YAML-driven. Negative values leave that control at the
camera's auto/default setting:

```yaml
usb_grabber:
  ros__parameters:
    brightness: -1
    contrast: -1
    saturation: -1
    gain: -1
    auto_exposure: -1
    exposure_time_absolute: -1
```

For manual exposure, set `exposure_time_absolute` to a non-negative value. If
`auto_exposure` is not set, the node requests manual exposure automatically.

The camera driver defines the valid ranges, so check them on the target machine:

```bash
v4l2-ctl -d /dev/v4l/by-id/<camera> --list-ctrls
```

For `target_cv`, use:

```yaml
target_cv:
  ros__parameters:
    image_topic: "camera/image/compressed"
    compressed_input: true
    sim_hsv: false
```

`target_cv` detects red target clusters rather than only the largest red
contour. That lets the same implementation handle both the solid red practice
circle and the official red/white bullseye with separated rings.

Mask and cluster tuning:
- `hsv_blur_kernel_px` smooths the HSV image before thresholding
- `hsv_red*_h_*`, `hsv_s_*`, and `hsv_v_*` define the red HSV bands
- `red_dominance_ratio`, `red_difference_min`, and `red_min_channel` catch
  red pixels that shift under outdoor lighting
- `morph_kernel_px` removes small islands before target grouping
- `cluster_kernel_px` groups separated bullseye rings into one target candidate
- `min_detection_confidence` rejects weak red clusters

Debug output:
- `/target_cv/annotated` shows the selected center, cluster outline, scores,
  and rejection reason
- `/target_cv/mask` shows the selected red evidence when a target is accepted,
  otherwise the cleaned red mask used for diagnosis

Camera type presets:

```text
rolling -> /dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_12MP_SN0001-video-index0
global  -> /dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9782_USB_Camera_UC852-video-index0
```

Global-shutter camera example:

```bash
ros2 launch drone_target_cv usb_grabber.launch.py
```

The legacy standalone params file is `config/old_usb_grabber_live.yaml`; edit
`camera_type` there to switch between `global` and `rolling`.

Default rolling-shutter device path:

```bash
/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_12MP_SN0001-video-index0
```

Override example:

Set `device_path` in the selected params YAML to a stable `/dev/v4l/by-id/...`
path. A non-empty `device_path` overrides `camera_type`.
