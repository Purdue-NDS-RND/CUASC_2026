# vision_pipeline

Vision utilities for image capture, target detection, and related mission support.

## Build

```bash
cd ~/dev/CUASC_2026
colcon build --packages-select vision_pipeline
source install/setup.bash
```

## Run

```bash
ros2 launch vision_pipeline vision_demo.launch.py
```

## `target_cv`

`target_cv` detects the visual target from the camera feed and publishes:

- `/drone_package_drop/target_detection` as centered normalized offsets in `[-1, 1]`
- `/drone_package_drop/image_size` as raw image width/height for observability/debugging
- `/drone_package_drop/annotated_image` as the annotated debug image

For `target_detection`, `(0.0, 0.0)` means the target is centered in the image, positive `x` means the target is right of center, and positive `y` means the target is below center. The `(-1.0, -1.0)` pair remains the sentinel for "target not found".
