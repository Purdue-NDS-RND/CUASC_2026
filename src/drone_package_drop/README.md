# drone_package_drop

Precision payload-drop mission package for CUASC 2026. Uses GPS navigation to fly to a target zone, a downward-facing camera with HSV colour detection to centre on a red/white bullseye, then descends and actuates a servo to release the payload.

## Launch

```bash
# Default (uses config/mission_params.yaml)
ros2 launch drone_package_drop payload_drop.launch.py

# Custom params file
ros2 launch drone_package_drop payload_drop.launch.py params:=config/my_params.yaml
```

The launch file starts **four** nodes:

| Node | Package | Description |
|---|---|---|
| `simple_takeoff_service` | `drone_utils` | Stream setup, arm, takeoff |
| `gimbal_point_service` | `drone_utils` | Gimbal proxy service |
| `payload_drop` | `drone_package_drop` | Main mission state machine |
| `target_cv` | `drone_package_drop` | Camera-based target detection |

## Nodes

### `payload_drop`

Timer-driven state machine that executes the full drop mission. Holds yaw = 0 (north) throughout so the downward camera stays orientation-stable.

**State flow:**
```
INIT → WAITING_FOR_CONNECTION → WAITING_FOR_GPS → TAKEOFF
     → TRANSIT_TO_TARGET → ACQUIRE_TARGET
        ├→ TARGET_NOT_FOUND (recovery TBD)
        └→ CENTER_ON_TARGET → DESCEND → DROP_PAYLOAD
              → RETURN_TO_LAUNCH → COMPLETE
```

**Subscriptions:**
| Topic | Type | Purpose |
|---|---|---|
| `/mavros/state` | `State` | Connection & mode |
| `/mavros/global_position/global` | `NavSatFix` | Drone GPS |
| `/mavros/local_position/pose` | `PoseStamped` | Local altitude |
| `/drone_package_drop/target_detection` | `PointStamped` | Target pixel (x, y) from `target_cv` |
| `/drone_package_drop/image_size` | `PointStamped` | Image dimensions from `target_cv` |

**Publications:**
| Topic | Type | Purpose |
|---|---|---|
| `/mavros/setpoint_raw/global` | `GlobalPositionTarget` | GPS position setpoint |
| `/mavros/setpoint_raw/local` | `PositionTarget` | Local velocity setpoint (centring phase) |

**Service clients:**
| Service | Type | Purpose |
|---|---|---|
| `drone_utils/takeoff` | `CommandTOL` | Arm + takeoff |
| `drone_utils/set_gimbal_point` | `GimbalManagerPitchyaw` | Point camera down |
| `/mavros/set_mode` | `SetMode` | RTL at end |
| `/mavros/cmd/command` | `CommandLong` | `MAV_CMD_DO_SET_SERVO` to release payload |

**Parameters:**
| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_latitude` | double | 0.0 | GPS latitude of drop zone |
| `target_longitude` | double | 0.0 | GPS longitude of drop zone |
| `transit_altitude_m` | double | 20.0 | Altitude for GPS transit |
| `drop_altitude_m` | double | 5.0 | Altitude to descend to before drop |
| `centering_tolerance_px` | double | 30.0 | Pixel distance from image centre to consider "centred" |
| `arrival_radius_m` | double | 3.0 | Horizontal GPS arrival radius |
| `arrival_alt_tolerance_m` | double | 2.0 | Vertical tolerance for arrival |
| `servo_channel` | int | 9 | Servo channel for drop mechanism |
| `servo_open_pwm` | int | 1900 | PWM to open/release |
| `servo_close_pwm` | int | 1100 | PWM to close/secure |
| `setpoint_rate_hz` | double | 20.0 | Control loop rate |
| `guided_mode_name` | string | `"GUIDED"` | ArduPilot mode string |
| `not_found_ascent_m` | double | 5.0 | Metres to climb when target not found |
| `centering_dwell_s` | double | 1.0 | Seconds target must stay centred before descending |
| `drop_hover_dwell_s` | double | 2.0 | Seconds to hover at drop altitude before releasing |

---

### `target_cv`

Subscribes to a camera image topic, detects a red target using HSV colour filtering and image moments, and publishes the target centre pixel.

**Subscriptions:**
| Topic | Type |
|---|---|
| `camera/image` (configurable) | `sensor_msgs/Image` |

**Publications:**
| Topic | Type | Description |
|---|---|---|
| `/drone_package_drop/target_detection` | `PointStamped` | Target centre pixel (x, y) |
| `/drone_package_drop/image_size` | `PointStamped` | Image (width, height, 0) |
| `/drone_package_drop/annotated_image` | `Image` | Debug annotated frame |

**Detection pipeline:**
1. Convert to HSV
2. Gaussian blur
3. Dual-range red mask (H 0–10 and 170–180)
4. Morphological open + close
5. Compute image moments → centroid

**Parameters:**
| Parameter | Type | Default | Description |
|---|---|---|---|
| `image_topic` | string | `"camera/image"` | Camera topic name |
| `debug_view` | bool | `False` | Show OpenCV debug window (spawns a separate thread) |

---

## Config

All parameters live in `config/mission_params.yaml` and are loaded by the launch file. The file contains three namespaced sections:

- `simple_takeoff_service.ros__parameters` — takeoff & stream setup tuning
- `payload_drop.ros__parameters` — mission-specific settings (**set `target_latitude` / `target_longitude` before flight!**)
- `target_cv.ros__parameters` — camera topic and debug toggle

## Dependencies

- `rclpy`, `geometry_msgs`, `sensor_msgs`, `mavros_msgs`
- `opencv-python` (via system or pip — used directly, not through `cv_bridge`)
- `drone_utils` (takeoff + gimbal services)
