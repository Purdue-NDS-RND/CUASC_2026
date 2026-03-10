# drone_demo

Demo / test-flight package for CUASC 2026. Flies the drone through a list of local ENU waypoints, holds at each one, then commands RTL. Useful for verifying the flight stack, tuning arrival radii, and running SITL checkout flights before real missions.

## Launch

```bash
# Default square pattern
ros2 launch drone_demo waypoint_demo.launch.py

# Zig-zag pattern
ros2 launch drone_demo waypoint_demo.launch.py config:=config/waypoint_zig_zag.yaml

# Custom config + params
ros2 launch drone_demo waypoint_demo.launch.py \
    config:=config/waypoint_zig_zag.yaml \
    params:=config/mission_params.yaml
```

The launch file starts **two** nodes:

| Node | Package | Description |
|---|---|---|
| `simple_takeoff_service` | `drone_utils` | Stream setup, arm, takeoff |
| `waypoint_demo_mission` | `drone_demo` | Waypoint state machine |

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `config` | `config/waypoint_square.yaml` | Waypoint file (relative to package share) — read directly by the node for the `waypoints` key |
| `params` | `config/mission_params.yaml` | ROS parameter file (relative to package share) |

## Node

### `waypoint_demo_mission`

Timer-driven state machine that takes off, flies through ENU waypoints, holds at each, then switches to RTL.

Waypoints are **local ENU metre offsets** from the home/arming position — no GPS or coordinate conversion required. They are read directly from the config YAML (not through ROS params) so nested lists work naturally.

**State flow:**
```
INIT → WAITING_FOR_CONNECTION → WAITING_FOR_TAKEOFF_SERVICE
     → TAKING_OFF → WAITING_FOR_ALTITUDE
     → GO_TO_WAYPOINT → HOLD_AT_WAYPOINT → ADVANCE_WAYPOINT
       (repeat for each waypoint)
     → SET_RTL → DONE
```

**Subscriptions:**
| Topic | Type | Purpose |
|---|---|---|
| `/mavros/state` | `State` | Connection & armed state |
| `/mavros/local_position/pose` | `PoseStamped` | ENU position feedback |

**Publications:**
| Topic | Type | Purpose |
|---|---|---|
| `/mavros/setpoint_position/local` | `PoseStamped` | Continuous ENU setpoint (prevents MAVROS offboard timeout) |

**Service clients:**
| Service | Type | Purpose |
|---|---|---|
| `drone_utils/takeoff` | `CommandTOL` | Arm + takeoff |
| `/mavros/set_mode` | `SetMode` | Switch to RTL at end |

**Parameters** (set in `mission_params.yaml` under `waypoint_demo_mission.ros__parameters`):

| Parameter | Type | Default | Description |
|---|---|---|---|
| `config_file` | string | `""` | Absolute path to waypoint YAML (set by launch file) |
| `takeoff_altitude_m` | double | 20.0 | Target takeoff altitude; 90 % gate before waypoints start |
| `waypoint_altitude_m` | double | 20.0 | ENU Z for all waypoints |
| `arrival_radius_m` | double | 3.0 | Horizontal arrival distance (m) |
| `arrival_height_tolerance_m` | double | 2.0 | Vertical arrival tolerance (m) |
| `hold_time_s` | double | 3.0 | Seconds to hold at each waypoint |
| `setpoint_rate_hz` | double | 20.0 | Control loop / setpoint publish rate |
| `desired_yaw_deg` | double | 90.0 | Yaw angle (0° = East, 90° = North) |
| `rtl_mode` | string | `"RTL"` | Mode string sent after last waypoint |

## Config Files

### `config/mission_params.yaml`

ROS parameters loaded by the launch file. Contains two namespaced sections:
- `simple_takeoff_service.ros__parameters` — takeoff & stream setup
- `waypoint_demo_mission.ros__parameters` — flight tuning

### Waypoint files

Each waypoint file has a top-level `waypoints` key with `[east_m, north_m]` pairs:

**`config/waypoint_square.yaml`** — 20 m square:
```yaml
waypoints:
  - [20.0, 0.0]
  - [20.0, 20.0]
  - [0.0, 20.0]
  - [0.0, 0.0]
```

**`config/waypoint_zig_zag.yaml`** — zig-zag pattern:
```yaml
waypoints:
  - [5.0, 0.0]
  - [5.0, 5.0]
  - [10.0, 5.0]
  - [10.0, 10.0]
```

To create a new pattern, copy either file and edit the waypoint list.

## Dependencies

- `rclpy`, `geometry_msgs`, `mavros_msgs`
- `launch`, `launch_ros`, `ament_index_python`
- `drone_utils` (takeoff service)
- `pyyaml` (reads waypoint files directly)
