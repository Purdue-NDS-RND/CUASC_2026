# drone_control

ROS 2 package for MAVROS-based drone control, target spawning, and computer vision-based target localization.

Developed for the **C-UASC 2026** competition at Mojave Air & Space Port.

---

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Nodes](#nodes)
  - [simple_takeoff](#simple_takeoff)
  - [target_spawner](#target_spawner)
  - [waypoint_follower](#waypoint_follower)
  - [target_follower](#target_follower)
  - [target_localizer](#target_localizer)
  - [target_visualizer](#target_visualizer)
- [Launch Files](#launch-files)
- [Quick Start](#quick-start)
- [Topics & Services](#topics--services)
- [Things to Watch Out For](#things-to-watch-out-for)
- [Camera Configuration](#camera-configuration)

---

## Overview

This package provides:

| Node | Purpose |
|------|---------|
| `simple_takeoff` | Service-driven takeoff with MAVROS (mode → arm → takeoff) |
| `target_spawner` | Spawn targets in Gazebo, publish position with simulated sensor noise |
| `waypoint_follower` | **Main demo node** - flies through waypoints using noisy positions |
| `target_follower` | (Legacy) Simple target follower with setpoint streaming |
| `target_localizer` | Convert CV detections (pixels) to GPS coordinates |
| `target_visualizer` | Live matplotlib plot of localization results |

---

## Prerequisites

- **ROS 2 Humble**
- **MAVROS** (connected to ArduPilot SITL or real FCU)
- **Gazebo Harmonic** (for simulation with target_spawner)
- **Python 3.10+** with `matplotlib` (for target_visualizer)

Ensure MAVROS is running and connected before starting these nodes:
```bash
ros2 topic echo /mavros/state --once
```

---

## Installation

```bash
cd ~/CUASC_2026
colcon build --packages-select drone_control --symlink-install
source install/setup.bash
```

---

## Nodes

### simple_takeoff

Handles mode switching, arming, and takeoff via MAVROS. Can be triggered automatically or via service.

**Run:**
```bash
ros2 run drone_control simple_takeoff
```

**Service:**
```bash
# Trigger takeoff to 25m altitude
ros2 service call /drone_control/takeoff mavros_msgs/srv/CommandTOL \
  "{altitude: 25.0, min_pitch: 0.0, yaw: 0.0}"
```

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `takeoff_altitude_m` | 20.0 | Default takeoff altitude |
| `auto_takeoff` | true | Takeoff automatically on startup |
| `arm_on_start` | true | Arm automatically |
| `guided_mode_name` | "GUIDED" | Flight mode to set |
| `max_arm_attempts` | 5 | Max arming retries |

**Example with params:**
```bash
ros2 run drone_control simple_takeoff --ros-args \
  -p takeoff_altitude_m:=30.0 \
  -p auto_takeoff:=false
```

---

### target_spawner

Spawns a colored box in Gazebo at random positions and publishes the position with optional distance-based noise simulation.

**Run:**
```bash
ros2 run drone_control target_spawner
```

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_name` | "map" | Gazebo world name |
| `radius_m` | 30.0 | Spawn radius from origin |
| `hover_radius_m` | 2.0 | Distance threshold for "arrived" |
| `hover_duration_s` | 5.0 | Time at target before respawn |
| `simulate_accuracy` | true | Add distance-based noise |
| `max_noise_m` | 5.0 | Max noise when far |
| `min_noise_m` | 0.1 | Min noise when close |
| `noise_falloff_dist_m` | 50.0 | Distance at which noise is max |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_control/target/pose` | `PoseStamped` | True target position |
| `/drone_control/target/pose_noisy` | `PoseStamped` | Noisy position (distance-based) |
| `/drone_control/target/gps` | `NavSatFix` | GPS coordinates |

**Services:**

| Service | Type | Description |
|---------|------|-------------|
| `/drone_control/respawn_target` | `Trigger` | Delete current target, spawn new one |

---

### waypoint_follower

**This is the main demo node.** State machine that handles the complete flight:
1. Waits for MAVROS connection
2. Sets GUIDED mode and arms
3. Calls takeoff service
4. Flies to waypoints using noisy position stream
5. Hovers at waypoint, then requests respawn for next waypoint
6. Repeats until max_waypoints or shutdown

**Run:**
```bash
ros2 run drone_control waypoint_follower
```

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `takeoff_altitude_m` | 20.0 | Initial takeoff altitude |
| `waypoint_altitude_m` | 20.0 | Altitude to fly above each target |
| `arrival_radius_m` | 3.0 | Horizontal distance to consider "arrived" |
| `arrival_height_tolerance_m` | 2.0 | Vertical tolerance for arrival |
| `hover_time_s` | 3.0 | Hover duration at each waypoint |
| `max_waypoints` | 0 | Max waypoints (0 = unlimited) |
| `return_to_launch` | false | RTL after max waypoints |
| `use_noisy_position` | true | Use noisy position (simulates sensors) |

**Example:**
```bash
ros2 run drone_control waypoint_follower --ros-args \
  -p takeoff_altitude_m:=25.0 \
  -p waypoint_altitude_m:=15.0 \
  -p arrival_radius_m:=2.0 \
  -p max_waypoints:=5 \
  -p return_to_launch:=true
```

---

### target_follower

**(Legacy)** Simple follower that subscribes to target position and publishes setpoints. Use `waypoint_follower` instead for the full demo.

**Run:**
```bash
ros2 run drone_control target_follower
```

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_topic` | `/drone_control/target/pose` | Topic to follow |
| `hover_altitude_m` | 20.0 | Altitude above target |
| `setpoint_rate_hz` | 10.0 | Setpoint publish rate |

---

### target_localizer

Converts computer vision detections (pixel coordinates) to world GPS coordinates using camera intrinsics and drone pose.

**Run:**
```bash
ros2 run drone_control target_localizer
```

**Detection Input Format** (`geometry_msgs/PointStamped` on `/drone_control/detection`):
- `header.frame_id`: Target ID as string (e.g., "1", "2", "target_A")
- `point.x`: Pixel u (column)
- `point.y`: Pixel v (row)  
- `point.z`: Confidence (0.0-1.0)

**Example detection publisher (from your CV node):**
```python
from geometry_msgs.msg import PointStamped

msg = PointStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = "1"  # Target ID
msg.point.x = 640.0  # Pixel u
msg.point.y = 360.0  # Pixel v
msg.point.z = 0.95   # Confidence
detection_pub.publish(msg)
```

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_fx` | 424.0 | Focal length X (pixels) |
| `camera_fy` | 424.0 | Focal length Y (pixels) |
| `camera_cx` | 640.0 | Principal point X |
| `camera_cy` | 360.0 | Principal point Y |
| `image_width` | 1280 | Image width |
| `image_height` | 720 | Image height |
| `camera_pitch_offset_deg` | 0.0 | 0 = straight down |
| `ground_altitude_m` | 0.0 | Ground plane Z in local frame |
| `observation_buffer_size` | 50 | Points per target for filtering |
| `min_confidence` | 0.3 | Minimum detection confidence |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_control/targets/estimates` | `PoseArray` | All target estimates (local) |
| `/drone_control/targets/estimates_gps` | `String` | JSON with GPS per target |
| `/drone_control/targets/observations` | `PoseArray` | Raw observations |
| `/drone_control/targets/markers` | `MarkerArray` | RViz visualization |

**GPS JSON Output Format:**
```json
{
  "timestamp": 1706450000.123,
  "home": {"latitude": 34.123, "longitude": -117.456, "altitude": 100.0},
  "targets": {
    "1": {
      "local": {"x": 10.5, "y": -5.2, "z": 0.0},
      "gps": {"latitude": 34.1231, "longitude": -117.4559, "altitude": 100.0},
      "num_observations": 45,
      "last_seen": 1706450000.100
    }
  }
}
```

---

### target_visualizer

Live matplotlib plot showing target observations and estimates in a top-down view.

**Run:**
```bash
ros2 run drone_control target_visualizer
```

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate_hz` | 5.0 | Plot refresh rate |
| `history_size` | 200 | Max points shown per target |
| `auto_scale` | true | Auto-fit view to data |
| `plot_bounds_m` | 100.0 | Fixed bounds if auto_scale=false |
| `show_drone` | true | Show drone position marker |
| `marker_size_obs` | 10 | Size of observation dots |
| `marker_size_est` | 150 | Size of estimate markers |

Close the matplotlib window to gracefully shutdown the node.

---

## Launch Files

### waypoint_demo.launch.py ⭐ (Recommended)

**Main demo launch.** Launches all three nodes for complete waypoint navigation with noise simulation:
- `simple_takeoff` - provides takeoff service
- `target_spawner` - spawns targets, publishes noisy positions
- `waypoint_follower` - flies through waypoints

```bash
ros2 launch drone_control waypoint_demo.launch.py
```

**Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `takeoff_altitude_m` | "20.0" | Initial takeoff altitude |
| `waypoint_altitude_m` | "20.0" | Altitude above each target |
| `spawn_radius_m` | "50.0" | Radius to spawn targets |
| `arrival_radius_m` | "3.0" | Arrival detection radius |
| `hover_time_s` | "3.0" | Hover time at each waypoint |
| `max_waypoints` | "0" | Max waypoints (0 = unlimited) |
| `return_to_launch` | "false" | RTL after max waypoints |
| `world_name` | "map" | Gazebo world name |
| `use_noisy_position` | "true" | Use noisy position stream |
| `max_noise_m` | "5.0" | Max noise when far |
| `min_noise_m` | "0.1" | Min noise when close |
| `noise_falloff_dist_m` | "50.0" | Distance for max noise |

**Examples:**
```bash
# Basic usage
ros2 launch drone_control waypoint_demo.launch.py

# 5 waypoints at 30m altitude, then RTL
ros2 launch drone_control waypoint_demo.launch.py \
  takeoff_altitude_m:=30.0 \
  waypoint_altitude_m:=30.0 \
  max_waypoints:=5 \
  return_to_launch:=true

# Large area with more noise
ros2 launch drone_control waypoint_demo.launch.py \
  spawn_radius_m:=100.0 \
  max_noise_m:=10.0 \
  noise_falloff_dist_m:=80.0
```

---

### target_demo.launch.py (Legacy)

Launches `target_spawner` and `target_follower` together. Use `waypoint_demo.launch.py` instead.

```bash
ros2 launch drone_control target_demo.launch.py
```

**Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `spawn_model` | "true" | Spawn Gazebo model |
| `radius_m` | "30.0" | Spawn radius |
| `hover_radius_m` | "2.0" | Arrival threshold |
| `hover_duration_s` | "5.0" | Hover time before respawn |
| `target_altitude_m` | "20.0" | Hover altitude |

---

## Quick Start

### Waypoint Demo (Recommended)

```bash
# Terminal 1: Start Gazebo + ArduPilot SITL + MAVROS
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

# Terminal 2: Run the waypoint demo
source ~/CUASC_2026/install/setup.bash
ros2 launch drone_control waypoint_demo.launch.py
```

The drone will:
1. Take off to 20m
2. Fly to a randomly spawned target (using noisy position)
3. Hover for 3 seconds
4. Delete the target, spawn a new one
5. Repeat indefinitely

### Waypoint Demo with Limits

```bash
# 5 waypoints, return home when done
ros2 launch drone_control waypoint_demo.launch.py \
  max_waypoints:=5 \
  return_to_launch:=true
```

### Target Localization Pipeline

```bash
# Terminal 1: Ensure MAVROS is running

# Terminal 2: Run localizer
ros2 run drone_control target_localizer --ros-args \
  -p camera_fx:=424.0 \
  -p camera_fy:=424.0

# Terminal 3: Run visualizer
ros2 run drone_control target_visualizer

# Terminal 4: (Your CV node publishes to /drone_control/detection)
```

### Service-Based Takeoff

```bash
# Terminal 1: Run takeoff node (no auto takeoff)
ros2 run drone_control simple_takeoff --ros-args -p auto_takeoff:=false

# Terminal 2: Trigger takeoff when ready
ros2 service call /drone_control/takeoff mavros_msgs/srv/CommandTOL \
  "{altitude: 20.0}"
```

---

## Topics & Services

### Subscriptions (all nodes)

| Topic | Type | Used By |
|-------|------|---------|
| `/mavros/state` | `State` | simple_takeoff, waypoint_follower |
| `/mavros/local_position/pose` | `PoseStamped` | all nodes |
| `/mavros/home_position/home` | `HomePosition` | target_spawner, target_localizer |
| `/drone_control/target/pose_noisy` | `PoseStamped` | waypoint_follower |

### Publications

| Topic | Type | Publisher |
|-------|------|-----------|
| `/mavros/setpoint_position/local` | `PoseStamped` | simple_takeoff, waypoint_follower |
| `/drone_control/target/pose` | `PoseStamped` | target_spawner (true position) |
| `/drone_control/target/pose_noisy` | `PoseStamped` | target_spawner (noisy position) |
| `/drone_control/targets/estimates_gps` | `String` | target_localizer |
| `/drone_control/targets/markers` | `MarkerArray` | target_localizer |

### Services

| Service | Type | Provider | Description |
|---------|------|----------|-------------|
| `/drone_control/takeoff` | `CommandTOL` | simple_takeoff | Trigger takeoff |
| `/drone_control/respawn_target` | `Trigger` | target_spawner | Delete current, spawn new target |

---

## Things to Watch Out For

### ⚠️ MAVROS Connection

- **Always check MAVROS is connected** before running nodes:
  ```bash
  ros2 topic echo /mavros/state --once
  ```
- If `connected: false`, nodes will wait/retry but may timeout.

### ⚠️ Gazebo World Name

- `target_spawner` defaults to world name `"map"`.
- If your world has a different name, spawning will fail silently.
- Check with: `gz topic -l | grep world`
- Override with: `-p world_name:=your_world_name`

### ⚠️ Home Position Required

- `target_localizer` and `target_spawner` need a valid home position for GPS conversion.
- Home is set when ArduPilot arms. If you see `NaN` GPS values, ensure the drone has armed at least once.
- Check with: `ros2 topic echo /mavros/home_position/home --once`

### ⚠️ Camera Calibration

- Default camera intrinsics assume **1280×720** resolution with **120° FOV**.
- If using different resolution, recalculate:
  ```python
  # For 120° diagonal FOV:
  # f = (image_width / 2) / tan(fov_horizontal / 2)
  # For 1920x1080: fx = fy = 636
  # For 1280x720:  fx = fy = 424
  # For 640x480:   fx = fy = 212
  ```

### ⚠️ Detection Format

- Localizer expects detections on `/drone_control/detection` as `PointStamped`.
- **Target ID goes in `frame_id`**, not the actual frame name!
- Pixels in `point.x` (u) and `point.y` (v), confidence in `point.z`.

### ⚠️ Coordinate Frames

- All local positions use **ENU (East-North-Up)** frame.
- X = East, Y = North, Z = Up.
- GPS conversion assumes WGS84 ellipsoid.

### ⚠️ Setpoint Streaming (ArduPilot)

- ArduPilot requires continuous setpoint streaming before arming in GUIDED mode.
- `simple_takeoff` handles this with `setpoint_warmup_s` (default 2s).
- If arming fails, increase warmup time.

### ⚠️ matplotlib Backend

- `target_visualizer` uses `TkAgg` backend.
- If running headless (SSH without X11), it will crash.
- For headless: use RViz with `/drone_control/targets/markers` instead.

### ⚠️ Multiple Targets

- Localizer tracks targets by ID (from detection `frame_id`).
- Each target gets its own observation buffer and color.
- Old targets are never removed—restart node to clear.

---

## Camera Configuration

For Arducam IMX519 with 120° M12 lens:

| Resolution | fx = fy | cx | cy |
|------------|---------|----|----|
| 1920×1080 | 636 | 960 | 540 |
| 1280×720 | 424 | 640 | 360 |
| 640×480 | 212 | 320 | 240 |

Example for 1080p:
```bash
ros2 run drone_control target_localizer --ros-args \
  -p camera_fx:=636.0 \
  -p camera_fy:=636.0 \
  -p camera_cx:=960.0 \
  -p camera_cy:=540.0 \
  -p image_width:=1920 \
  -p image_height:=1080
```

---

## License

TODO: Add license

---

## Maintainer

**Parth Patel** - pate2293@purdue.edu  
Purdue NDS R&D
