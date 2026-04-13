# drone_mission_demo

Example mission package built on top of `drone_mission_core`.

This package proves the new multi-mission framework with a single-flight sequence:
1. takeoff
2. fly a square pattern
3. fly a zig-zag pattern
4. RTL

Square and zig-zag are not separate mission implementations. They are two configurations of the same reusable `LocalWaypointMission`.

## What It Contains

### Mission Implementations

Defined in `drone_mission_demo/missions/`:

| Mission Type | Class | Purpose |
|---|---|---|
| `takeoff` | `TakeoffMission` | Request takeoff and wait for altitude gate |
| `local_waypoint` | `LocalWaypointMission` | Fly a local ENU waypoint pattern with hold time at each point |
| `rtl` | `RTLMission` | Request RTL mode and wait for FCU mode confirmation |

### Launch File

`launch/multi_mission_demo.launch.py` starts:
- `simple_takeoff_service` from `drone_utils`
- `mission_executor` from `drone_mission_core`

### Config Layout

This package keeps mission config organized under `config/`:

| Path | Purpose |
|---|---|
| `config/params/mission_params.yaml` | ROS parameters for the takeoff service and mission executor |
| `config/patterns/square.yaml` | Reusable square waypoint pattern |
| `config/patterns/zig_zag.yaml` | Reusable zig-zag waypoint pattern |
| `config/sequences/square_then_zig_zag.yaml` | Default multi-mission sequence |
| `config/sequences/square_only.yaml` | Single-mission compatibility example |

## `LocalWaypointMission` Behavior

`LocalWaypointMission` owns its own small internal state machine:

```text
GO_TO_WAYPOINT -> HOLD_AT_WAYPOINT -> ADVANCE_WAYPOINT
```

It:
- reads a waypoint pattern from YAML
- commands local ENU setpoints through the shared `MissionContext`
- checks horizontal and vertical arrival tolerances
- holds for `hold_time_s` at each waypoint
- completes when the full pattern is exhausted

## Run

Default multi-mission sequence:

```bash
ros2 launch drone_mission_demo multi_mission_demo.launch.py
```

Choose a different sequence:

```bash
ros2 launch drone_mission_demo multi_mission_demo.launch.py \
  sequence:=config/sequences/square_only.yaml
```

Choose a different params file:

```bash
ros2 launch drone_mission_demo multi_mission_demo.launch.py \
  params:=config/params/mission_params.yaml
```

## Sequence Example

Default sequence file:

```yaml
mission_sequence:
  on_failure: abort_and_rtl
  missions:
    - type: takeoff
    - type: local_waypoint
      name: fly_square
      config:
        pattern_file: ../patterns/square.yaml
    - type: local_waypoint
      name: fly_zig_zag
      config:
        pattern_file: ../patterns/zig_zag.yaml
    - type: rtl
```

## Extension Path

This package is the example layer, not the long-term home for every mission.

Near-term expected next step:
- add a future `PackageDropMission` using the same executor/context model

That future mission can keep its own internal state machine while reusing the shared framework rather than running as a separate ROS node.
