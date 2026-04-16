# drone_mission_demo

Example mission package built on top of `drone_mission_core`.

This package proves the new multi-mission framework with a single-flight sequence:
1. takeoff
2. fly a square pattern
3. fly a zig-zag pattern
4. RTL

Square and zig-zag are not separate mission implementations. They are two configurations of the same reusable `LocalWaypointMission`.

The package also includes a package-drop demo sequence:
1. takeoff
2. transit to the drop zone
3. visually track the target while descending
4. release the payload
5. RTL

It also includes a package-delivery touch-and-go sequence:
1. takeoff
2. transit to the delivery zone
3. visually track and descend onto the target
4. deliver after touchdown while staying armed
5. relaunch and continue

## What It Contains

### Mission Implementations

Defined in `drone_mission_demo/missions/`:

| Mission Type | Class | Purpose |
|---|---|---|
| `takeoff` | `TakeoffMission` | Request takeoff and wait for altitude gate |
| `local_waypoint` | `LocalWaypointMission` | Fly a local ENU waypoint pattern with hold time at each point |
| `package_delivery` | `PackageDeliveryMission` | Fly to a GPS target, touch down on it while armed, deliver, and relaunch |
| `package_drop` | `PackageDropMission` | Fly to a GPS target, visually track while descending, and release payload |
| `rtl` | `RTLMission` | Request RTL mode and wait for FCU mode confirmation |

### Launch File

`launch/multi_mission_demo.launch.py` starts:
- `simple_takeoff_service` from `drone_utils`
- `mission_executor` from `drone_mission_core`

`launch/package_drop_demo.launch.py` starts the shared vision-guided delivery stack:
- `simple_takeoff_service` from `drone_utils`
- `gimbal_point_service` from `drone_utils`
- `target_cv` from `vision_pipeline`
- `mission_executor` from `drone_mission_core`

### Config Layout

This package keeps mission config organized under `config/`:

| Path | Purpose |
|---|---|
| `config/params/mission_params.yaml` | ROS parameters for the takeoff service and mission executor |
| `config/params/package_drop_params.yaml` | ROS parameters for the package-drop launch stack |
| `config/patterns/square.yaml` | Reusable square waypoint pattern |
| `config/patterns/zig_zag.yaml` | Reusable zig-zag waypoint pattern |
| `config/sequences/package_delivery_demo.yaml` | Armed touchdown delivery mission sequence |
| `config/sequences/package_drop_demo.yaml` | Package-drop mission sequence |
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

## `PackageDropMission` Behavior

`PackageDropMission` owns a vision-guided drop flow:

```text
TRANSIT_TO_TARGET -> ACQUIRE_TARGET -> TRACK_AND_DESCEND -> DROP_PAYLOAD
```

It:
- flies to the configured GPS drop zone
- enables `target_cv` on mission entry and disables it on mission exit
- waits for a stable visual lock from `target_cv`
- commands XY correction and vertical descent simultaneously through shared local velocity setpoints
- bounds target-loss recovery with `max_recovery_altitude_m` and `max_recovery_attempts`
- supports `fake_drop: true` for simulation-only testing, which still uses GPS transit and vision tracking but skips the real servo actuation step after the normal drop hover
- can use `failure_policy: continue_to_next` so a failed drop falls through to the next mission in the sequence

## `PackageDeliveryMission` Behavior

`PackageDeliveryMission` keeps the same early visual-acquisition flow as `PackageDropMission` but changes the endgame:

```text
TRANSIT_TO_TARGET -> ACQUIRE_TARGET -> TRACK_AND_DESCEND
-> TOUCHDOWN_CONFIRM -> DELIVER_PAYLOAD -> RELAUNCH
```

It:
- enables `target_cv` on mission entry and disables it on mission exit
- flies to the configured GPS delivery zone and acquires the visual target
- performs an armed touch-and-go instead of switching to `LAND`
- confirms touchdown with a short altitude-stability dwell before delivery
- actuates the servo, or uses `fake_drop: true`, only after touchdown confirmation
- relaunches to `relaunch_altitude_m` and completes airborne so the next mission can continue
- preserves the original RTL/home point by avoiding disarm and re-arm inside the mission

This v1 assumes the delivery target is on roughly the same ground plane as the original launch location.

Delivery-specific config keys:
- `touchdown_altitude_m` — local-altitude threshold used to begin touchdown confirmation
- `touchdown_dwell_s` — stable-on-target dwell before delivery
- `relaunch_altitude_m` — altitude to climb back to after delivery
- `final_descent_rate_mps` — slower final descent rate near touchdown

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

Run the package-drop demo:

```bash
ros2 launch drone_mission_demo package_drop_demo.launch.py
```

Run the package-delivery demo on the same launch stack:

```bash
ros2 launch drone_mission_demo package_drop_demo.launch.py \
  sequence:=config/sequences/package_delivery_demo.yaml
```

The default `package_drop_demo.yaml` is configured for simulation with `fake_drop: true` and an offset GPS target so the mission still performs transit and vision-guided tracking before simulating the release. Set `fake_drop: false` in `config/sequences/package_drop_demo.yaml` to exercise the real servo actuation path.

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
- continue porting additional mission behaviors into reusable mission classes built on the same executor/context model

New missions should keep their own internal state machines while reusing the shared framework rather than running as separate ROS nodes.
