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
- `target_cv` from `drone_target_cv`
- `session_logger` from `drone_utils`
- `mission_executor` from `drone_mission_core`

`launch/package_delivery_demo.launch.py` starts the same stack but defaults to the package-delivery sequence.

`launch/package_drop_live.launch.py` starts the live-flight version of that stack:
- `mipi_grabber` from `drone_target_cv`
- `simple_takeoff_service` from `drone_utils`
- `target_cv` from `drone_target_cv`
- `mission_executor` from `drone_mission_core`

`launch/package_delivery_live.launch.py` starts the same live stack but defaults to the package-delivery live sequence.

### Config Layout

This package keeps mission config organized under `config/`:

| Path | Purpose |
|---|---|
| `config/params/mission_params.yaml` | ROS parameters for the takeoff service and mission executor |
| `config/params/live_target_mission.yaml` | Shared ROS parameters for the live MIPI-camera vision stack |
| `config/params/sim_target_mission.yaml` | Shared ROS parameters for the sim vision stack |
| `config/sequences/package_delivery_live.yaml` | Live-flight package-delivery template sequence |
| `config/patterns/square.yaml` | Reusable square waypoint pattern |
| `config/patterns/zig_zag.yaml` | Reusable zig-zag waypoint pattern |
| `config/sequences/package_delivery_demo.yaml` | Armed touchdown delivery mission sequence |
| `config/sequences/package_drop_demo.yaml` | Package-drop mission sequence |
| `config/sequences/package_drop_live.yaml` | Live-flight package-drop template sequence |
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
TRANSIT_TO_TARGET -> ACQUIRE_TARGET -> TRACK_AND_DESCEND
-> FINAL_FIXED_DROP_COLUMN -> DROP_PAYLOAD
```

It:
- flies to the configured GPS drop zone
- enables `target_cv` on mission entry and disables it on mission exit
- waits for a stable visual lock from `target_cv`
- uses centered normalized target offsets from `target_cv` so the XY control law does not depend on camera resolution
- commands XY correction and vertical descent simultaneously through shared local velocity setpoints
- holds briefly on target-loss flicker before climbing for recovery
- freezes the current GPS lat/lon for the final drop column once low and centered
- bounds target-loss recovery with `max_recovery_altitude_m` and `max_recovery_attempts`
- supports `fake_drop: true` for simulation-only testing, which still uses GPS transit and vision tracking but skips the real sprayer release command after the normal drop hover
- can use `failure_policy: continue_to_next` so a failed drop falls through to the next mission in the sequence

Drop-specific vision tuning keys:
- `drop_column_handoff_altitude_m` — altitude below which a centered target commits to a fixed GPS drop column
- `drop_altitude_tolerance_m` — altitude tolerance for accepting the release height

## `PackageDeliveryMission` Behavior

`PackageDeliveryMission` keeps the same early visual-acquisition flow as `PackageDropMission` but changes the endgame:

```text
TRANSIT_TO_TARGET -> ACQUIRE_TARGET -> TRACK_AND_DESCEND
-> FINAL_FIXED_COLUMN_DESCENT -> GROUND_DWELL -> GUIDED_RELAUNCH
```

It:
- enables `target_cv` on mission entry and disables it on mission exit
- flies to the configured GPS delivery zone and acquires the visual target
- performs an armed touch-and-go in `GUIDED` instead of switching to `LAND`
- keeps visual XY corrections active using centered normalized target offsets, so tracking behavior stays consistent across camera resolutions
- keeps visual XY corrections active until the vehicle is low and centered
- freezes the current GPS lat/lon at the handoff height and descends on that fixed column
- confirms touchdown from `/mavros/extended_state` rather than guessing from altitude stall
- releases the gripper, or uses `fake_drop: true`, only after touchdown confirmation
- holds on the ground for `delivery_dwell_s` before relaunching
- relaunches to `relaunch_altitude_m` and completes airborne so the next mission can continue
- preserves the original RTL/home point by avoiding disarm and re-arm inside the mission

This mission requires the FCU to stay armed during the ground dwell. Configure
ArduPilot `DISARM_DELAY` accordingly. A safe starting point is
`DISARM_DELAY >= delivery_dwell_s + 5`, with `15-20 s` recommended for a
`5-10 s` dwell.

Final touchdown setpoints may go below the takeoff/home-relative zero altitude so slightly lower delivery targets do not stall just above the ground.
The real actuator path now uses MAVROS gripper and sprayer commands; until the
hardware flow is fully validated, `fake_drop: true` remains the safer demo setting.

Delivery-specific config keys:
- `landing_check_threshold_m` — local-altitude threshold where the mission freezes the current GPS position and starts the fixed-column touchdown
- `touchdown_handoff_tolerance_m` — separate altitude tolerance used only for the fixed-column touchdown handoff so this transition is not coupled to the broader arrival-altitude tolerance
- `touchdown_dwell_s` — landed-state debounce before the mission accepts touchdown
- `touchdown_min_altitude_m` — lowest home-relative global setpoint allowed during final touchdown; use a negative value when the target ground may be below the takeoff point
- `delivery_dwell_s` — time to remain on the ground before relaunch
- `relaunch_altitude_m` — altitude to climb back to after delivery
- `centering_tolerance_norm` — centered-error magnitude in normalized image units required before descent continues
- `centering_gain_mps_per_norm` — horizontal velocity gain in m/s per normalized error unit
- `guided_relaunch_rate_mps` — positive climb-rate request used to lift off from the ground in `GUIDED`
- `guided_relaunch_max_climb_rate_mps` — max climb rate used to map the relaunch request onto ArduPilot's `SET_ATTITUDE_TARGET` thrust field
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

Run the package-delivery demo:

```bash
ros2 launch drone_mission_demo package_delivery_demo.launch.py
```

The default `package_drop_demo.yaml` is configured for simulation with `fake_drop: true` and an offset GPS target so the mission still performs transit and vision-guided tracking before simulating the release. Set `fake_drop: false` in `config/sequences/package_drop_demo.yaml` to exercise the real sprayer actuation path.

Run the live package-drop stack:

```bash
ros2 launch drone_mission_demo package_drop_live.launch.py
```

Run the live package-delivery stack:

```bash
ros2 launch drone_mission_demo package_delivery_live.launch.py
```

The live launch files boot `mipi_grabber` and feed `target_cv` from `/camera/image/compressed` with `debug_view: true`. Live camera/CV/logger defaults live in `config/params/live_target_mission.yaml`: `1280x720`, `fps: 60`, `image_publishing_rate: 30.0`, `camera_info_file: mipi_info.yaml`, and `sim_hsv: false`. The previous USB live path is preserved as `old_package_drop_live.launch.py`, `old_package_delivery_live.launch.py`, and `config/params/old_live_target_mission.yaml`. Demo launches use `config/params/sim_target_mission.yaml`, including `sim_hsv: true` and the demo logger settings. `package_drop_live.yaml` uses real sprayer actuation with `fake_drop: false`; the demo sequences and `package_delivery_live.yaml` keep `fake_drop: true`. The live sequence coordinates are templates only: edit `target_latitude` and `target_longitude` before real flight.

Use the workspace helper to fill both live sequence coordinate templates:

```bash
./set_live_mission_coords.py
```

Choose `manual` to type the target latitude and longitude, or choose `auto` to read the current FCU GPS fix from `/mavros/global_position/global`. By default the helper updates the source YAMLs only; pass `--also-install` if you need the already-built `install/share` copies updated before rebuilding.

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
