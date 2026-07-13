# CUASC 2026 Autonomous Drone Platform

CUASC 2026 is a competition-oriented autonomous aerial robotics platform built with ROS 2, ArduPilot, MAVROS, and computer vision. It coordinates complete drone missions—from takeoff and waypoint navigation to visual target acquisition, payload delivery, relaunch, and return-to-launch—through a reusable mission framework designed for both simulation and real hardware.

The project’s central challenge is not making a drone perform one behavior in isolation. It is making perception, flight control, mission logic, and payload hardware work together reliably throughout a multi-stage flight.

> This repository is under active development. Flight code and hardware procedures should be validated in simulation and reviewed against the team’s preflight process before use on a real aircraft.

## What the System Does

- Runs multiple missions during one flight without landing between stages unless the mission explicitly requires it.
- Loads mission order and tuning from human-readable YAML instead of hard-coding a flight plan.
- Navigates local ENU patterns and global GPS waypoints through MAVROS setpoints.
- Detects red visual targets from USB or MIPI camera feeds and publishes resolution-independent alignment errors.
- Supports vision-guided payload drops and armed touch-and-go deliveries with automatic relaunch.
- Centralizes telemetry, setpoint ownership, mode changes, takeoff, and actuator commands to prevent competing controllers.
- Provides simulation configurations, mission logging, telemetry readiness checks, and focused live-flight tests.

## Architecture

```mermaid
flowchart LR
    Config["YAML mission sequences<br/>and parameters"] --> Executor["Mission executor"]
    Registry["Mission registry"] --> Executor
    Executor --> Mission["Active mission object<br/>timer-driven state machine"]

    Camera["USB / MIPI camera"] --> CV["Target detection"]
    CV --> Context["Shared mission context"]
    Mission <--> Context

    Context <--> MAVROS["MAVROS interface"]
    MAVROS <--> FCU["ArduPilot flight controller"]
    Context --> Payload["Gimbal / payload actuators"]

    Executor --> Logs["Session telemetry<br/>and debug imagery"]
```

The system uses one timer-driven executor rather than one ROS node per mission. Missions are lightweight Python objects with a common lifecycle:

```text
on_enter(context) -> update(context) -> on_exit(context)
```

Each mission returns a status such as `RUNNING`, `SUCCESS`, or `FAILURE`. The executor advances the sequence, continuously republishes the active setpoint, and applies the configured failure policy—normally aborting the remaining sequence and requesting RTL.

This design keeps behavior modular while ensuring that only one component owns flight-control outputs at a time.

## Example Mission

A package-delivery flight can be described as:

1. Establish telemetry, enter `GUIDED`, arm, and take off.
2. Fly through configured GPS waypoints to the delivery area.
3. Enable computer vision and acquire the ground target.
4. Correct horizontal error while descending toward the target.
5. Commit to a fixed final column, confirm touchdown, and release the payload.
6. Remain armed, relaunch to a safe altitude, continue the sequence, and RTL.

The same executor can instead run local waypoint patterns, circuit time trials, package drops, or isolated hardware-test missions by selecting a different YAML sequence.

## Repository Layout

| Package | Responsibility |
|---|---|
| [`drone_mission_core`](src/drone_mission_core) | Mission lifecycle API, dynamic registry, YAML sequence loader, shared ROS interfaces, and timer-driven executor |
| [`drone_mission_demo`](src/drone_mission_demo) | Takeoff, local/GPS waypoint, package-drop, package-delivery, and RTL mission implementations |
| [`drone_target_cv`](src/drone_target_cv) | USB/MIPI image capture and red-target detection for closed-loop alignment |
| [`drone_utils`](src/drone_utils) | Takeoff orchestration, MAVLink stream setup, gimbal control, session logging, and simulation helpers |
| [`drone_live_tests`](src/drone_live_tests) | Focused real-aircraft tests that exercise risky behaviors independently of a full mission |
| [`vision_pipeline`](src/vision_pipeline) | Broader image capture, inference, geolocation, clustering, and offline photogrammetry experiments |

More detailed design documentation is available in [`Architecture.md`](Architecture.md). Package-level READMEs describe the interfaces and configuration for each subsystem.

## Engineering Highlights

### Reusable mission composition

Mission classes separate behavior from infrastructure. A registry discovers mission types, while YAML controls their order and per-flight configuration. The same `LocalWaypointMission`, for example, can fly a square or zig-zag pattern without duplicating control code.

### Non-blocking control

Flight behaviors are explicit state machines driven by ROS timers and asynchronous service calls. The executor remains responsive to telemetry and can enforce failure behavior while a mission is in progress.

### Simulation-to-hardware path

The software targets Gazebo/ArduPilot SITL as well as a Jetson-connected flight controller. Camera sources and launch configurations can be swapped without changing the mission API, and hardware preflight scripts verify that required MAVROS telemetry is available before flight.

### Safety and observability

The framework supports abort-and-RTL policies, bounded target-loss recovery, touchdown confirmation from flight-controller state, configurable fake payload releases, timestamped command/image logs, and standalone live tests for high-risk transitions.

## Technology

- Python and ROS 2 Humble
- ArduPilot, MAVROS, and MAVLink
- Gazebo Harmonic and ArduPilot SITL
- OpenCV and NumPy
- USB and MIPI camera pipelines
- YAML-driven mission and parameter configuration
- Jetson companion-computer deployment

## Getting Started

The full environment instructions have been moved to [`SETUP.md`](SETUP.md). Jetson-specific configuration is documented in [`JETSON_SETUP.md`](JETSON_SETUP.md).

After installing the required ROS 2, MAVROS, Gazebo, and ArduPilot dependencies:

```bash
colcon build
source install/setup.bash
ros2 launch drone_mission_demo multi_mission_demo.launch.py
```

Run a specific sequence by overriding the launch argument:

```bash
ros2 launch drone_mission_demo multi_mission_demo.launch.py \
  sequence:=config/sequences/square_only.yaml
```

Real-aircraft operation requires the appropriate vehicle configuration, mission parameters, and preflight telemetry checks; see the setup and package-specific documentation before attempting a live run.

## Extending the Platform

New behaviors can be added without changing the executor:

1. Subclass `BaseMission`.
2. Register the class with `@register_mission("mission_type")`.
3. Implement its non-blocking lifecycle methods.
4. Add the module and mission configuration to a sequence YAML file.

The result is a flight stack where new mission logic remains isolated, testable, and composable with existing behaviors.
