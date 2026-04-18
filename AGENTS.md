# AGENTS.md

This file documents the new multi-mission framework that was added to the workspace so future contributors can understand what changed, where the new code lives, and how to extend it safely.

## What Changed

A new mission framework was added in **new packages only**:
- `src/drone_mission_core`
- `src/drone_mission_demo`

The existing packages were intentionally left untouched:
- `src/drone_demo`
- `src/drone_utils`
- `src/drone_control`

The goal of this change was to prove a reusable mission system that supports **multiple missions in one flight** without landing between intermediate steps.

## New Architecture

### `drone_mission_core`

This is the reusable framework layer.

Key files:
- `drone_mission_core/mission_api.py`
- `drone_mission_core/mission_context.py`
- `drone_mission_core/registry.py`
- `drone_mission_core/mission_executor.py`

Main responsibilities:
- define the mission lifecycle
- define mission statuses and failure policy
- load sequence YAML
- register mission types dynamically
- own shared ROS2 subscriptions, publishers, and service clients
- execute one mission at a time on a timer

### `drone_mission_demo`

This is the example mission layer built on top of the core framework.

Key files:
- `drone_mission_demo/missions/`
- `launch/multi_mission_demo.launch.py`
- `config/sequences/*.yaml`
- `config/patterns/*.yaml`
- `config/params/mission_params.yaml`

Current v1 missions:
- `TakeoffMission`
- `LocalWaypointMission`
- `RTLMission`

## Important Design Decisions

### Why a timer-driven executor?

Because it matches the existing repo.

The workspace already relied on:
- subscriptions
- async service clients
- enum-based state machines
- `create_timer()` loops

The new framework keeps that model instead of introducing ROS2 actions or behavior-tree tooling.

### Why missions are objects instead of nodes

The executor owns shared control surfaces:
- MAVROS subscriptions
- local position setpoint publisher
- takeoff service client
- mode service client

This avoids multiple mission nodes competing for setpoint ownership and makes it possible to chain missions in one flight cleanly.

### Why the current packages were not modified

This v1 is a proof package. It was intentionally implemented in parallel so the team can:
- compare the old and new patterns safely
- keep old demos runnable
- iterate on the framework without destabilizing current mission packages

## Current Behavior

Default example sequence:
1. `TakeoffMission`
2. `LocalWaypointMission` with square pattern
3. `LocalWaypointMission` with zig-zag pattern
4. `RTLMission`

Default failure behavior:
- abort remaining sequence
- request `RTL`

No landing is performed between intermediate missions.

## How to Run

Typical launch:

```bash
ros2 launch drone_mission_demo multi_mission_demo.launch.py
```

Single-pattern example:

```bash
ros2 launch drone_mission_demo multi_mission_demo.launch.py \
  sequence:=config/sequences/square_only.yaml
```

## How to Add a New Mission

1. Add a mission class in a package that depends on `drone_mission_core`.
2. Subclass `BaseMission`.
3. Register it with `@register_mission("your_type")`.
4. Implement:
   - `on_enter(context)`
   - `update(context)`
   - optional `on_exit(context)`
5. Return `MissionStatus` values instead of blocking.
6. Add the mission module to the executor’s `mission_modules` list.
7. Reference the mission from sequence YAML.

## How to Extend the Framework Safely

Preferred order:
1. extend `MissionContext` only when a new shared control surface is truly needed
2. keep mission-specific internal state inside the mission class
3. avoid turning missions back into standalone nodes unless there is a strong reason
4. keep sequence config YAML-first and human-editable

For future mission extensions, the likely next changes are:
- add global position setpoint support to `MissionContext`
- expand local velocity setpoint helpers where mission behaviors need them
- add gimbal control helper
- add servo / actuator helper

## Files Added

### Core package
- `src/drone_mission_core/setup.py`
- `src/drone_mission_core/setup.cfg`
- `src/drone_mission_core/package.xml`
- `src/drone_mission_core/README.md`
- `src/drone_mission_core/drone_mission_core/__init__.py`
- `src/drone_mission_core/drone_mission_core/mission_api.py`
- `src/drone_mission_core/drone_mission_core/mission_context.py`
- `src/drone_mission_core/drone_mission_core/registry.py`
- `src/drone_mission_core/drone_mission_core/mission_executor.py`

### Demo package
- `src/drone_mission_demo/setup.py`
- `src/drone_mission_demo/setup.cfg`
- `src/drone_mission_demo/package.xml`
- `src/drone_mission_demo/README.md`
- `src/drone_mission_demo/drone_mission_demo/__init__.py`
- `src/drone_mission_demo/drone_mission_demo/missions.py`
- `src/drone_mission_demo/launch/multi_mission_demo.launch.py`
- `src/drone_mission_demo/config/params/mission_params.yaml`
- `src/drone_mission_demo/config/patterns/square.yaml`
- `src/drone_mission_demo/config/patterns/zig_zag.yaml`
- `src/drone_mission_demo/config/sequences/square_then_zig_zag.yaml`
- `src/drone_mission_demo/config/sequences/square_only.yaml`

## Notes for Future Contributors

- `colcon` was not available in the implementation environment used for this change, so full ROS build verification was not run there.
- `PyYAML` is now declared explicitly in the new packages because sequence and pattern loading depend on it.
- If you later decide to migrate old missions into this framework, do it by porting mission logic into mission classes rather than wiring the old nodes together externally.
