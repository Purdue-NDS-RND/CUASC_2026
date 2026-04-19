# drone_live_tests

Standalone live-flight test missions that reuse the `drone_mission_core`
framework without any vision or payload dependencies.

## Included Mission

`live_landing_test` is intended for a manually launched vehicle:

1. manually arm and take off
2. climb above `landing_check_threshold_m`
3. launch the test package
4. let it descend in the current column, confirm touchdown while staying armed,
   wait on the ground, and relaunch to `relaunch_altitude_m`

The touchdown, debounce, ground dwell, and guided relaunch logic are copied
from the package-delivery touchdown sequence, but no camera, gimbal, or servo
logic is used.

## Run

Before launching on real hardware, request MAVLink streams and verify that
landing-state telemetry is actually reaching the Jetson:

```bash
./set_stream_rate.sh 20
ros2 topic echo /mavros/extended_state
# optional:
ros2 topic hz /mavros/extended_state
```

Do not launch the live landing test until `/mavros/extended_state` is
publishing and `landed_state` is not `UNDEFINED`. SITL often provides this
automatically; hardware may not.

```bash
ros2 launch drone_live_tests live_landing_test.launch.py
```

Override sequence or params files if needed:

```bash
ros2 launch drone_live_tests live_landing_test.launch.py \
  sequence:=config/sequences/live_landing_test.yaml \
  params:=config/params/live_landing_test_params.yaml
```
