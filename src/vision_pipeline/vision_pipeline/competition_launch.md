# Prerequisites
```bash
cd ~/dev/CUASC_2026
colcon build --packages-select vision_pipeline drone_mission_core drone_mission_demo --symlink-install
source install/setup.bash
```

# Terminal 1
```bash
cd ~/dev/CUASC_2026/
./mavros_boot.sh device
```

# Terminal 2
```bash
cd ~/dev/CUASC_2026/src/vision_pipeline/vision_pipeline
./vision.sh
```

# Terminal 3
```bash
cd ~/dev/CUASC_2026
source install/setup.bash
ros2 run drone_mission_core mission_executor --ros-args \
  -p mission_modules:="['drone_mission_demo.missions', 'vision_pipeline.object_localization_mission']" \
  -p sequence_file:=/home/nds2/dev/CUASC_2026/src/vision_pipeline/config/competition_run.yaml
```
