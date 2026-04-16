# 1. Build and source the vision_pipeline

``bash
cd ~/dev/CUASC_2026
colcon build --packages-select vision_pipeline
source install/setup.bash
``

# 2. Live Flight Commands (Physical Camera Attached)
``bash
ros2 launch vision_pipeline vision_demo.launch.py
``

#3.
