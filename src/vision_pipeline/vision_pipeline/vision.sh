#!/bin/bash
echo "🚀 Pre-flight sequence initiated..."

# 1. Source the base ROS 2 installation
source /opt/ros/humble/setup.bash

# 2. Source your custom workspace
source ~/dev/CUASC_2026/install/setup.bash

# 3. Activate the YOLO virtual environment
source ~/dev/VTOL-Project/src/yolo_models/yolo_env/bin/activate

# 4. Enforce the PyTorch path (just in case the .bashrc misses it)
export PYTHONPATH=$HOME/dev/VTOL-Project/src/yolo_models/yolo_env/lib/python3.10/site-packages:$PYTHONPATH

sudo systemctl restart nvargus-daemon

echo "✅ Environment configured. Launching pipeline!"

# 5. Launch all three nodes
ros2 launch vision_pipeline vision_demo.launch.py
