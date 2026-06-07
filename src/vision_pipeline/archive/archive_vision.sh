#!/bin/bash

# Prevent the script from continuing if a command fails
set -e

echo "🚀 Pre-flight sequence initiated for user: $USER"

# 1. Source the base ROS 2 installation
# Using absolute path here is fine as /opt/ros is standard
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS 2 Humble not found in /opt!"
    exit 1
fi

# 2. Source custom workspace using $HOME
WORKSPACE_PATH="$HOME/CUASC_2026/install/setup.bash"
if [ -f "$WORKSPACE_PATH" ]; then
    source "$WORKSPACE_PATH"
else
    echo "⚠️  Workspace not found at $WORKSPACE_PATH. Did you run colcon build?"
fi

# 3. Define the YOLO path once to stay DRY (Don't Repeat Yourself)
YOLO_DIR="$HOME/dev/VTOL-Project/src/yolo_models"
VENV_PATH="$YOLO_DIR/yolo_env"

# 4. Activate the virtual environment
if [ -d "$VENV_PATH" ]; then
    source "$VENV_PATH/bin/activate"
    
    # 5. Enforce PYTHONPATH dynamically
    # This finds the site-packages folder even if Python version changes slightly
    SITE_PKGS=$(find "$VENV_PATH/lib" -name "site-packages" -type d | head -n 1)
    export PYTHONPATH="$SITE_PKGS:$PYTHONPATH"
else
    echo "❌ YOLO virtual environment not found at $VENV_PATH"
fi

# 6. Reset Camera Daemon (Requires sudo)
echo "📷 Resetting Argus Daemon..."
sudo systemctl restart nvargus-daemon

echo "✅ Environment configured. Launching pipeline!"

# 7. Launch nodes
ros2 launch vision_pipeline vision_demo.launch.py
