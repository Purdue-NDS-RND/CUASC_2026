#!/bin/bash

# Prevent script from continuing if any compilation or sourcing step fails
set -e

echo "🚀 CUASCVTOL Pre-Flight Build & Launch Sequence"
echo "================================================="

# 1. Source the base ROS 2 Humble installation
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ Base ROS 2 Humble Sourced"
else
    echo "❌ ROS 2 Humble not found in /opt! Exiting."
    exit 1
fi

# 2. Navigate to workspace root and rebuild the vision pipeline package
WORKSPACE_DIR="$HOME/CUASC_2026"
if [ -d "$WORKSPACE_DIR" ]; then
    echo "🔨 Building vision_pipeline package..."
    cd "$WORKSPACE_DIR"
    # Rebuild only the vision_pipeline package to save valuable pre-flight time
    colcon build --packages-select vision_pipeline

    # Source local workspace setup
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
        echo "✅ Custom VTOL Workspace Sourced Successfully"
    else
        echo "❌ install/setup.bash not found after colcon build!"
        exit 1
    fi
else
    echo "❌ Workspace directory not found at $WORKSPACE_DIR!"
    exit 1
fi

# 3. Setup YOLO Virtual Environment and Python Paths
YOLO_DIR="$HOME/dev/VTOL-Project/src/yolo_models"
VENV_PATH="$YOLO_DIR/yolo_env"

if [ -d "$VENV_PATH" ]; then
    echo "🐍 Activating YOLO virtual environment..."
    source "$VENV_PATH/bin/activate"

    # Enforce correct PYTHONPATH dynamically matching python site-packages
    SITE_PKGS=$(find "$VENV_PATH/lib" -name "site-packages" -type d | head -n 1)
    if [ -n "$SITE_PKGS" ]; then
        export PYTHONPATH="$SITE_PKGS:$PYTHONPATH"
        echo "✅ PYTHONPATH Exported: $SITE_PKGS"
    else
        echo "⚠️  Could not resolve site-packages directory inside virtual environment!"
    fi
else
    echo "❌ YOLO virtual environment not found at $VENV_PATH!"
    exit 1
fi

# 4. Reset Camera Daemon (Crucial to clear Jetson camera driver lockups)
echo "📷 Restarting nvargus-daemon (Requires sudo privileges)..."
sudo systemctl restart nvargus-daemon
sleep 1.0 # Give the system daemon a moment to fully initialize

echo "🔥 Pipeline Environment Ready! Slicing and Inference Live Flight Node Starting Now."
echo "================================================================================"

# 5. Launch the unified yolo mission logger system node
# Note: Ensure your setup.py has 'yolo_mission_node = vision_pipeline.yolo_mission_node:main'
ros2 run vision_pipeline yolo_mission_node
