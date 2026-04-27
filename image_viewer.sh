#!/usr/bin/env bash

set -eo pipefail

# Prevent COLCON_TRACE unbound variable error from setup.bash
export COLCON_TRACE=${COLCON_TRACE:-0}

# Source the built workspace so the new debug viewer runs correctly
source ./install/setup.bash || {
    echo "Error: Missing install/setup.bash. Remember to run 'colcon build' first." >&2
    exit 1
}

transport="${1:-raw}"

if [ "$transport" = "compressed" ]; then
    echo "Launching Dashboard Viewer with COMPRESSED topic inputs..."
    exec ros2 run drone_target_cv debug_viewer --ros-args -p compressed_input:=true
elif [ "$transport" = "raw" ]; then
    echo "Launching Dashboard Viewer with RAW topic inputs..."
    exec ros2 run drone_target_cv debug_viewer --ros-args -p compressed_input:=false
elif [ "$transport" = "old" ]; then
    echo "Launching OLD Image Viewer (rqt_image_view)..."
    exec ros2 run rqt_image_view rqt_image_view
else
    echo "Usage: $0 [raw|compressed|old]" >&2
    exit 1
fi

