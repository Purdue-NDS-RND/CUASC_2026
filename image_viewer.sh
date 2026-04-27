#!/usr/bin/env bash

set -eo pipefail

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
else
    echo "Usage: $0 [raw|compressed]" >&2
    exit 1
fi

