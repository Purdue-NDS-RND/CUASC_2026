#!/usr/bin/env bash
set -euo pipefail

print_usage() {
    echo "Usage: $0 [--unlock] [--no-rviz] [--help]"
    echo "  default     Launch iris sim with the gimbal/camera mechanically locked"
    echo "  --unlock    Launch the original free-moving gimbal model"
    echo "  --no-rviz   Do not open RViz"
}

unlock=false
rviz=true

for arg in "$@"; do
    case "$arg" in
        --unlock|-u)
            unlock=true
            ;;
        --no-rviz)
            rviz=false
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            print_usage
            exit 1
            ;;
    esac
done

echo "Launching iris sim (unlock=${unlock}, rviz=${rviz})"
ros2 launch ardupilot_gz_bringup iris_runway.launch.py \
    unlock:="${unlock}" \
    rviz:="${rviz}"

sleep 10
python3 src/drone_utils/drone_utils/sdf_objects.py 
