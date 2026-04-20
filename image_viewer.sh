#!/usr/bin/env bash

set -euo pipefail

transport="${1:-raw}"

case "$transport" in
  raw)
    exec ros2 run rqt_image_view rqt_image_view
    ;;
  compressed)
    exec ros2 run rqt_image_view rqt_image_view --ros-args -p image_transport:=compressed
    ;;
  *)
    echo "Usage: $0 [raw|compressed]" >&2
    exit 1
    ;;
esac
