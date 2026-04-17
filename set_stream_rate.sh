#!/usr/bin/env bash
set -euo pipefail

rate="${1:-20}"

ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: ${rate}, on_off: true}"