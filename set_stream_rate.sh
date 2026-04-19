#!/usr/bin/env bash
set -euo pipefail

rate="${1:-20}"
extended_state_rate="${2:-2.0}"

ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: ${rate}, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 245, message_rate: ${extended_state_rate}}"
