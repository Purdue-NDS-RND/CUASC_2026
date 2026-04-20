#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: %s <release|grab>\n' "$(basename "$0")" >&2
    exit 1
}

if [[ $# -ne 1 ]]; then
    usage
fi

case "$1" in
    release|open)
        action_name="release"
        action_value="0.0"  # GRIPPER_ACTION_RELEASE
        ;;
    grab|close)
        action_name="grab"
        action_value="1.0"  # GRIPPER_ACTION_GRAB
        ;;
    *)
        usage
        ;;
esac

printf '[INFO] Sending gripper %s command\n' "${action_name}"
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
    "{broadcast: false, command: 211, confirmation: 0, param1: 1.0, param2: ${action_value}, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
