#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: %s <open|close>\n' "$(basename "$0")" >&2
    printf 'Aliases: open=enable, close=disable\n' >&2
    exit 1
}

if [[ $# -ne 1 ]]; then
    usage
fi

case "$1" in
    open|enable|on)
        action_name="enable"
        action_value="1.0"
        ;;
    close|disable|off)
        action_name="disable"
        action_value="0.0"
        ;;
    *)
        usage
        ;;
esac

printf '[INFO] Sending sprayer %s command\n' "${action_name}"
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
    "{broadcast: false, command: 216, confirmation: 0, param1: ${action_value}, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
