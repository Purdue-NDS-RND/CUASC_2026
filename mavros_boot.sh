#!/usr/bin/env bash

# Usage:
#   ./mavros_boot.sh sim   # Launch MAVROS for simulation (UDP)
#   ./mavros_boot.sh device  # Launch MAVROS for real device (/dev/ttyACM0)

set -e

print_usage() {
	echo "Usage: $0 <sim|device>"
	echo "  sim  - ros2 launch mavros apm.launch fcu_url:=udp://:14550@"
	echo "  device - ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200"
}

if [[ $# -ne 1 ]]; then
	echo "Error: exactly one argument is required."
	print_usage
	exit 1
fi

mode="$1"

case "$mode" in
	sim)
		ros2 launch mavros apm.launch fcu_url:=udp://:14550@
		;;
	device)
		ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
		;;
	*)
		echo "Error: invalid argument '$mode'."
		print_usage
		exit 1
		;;
esac
