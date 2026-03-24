#!/bin/bash
# get all packages with the name starting with drone_*

set -e

clean=0
for arg in "$@"; do
	case "$arg" in
		--clean|-c)
			clean=1
			;;
		--help|-h)
			echo "Usage: $0 [--clean]"
			echo "  --clean, -c   Remove build/install/log artifacts for drone_* packages before rebuilding"
			exit 0
			;;
		*)
			echo "Unknown argument: $arg"
			echo "Usage: $0 [--clean]"
			exit 1
			;;
	esac
done

packages=$(find src -maxdepth 1 -mindepth 1 -type d -printf '%f\n' | grep '^drone_' | sort)

if [ -z "$packages" ]; then
	echo "No packages matching drone_* found under src/."
	exit 1
fi

echo -e "Building packages:\n$packages\n"

if [ "$clean" -eq 1 ]; then
	echo "Cleaning drone_* build/install/log artifacts..."
	rm -rf build/drone_* install/drone_* log/latest_build/drone_* log/latest_test/drone_*
fi

colcon build --packages-select $packages


source install/setup.bash