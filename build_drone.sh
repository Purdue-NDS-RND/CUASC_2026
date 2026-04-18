#!/bin/bash
# get all packages with the name starting with drone_* and vision_pipeline

_build_drone_sourced=0
if [ "${BASH_SOURCE[0]}" != "$0" ]; then
	_build_drone_sourced=1
fi

build_drone_finish() {
	local status="${1:-0}"
	if [ "$_build_drone_sourced" -eq 1 ]; then
		return "$status"
	fi
	exit "$status"
}

sanitize_colon_path_var() {
	local var_name="$1"
	local original_value="${!var_name:-}"
	local sanitized=""
	local entry=""

	[ -n "$original_value" ] || return 0

	IFS=':' read -r -a _path_entries <<< "$original_value"
	for entry in "${_path_entries[@]}"; do
		[ -n "$entry" ] || continue
		[ -e "$entry" ] || continue
		if [ -n "$sanitized" ]; then
			sanitized="${sanitized}:$entry"
		else
			sanitized="$entry"
		fi
	done

	export "$var_name=$sanitized"
}

build_drone_main() {
	local clean=0
	local packages=""
	local script_name="${BASH_SOURCE[0]}"
	local arg=""

	for arg in "$@"; do
		case "$arg" in
			--clean|-c)
				clean=1
				;;
			--help|-h)
				echo "Usage: $script_name [--clean]"
				echo "  --clean, -c   Remove build/install/log artifacts for drone_* and vision_pipeline before rebuilding"
				return 0
				;;
			*)
				echo "Unknown argument: $arg"
				echo "Usage: $script_name [--clean]"
				return 1
				;;
		esac
	done

	packages=$(find src -maxdepth 1 -mindepth 1 -type d -printf '%f\n' | grep -E '^(drone_.*|vision_pipeline)$' | sort)

	if [ -z "$packages" ]; then
		echo "No packages matching drone_* or vision_pipeline found under src/."
		return 1
	fi

	echo -e "Building packages:\n$packages\n"

	# A previously sourced workspace can leave deleted package prefixes in the
	# environment, which makes colcon warn before the build starts.
	sanitize_colon_path_var AMENT_PREFIX_PATH
	sanitize_colon_path_var CMAKE_PREFIX_PATH
	sanitize_colon_path_var PYTHONPATH

	if [ "$clean" -eq 1 ]; then
		echo "Cleaning drone_* and vision_pipeline build/install/log artifacts..."
		rm -rf \
			build/drone_* install/drone_* log/latest_build/drone_* log/latest_test/drone_* \
			build/vision_pipeline install/vision_pipeline log/latest_build/vision_pipeline log/latest_test/vision_pipeline
	fi

	colcon build --packages-select $packages || return $?
	source install/setup.bash || return $?
	return 0
}

build_drone_main "$@"
build_drone_finish $?
