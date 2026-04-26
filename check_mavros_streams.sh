#!/usr/bin/env bash
set -euo pipefail

stream_rate="${1:-20}"
extended_state_rate="${2:-2.0}"
topic_timeout_s="${3:-5}"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

required_topics=(
    "/mavros/imu/data"
    "/mavros/local_position/pose"
    "/mavros/extended_state"
)

check_topic_once() {
    local topic="$1"
    local output

    printf '[INFO] Waiting for %s\n' "${topic}"
    if ! output="$(timeout "${topic_timeout_s}s" ros2 topic echo --once "${topic}" 2>&1)"; then
        printf '[FAIL] ❌ No message received on %s within %ss\n' "${topic}" "${topic_timeout_s}" >&2
        if [[ -n "${output}" ]]; then
            printf '%s\n' "${output}" >&2
        fi
        return 1
    fi

    printf '[PASS] ✅ %s published\n' "${topic}"

    if [[ "${topic}" == "/mavros/extended_state" ]]; then
        local landed_state
        landed_state="$(printf '%s\n' "${output}" | awk '/landed_state:/ {print $2; exit}')"

        if [[ -z "${landed_state}" ]]; then
            printf '[FAIL] ❌ Could not parse landed_state from /mavros/extended_state\n' >&2
            printf '%s\n' "${output}" >&2
            return 1
        fi

        if [[ "${landed_state}" == "0" ]]; then
            printf '[FAIL] ❌ /mavros/extended_state is still UNDEFINED (landed_state=0)\n' >&2
            printf '%s\n' "${output}" >&2
            return 1
        fi

        printf '[PASS] ✅ /mavros/extended_state landed_state=%s\n' "${landed_state}"
    fi
}

printf '[INFO] Requesting MAVROS streams: stream_rate=%s Hz, extended_state_rate=%s Hz\n' \
    "${stream_rate}" "${extended_state_rate}"
"${script_dir}/set_stream_rate.sh" "${stream_rate}" "${extended_state_rate}"

for topic in "${required_topics[@]}"; do
    check_topic_once "${topic}"
done

printf '[PASS] ✅ All required MAVROS streams are live\n'
