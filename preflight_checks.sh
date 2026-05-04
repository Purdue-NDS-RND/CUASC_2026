#!/usr/bin/env bash
set -euo pipefail

stream_rate="${1:-20}"
extended_state_rate="${2:-2.0}"
topic_timeout_s="${3:-5}"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

check_config_tree() {
    local package_name="$1"
    local src_dir="$2"
    local install_dir="$3"
    local failed=0
    local rel_path=""
    local src_file=""
    local install_file=""

    printf '[INFO] 🔎 Checking installed configs for %s\n' "${package_name}"

    if [[ ! -d "${src_dir}" ]]; then
        printf '[FAIL] ❌ Source config dir missing: %s\n' "${src_dir}" >&2
        return 1
    fi

    if [[ ! -d "${install_dir}" ]]; then
        printf '[FAIL] ❌ Installed config dir missing: %s\n' "${install_dir}" >&2
        printf '[INFO] 🔧 Rebuild this package before flying: colcon build --packages-select %s\n' "${package_name}" >&2
        return 1
    fi

    while IFS= read -r rel_path; do
        [[ -n "${rel_path}" ]] || continue
        src_file="${src_dir}/${rel_path}"
        install_file="${install_dir}/${rel_path}"

        if [[ ! -f "${install_file}" ]]; then
            printf '[FAIL] ❌ Missing installed config: %s\n' "${install_file}" >&2
            failed=1
            continue
        fi

        if ! cmp -s "${src_file}" "${install_file}"; then
            printf '[FAIL] ❌ Installed config differs from source: %s\n' "${rel_path}" >&2
            diff -u \
                --label "src/${package_name}/config/${rel_path}" \
                --label "install/${package_name}/config/${rel_path}" \
                "${src_file}" "${install_file}" >&2 || true
            failed=1
        fi
    done < <(cd "${src_dir}" && find . -type f -name '*.yaml' -printf '%P\n' | sort)

    while IFS= read -r rel_path; do
        [[ -n "${rel_path}" ]] || continue
        src_file="${src_dir}/${rel_path}"
        install_file="${install_dir}/${rel_path}"

        if [[ ! -f "${src_file}" ]]; then
            printf '[FAIL] ❌ Extra installed config not present in source: %s\n' "${install_file}" >&2
            failed=1
        fi
    done < <(cd "${install_dir}" && find . -type f -name '*.yaml' -printf '%P\n' | sort)

    if [[ "${failed}" -ne 0 ]]; then
        printf '[FAIL] ❌ %s installed configs do not match source configs\n' "${package_name}" >&2
        printf '[INFO] 🔧 Run: source build_drone.sh --clean\n' >&2
        return 1
    fi

    printf '[PASS] ✅ %s installed configs match source configs\n' "${package_name}"
}

check_config_tree \
    "drone_mission_demo" \
    "${script_dir}/src/drone_mission_demo/config" \
    "${script_dir}/install/drone_mission_demo/share/drone_mission_demo/config"

check_config_tree \
    "drone_target_cv" \
    "${script_dir}/src/drone_target_cv/config" \
    "${script_dir}/install/drone_target_cv/share/drone_target_cv/config"

printf '[INFO] 📡 Running MAVROS stream checks: stream_rate=%s Hz, extended_state_rate=%s Hz, timeout=%ss\n' \
    "${stream_rate}" "${extended_state_rate}" "${topic_timeout_s}"
"${script_dir}/check_mavros_streams.sh" "${stream_rate}" "${extended_state_rate}" "${topic_timeout_s}"

printf '[PASS] ✅ Preflight checks passed\n'
