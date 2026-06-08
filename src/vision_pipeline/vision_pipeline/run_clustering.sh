#!/bin/bash

set -euo pipefail

MISSION_DATA_DIR="$HOME/post_mission_processing/CUASC_Mission_Data"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="$SCRIPT_DIR/target_clustering.py"

EPS=2
MIN_SAMPLES=1

echo "🔎 Searching for latest flight logs in:"
echo "   $MISSION_DATA_DIR"

if [[ ! -d "$MISSION_DATA_DIR" ]]; then
    echo "❌ Mission data directory not found:"
    echo "   $MISSION_DATA_DIR"
    exit 1
fi

LATEST_FLIGHT=$(find "$MISSION_DATA_DIR" \
    -mindepth 1 \
    -maxdepth 1 \
    -type d \
    -name "Flight_*" \
    -printf "%T@ %p\n" \
    | sort -n \
    | tail -1 \
    | cut -d' ' -f2-)

if [[ -z "$LATEST_FLIGHT" ]]; then
    echo "❌ No Flight_* directories found."
    exit 1
fi

echo
echo "🎯 Latest flight:"
echo "   $(basename "$LATEST_FLIGHT")"
echo
echo "⚙️ DBSCAN Parameters"
echo "   eps         = ${EPS} m"
echo "   min_samples = ${MIN_SAMPLES}"
echo

python3 "$SCRIPT_PATH" \
    --flight_dir "$LATEST_FLIGHT" \
    --eps "$EPS" \
    --min_samples "$MIN_SAMPLES"

echo
echo "✅ Clustering complete."
