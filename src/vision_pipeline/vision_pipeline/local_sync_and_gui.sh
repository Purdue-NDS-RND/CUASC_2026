#!/bin/bash
set -euo pipefail

# ----------------------------------------------------------
# Drone Connection Configuration
# ----------------------------------------------------------
DRONE_USER="nds01"
DRONE_IP="192.168.0.149"
LOCAL_ROOT="$HOME/post_mission_processing"
LOCAL_RAYCAST_DIR="$LOCAL_ROOT/raycast_sessions"
LOCAL_MISSION_DIR="$LOCAL_ROOT/CUASC_Mission_Data"

mkdir -p "$LOCAL_RAYCAST_DIR"
mkdir -p "$LOCAL_MISSION_DIR"

echo "📡 Connecting to $DRONE_USER@$DRONE_IP"

# ----------------------------------------------------------
# Find newest raycast session on drone
# ----------------------------------------------------------
LATEST_SESSION=$(ssh "$DRONE_USER@$DRONE_IP" \
  'find ~/raycast_sessions \
    -mindepth 1 \
    -maxdepth 1 \
    -type d \
    -name "Raycast_Session_*" \
    -printf "%T@ %p\n" | \
    sort -n | \
    tail -1 | \
    cut -d" " -f2-')

# ----------------------------------------------------------
# Find newest mission flight on drone
# ----------------------------------------------------------
LATEST_FLIGHT=$(ssh "$DRONE_USER@$DRONE_IP" \
  'find ~/CUASC_Mission_Data \
    -mindepth 1 \
    -maxdepth 1 \
    -type d \
    -name "Flight_*" \
    -printf "%T@ %p\n" | \
    sort -n | \
    tail -1 | \
    cut -d" " -f2-')

echo
echo "🎯 Latest Raycast Session:"
echo "   $LATEST_SESSION"
echo
echo "🎯 Latest Mission Flight:"
echo "   $LATEST_FLIGHT"


echo "🔌 Testing SSH connection..."

if ! ssh -o ConnectTimeout=5 "$DRONE_USER@$DRONE_IP" "echo connected" >/dev/null 2>&1; then
    echo "❌ Unable to connect to drone."
    exit 1
fi

echo "✅ Drone reachable."

# ----------------------------------------------------------
# Sync newest raycast session only
# ----------------------------------------------------------
echo
echo "🔄 Syncing newest raycast session..."
rsync -avz --partial --progress \
  "$DRONE_USER@$DRONE_IP:$LATEST_SESSION/" \
  "$LOCAL_RAYCAST_DIR/$(basename "$LATEST_SESSION")/"

# ----------------------------------------------------------
# Sync newest mission flight only
# ----------------------------------------------------------
echo
echo "🔄 Syncing newest mission flight..."
rsync -avz --progress \
  "$DRONE_USER@$DRONE_IP:$LATEST_FLIGHT/" \
  "$LOCAL_MISSION_DIR/$(basename "$LATEST_FLIGHT")/"

echo
echo "✅ Sync complete."
echo
echo "Local data stored at:"
echo "   $LOCAL_ROOT"

# ----------------------------------------------------------
# Launch GUI
# ----------------------------------------------------------
GUI_SCRIPT="raycast_gui.py"
if [ -f "$GUI_SCRIPT" ]; then
  python3 "$GUI_SCRIPT"
elif [ -f "./vision_pipeline/raycast_gui.py" ]; then
  python3 "./vision_pipeline/raycast_gui.py"
else
  echo "❌ Could not find raycast_gui.py"
  exit 1
fi
