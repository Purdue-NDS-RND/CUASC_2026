#!/bin/bash

# =================================================-------------------------
# CUASC Ground Station Data Sync and Raycast GUI Launcher
# Run this on your local computer to transfer and review flight metrics.
# =================================================-------------------------

# Prevent the script from continuing if a command fails
set -e

# --- CONFIGURATION (Adjust for your drone IP and username) ---
DRONE_USER="nds01"
DRONE_IP="192.168.0.149" # Update this to your Jetson's active network IP address

REMOTE_SESSIONS_DIR="/home/$DRONE_USER/raycast_sessions/"
LOCAL_SESSIONS_DIR="$HOME/raycast_sessions"

# Create local storage folders if not present
mkdir -p "$LOCAL_SESSIONS_DIR"

echo "📡 Establishing connection to drone ($DRONE_USER@$DRONE_IP)..."

# Test physical network ping connection
if ! ping -c 1 -W 2 "$DRONE_IP" > /dev/null 2>&1; then
    echo "⚠️  Warning: Cannot reach $DRONE_IP over ICMP ping. Check your network or routing bridge!"
fi

echo "🔄 Synchronizing raycast session datasets..."
echo "--------------------------------------------------------"

# Use compressed rsync instead of standard scp.
# Rsync uses delta-transfer algorithms to only copy modified frames and avoids
# re-transferring previously loaded images during successive run syncs.
rsync -avz --progress --exclude="*.tmp" \
    "$DRONE_USER@$DRONE_IP:$REMOTE_SESSIONS_DIR" \
    "$LOCAL_SESSIONS_DIR/"

echo "--------------------------------------------------------"
echo "✅ Synchronization complete! Files stored locally in: $LOCAL_SESSIONS_DIR"

# Launch Interactive Ground Station GUI
GUI_SCRIPT="raycast_gui.py"
if [ -f "$GUI_SCRIPT" ]; then
    echo "🖥️  Launching Interactive Target Geolocation Desk..."
    python3 "$GUI_SCRIPT"
else
    # Search locally within subdirectory setup
    GUI_FALLBACK="./vision_pipeline/raycast_gui.py"
    if [ -f "$GUI_FALLBACK" ]; then
        echo "🖥️  Launching Interactive Target Geolocation Desk (Fallback Path)..."
        python3 "$GUI_FALLBACK"
    else
        echo "❌ Error: Could not locate 'raycast_gui.py' inside local ground station workspace directory."
        exit 1
    fi
fi
