#!/bin/bash
set -x
echo "=== SCRIPT STARTED ==="
sleep 1

# Resolve the root directory of this script (inside your thesis folder)
THESIS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ----------------------------------------------
# 1. Source ROS Noetic
# ----------------------------------------------
source /opt/ros/noetic/setup.bash

# ----------------------------------------------
# 2. Find and source the Catkin workspace
# ----------------------------------------------
CATKIN_WS=$(find "$THESIS_ROOT" -type d -name devel -exec test -f "{}/setup.bash" \; -print -quit)
if [ -n "$CATKIN_WS" ]; then
    source "$CATKIN_WS/setup.bash"
    echo "[✓] Sourced catkin workspace: $CATKIN_WS"
    printf " [✓] Sourced workspace: %s \n" "$CATKIN_WS"
else
    echo "[!] Could not find catkin devel/setup.bash. Did you run catkin_make?"
fi

# ----------------------------------------------
# 3. Find and source PX4 Gazebo Classic environment
# ----------------------------------------------
PX4_DIR=$(find "$THESIS_ROOT" -type d -name PX4-Autopilot -print -quit)
if [ -n "$PX4_DIR" ]; then
    source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
        "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
    echo "[✓] Sourced PX4 setup from: $PX4_DIR"
else
    echo "[!] PX4-Autopilot folder not found inside thesis directory."
fi

# ----------------------------------------------
# 4. Source Gazebo 11 setup
# ----------------------------------------------
source /usr/share/gazebo-11/setup.bash

# ----------------------------------------------
# 5. Update paths for Gazebo models, materials, worlds
# ----------------------------------------------
export GAZEBO_MODEL_PATH="$THESIS_ROOT/gazebo/models:$GAZEBO_MODEL_PATH"
export GAZEBO_RESOURCE_PATH="$THESIS_ROOT/gazebo:$GAZEBO_RESOURCE_PATH"
export GAZEBO_WORLD_PATH="$THESIS_ROOT/gazebo/worlds:$GAZEBO_WORLD_PATH"

export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

# ----------------------------------------------
# 6. Networking config
# ----------------------------------------------
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
