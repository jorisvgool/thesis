#!/bin/bash

# ----------------------------------------------
# 1. Source ROS Noetic
# ----------------------------------------------
source /opt/ros/noetic/setup.bash

# ----------------------------------------------
# 2. Source your Catkin workspace
# ----------------------------------------------
source ~/catkin_ws/devel/setup.bash

# ----------------------------------------------
# 3. Source PX4 Gazebo Classic environment
# ----------------------------------------------
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
        ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default

# ----------------------------------------------
# 4. Source Gazebo 11 setup (shader libs, etc.)
# ----------------------------------------------
source /usr/share/gazebo-11/setup.bash

# ----------------------------------------------
# 5. Update paths for custom models/materials/worlds
# ----------------------------------------------
export GAZEBO_MODEL_PATH=$HOME/gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$HOME/gazebo:$GAZEBO_RESOURCE_PATH
export GAZEBO_WORLD_PATH=$HOME/gazebo/worlds:$GAZEBO_WORLD_PATH

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# ----------------------------------------------
# 6. Networking config
# ----------------------------------------------
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
