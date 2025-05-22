# PX4 Offboard Multi-Drone Simulation (ROS Noetic + MAVROS + Gazebo)

This repository provides a ROS Noetic-based simulation environment for PX4 offboard control using MAVROS and Gazebo Classic. It supports controlling multiple drones via Python nodes and is intended for research or educational use.

For a beginner-friendly introduction to PX4 offboard control with MAVROS and Python, refer to the official PX4 tutorial:  
https://docs.px4.io/main/en/ros/mavros_offboard_python.html

## System Requirements

- Ubuntu 20.04 (or WSL2 with Ubuntu 20.04)
- Internet connection for package installation and updates
- Basic familiarity with the Linux terminal

## 1. Clone the Workspace

```bash
git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
cd YOUR_REPO_NAME
```

Ensure that the repository includes a properly structured `catkin_ws` directory.

## 2. Install ROS Noetic

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release

# Add ROS package sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS Noetic Desktop Full
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# Source ROS environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Initialize rosdep
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
```

## 3. Install Catkin Tools and Build Essentials

```bash
sudo apt install -y python3-catkin-tools python3-osrf-pycommon python3-pip build-essential
```

## 4. Build the Catkin Workspace

```bash
cd catkin_ws
catkin build
```

Source the workspace after building:

```bash
source devel/setup.bash
```

To automatically source it in every terminal:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 5. Install PX4 Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default
```

## 6. Install MAVROS and GeographicLib

```bash
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

## 7. Set PX4 Environment Variables

Add the following lines to the end of your `~/.bashrc` file:

```bash
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
```

Then apply the changes:

```bash
source ~/.bashrc
```

## 8. Launch the Simulation

Once everything is built and sourced correctly, you can launch your simulation using:

```bash
roslaunch offboard_multi_py dual_drones.launch
```

Make sure to replace `offboard_multi_py` and `dual_drones.launch` with your actual package and launch file names if they differ.

---
