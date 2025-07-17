# ROS + MAVROS Installation Guide (Ubuntu 20.04 / ROS Noetic)

This guide walks you through setting up your system for quadrotor simulations using ROS Noetic, MAVROS, and PX4.

---

## 1. Set Up Your System

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install curl -y
sudo apt install python3-catkin-tools -y
```

---

## 2. Install ROS Noetic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
```

---

## 3. Initialize ROS Environment

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
```

---

## 4. Clone PX4 Autopilot (Optional)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
```

---

## 5. Install MAVROS (Binary Method)

```bash
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs -y
```

### Install GeographicLib datasets

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

---

## 6. Install MAVROS (Source Method – Optional)

```bash
# Create and initialize Catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

# Install required tools
sudo apt install python3-wstool python3-rosinstall-generator -y

# Generate .rosinstall file
rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

# Download source packages
wstool init src
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4

# Install dependencies
rosdep install --from-paths src --ignore-src -y

# Install GeographicLib datasets (again if needed)
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Build the workspace
catkin build
source devel/setup.bash
```

---

## 7. Verify Installation

```bash
rospack find mavros
```

---

## Need Help?

If you encounter issues (e.g. PX4 build errors, MAVROS dependency problems, or arming checks), please refer to the official documentation:

- PX4 + MAVROS setup: https://docs.px4.io/main/en/ros/mavros_installation.html  
- MAVROS Offboard mode: https://docs.px4.io/main/en/ros/mavros_offboard_python.html
