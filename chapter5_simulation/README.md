# Quadrotor Simulation Repository Overview

This repository provides all necessary components to run quadrotor simulations using ROS and Gazebo. It includes multiple simulation project folders organized by chapter and scenario.

> **First time setting up?** Start with the [INSTALL.md](INSTALL.md) guide to install ROS, MAVROS, PX4, and all required dependencies before using the simulations.

---

## Repository Structure

```
thesis/
├── chapter5_simulation/
│   ├── INSTALL.md            # Step-by-step installation instructions (ROS, MAVROS, PX4, etc.)
│   ├── catkin_ws/            # ROS packages for different simulations
│   │   ├── sim-1gb/          # Example: one simulation project
│   │   ├── sim-2gb/
│   │   └── ...
│   ├── gazebo/               # Gazebo models, worlds, and media
│   ├── init_gazebo.sh        # Gazebo environment setup script
│   ├── raw_data/             # Optional: raw experiment outputs
│   └── README.md             # This file
```

---

## 1. `catkin_ws/`

This folder contains all simulation project folders (ROS packages) under `chapter5_simulation/catkin_ws/`.

To use a simulation, **copy only the desired package folder** (e.g. `sim-1gs`) into your own ROS workspace under `~/catkin_ws/src/`.

Each simulation package includes its own `launch/`, `scripts/`, and configuration files.  
See the `README.md` inside each package for details.

---

## 2. `gazebo/`

Contains Gazebo assets needed for the simulations:

```
gazebo/
├── models/    # Custom or modified 3D models
├── worlds/    # .world files for different environments
└── media/     # Textures, meshes, etc.
```

### One-time setup

Copy or symlink this folder into your home directory so Gazebo can find it:

```bash
cp -r gazebo ~/gazebo
# or
ln -s "$(pwd)/gazebo" ~/gazebo
```

Gazebo automatically looks for models in `~/gazebo/models`.

---

## 3. `init_gazebo.sh`

This script sets up Gazebo environment variables and must be sourced at the start of every new terminal session.

Copy it to your home directory:

```bash
cp init_gazebo.sh ~/
```

Then, at the beginning of each terminal session:

```bash
source ~/init_gazebo.sh
```

You can optionally add this line to your `~/.bashrc` to automate it:

```bash
echo 'source ~/init_gazebo.sh' >> ~/.bashrc
```

---

## Quick-start Checklist

```bash
# 1. Clone the repository (not recommended)
git clone https://github.com/jorisvgool/thesis.git thesis_repo
cd thesis_repo/chapter5_simulation

# 2. Copy Gazebo assets and init script to your home directory
cp -r gazebo ~/gazebo
cp init_gazebo.sh ~/

# 3. Copy a specific simulation package to your own ROS workspace
cp -r catkin_ws/sim-1gs ~/catkin_ws/src/

# 4. Build your workspace (run from ~/catkin_ws)
cd ~/catkin_ws
catkin build

# 5. In every new terminal session
source ~/init_gazebo.sh
source devel/setup.bash

# Automate this in ~/.bashrc (optional)
echo 'source ~/init_gazebo.sh' >> ~/.bashrc
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# 6. Launch a simulation
roslaunch sim-1gs drone.launch
```

---

That’s it. Happy simulating!
