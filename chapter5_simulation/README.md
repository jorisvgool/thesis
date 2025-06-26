# Drone-Sim Repository Overview

This repository is organised into three main items that work together to run all ROS + Gazebo simulations.

```
repo-root/
├── catkin_ws/        # ROS workspace with every simulation package
│   └── README.md     # Detailed logic behind the package naming & flags
├── gazebo/           # Worlds, models, media - copied to $HOME on first use
└── init_gazebo.sh    # One-shot environment initialiser for each new shell
```

---

## 1 `catkin_ws/`

*Standard ROS 1 (Catkin) workspace.*

```bash
cd catkin_ws
catkin_make          # build once after cloning / pulling
source devel/setup.bash
```

All simulation packages live in `catkin_ws/src/`.  
See `catkin_ws/README.md` for the naming convention and per-package details.

---

## 2 `gazebo/`

Contains **all assets Gazebo needs**:

```
gazebo/
├── models/    # custom or edited models
├── worlds/    # .world descriptions
└── media/     # textures, meshes, sounds, etc.
```

### One-time setup

Copy (or symlink) this folder into your home directory so Gazebo can find it automatically:

```bash
cp -r gazebo ~/gazebo
# or
ln -s "$(pwd)/gazebo" ~/gazebo
```

*(Gazebo searches `~/gazebo/models` by default.)*

---

## 3 `init_gazebo.sh`

A helper script that **must be sourced in every terminal session** before launching a simulation.  
It exports the required environment variables so Gazebo can locate the models and worlds in `~/gazebo`.

```bash
# From the repo root, every new shell:
source init_gazebo.sh
```

Typical variables set:

```bash
export GAZEBO_MODEL_PATH="$HOME/gazebo/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="$HOME/gazebo:${GAZEBO_RESOURCE_PATH}"
```

Feel free to open the script and adjust paths if your layout differs.

---

## Quick-start Checklist

```bash
# 1. Clone repo
git clone <your-repo-url>
cd drone-sim-repo

# 2. Copy or symlink Gazebo assets
cp -r gazebo ~/gazebo      # or ln -s …

# 3. Build ROS workspace (once per pull/clone)
cd catkin_ws
catkin_make
source devel/setup.bash

# 4. For every new terminal session
cd /path/to/drone-sim-repo
source init_gazebo.sh
source catkin_ws/devel/setup.bash

# 5. Launch a simulation
roslaunch sim-1gb drone.launch
```

That’s it—happy simulating!
