# Quadrotor Simulation Package Index

This Catkin workspace contains multiple simulation packages, each representing a specific quadrotor setup or algorithmic feature.  
Package names are intentionally short and lowercase, with hyphens, so their purpose is easy to read at a glance.

---

## Naming Convention

```
sim-[#quadrotors][flags]
```

- **`#quadrotors`** – exact number of quadrotors in the scenario  
- **Flags** – one or more lowercase letters, combined in any order:

| Flag | Meaning                                                                                                |
|------|--------------------------------------------------------------------------------------------------------|
| **g** | **G**lobal — simulation uses a world-fixed (global) reference frame                                   |
| **l** | **L**ocal — simulation uses the quadrotor's body-fixed (local) reference frame                        |
| **b** | **B**ase — basic gradient-based formation control                                                     |
| **r** | **R**otational — distributed, centroid-based rotation of the entire formation                         |
| **c** | **C**ircling — distributed, agent-based rotation of the entire formation                              |
| **t** | **T**ranslational — distributed, rigid translational motion of the formation                          |
| **k** | **K**alman — predictive control using a Kalman filter                                                 |
| **s** | **S**inusoidal — sinusoidal motion with Kalman predictive control                                     |
| **a** | **A**rUco — ArUco marker detection with virtual control framework                                     |


*Combine letters to stack behaviours (e.g. `gk` = global + Kalman filter).*

---

## Package Overview

| Simulation Description                                                       | Package Name |
|------------------------------------------------------------------------------|--------------|
| 1 quadrotor — global frame, Kalman filter + sinusoidal motion                | `sim-1gs`    |
| 1 quadrotor — global frame with Kalman filter                                | `sim-1gk`    |
| 1 quadrotor — global, base behaviour                                         | `sim-1gb`    |
| 2 quadrotors — global, base behaviour                                        | `sim-2gb`    |
| 3 quadrotors — global, base behaviour                                        | `sim-3gb`    |
| 3 quadrotors — global, rotational motion around centroid                     | `sim-3gr`    |
| 3 quadrotors — global, agent-based circling                                  | `sim-3gc`    |
| 3 quadrotors — global, translational formation motion                        | `sim-3gt`    |
| 2 quadrotors — local-frame operation with ArUco detection                    | `sim-2la`    |
| 1 quadrotor — local-frame operation with ArUco detection                     | `sim-1la`    |


---

## Usage

Each simulation package is self-contained with its own launch files, scripts, and configuration.

```bash
# 1. Copy the desired simulation package to your catkin workspace
cp -r chapter5_simulation/catkin_ws/sim-1gb ~/catkin_ws/src/

# 2. Rebuild your workspace
cd ~/catkin_ws
catkin build
source devel/setup.bash

# 3. Launch the simulation
source ~/init_gazebo.sh
roslaunch sim-1gb drone.launch

# Replace 'sim-1gb' with any other package name as needed
```
