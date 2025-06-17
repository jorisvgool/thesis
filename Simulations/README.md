# Drone Simulation Package Index

This Catkin workspace contains multiple simulation packages, each with a specific drone configuration or algorithmic feature. The naming convention follows a short, lowercase format that helps identify the simulation setup quickly.

## Naming Convention

Each package name follows the structure:

```
sim-[#drones][flags]
```

Where:
- `#drones` = number of drones in the simulation
- Flags:
  - `g` = global/static/basic simulation
  - `k` = Kalman filter enabled
  - `s` = sine wave control
  - `a` = ArUco marker usage
  - `r` = rotation behavior (either rotating around an agent or agent self-rotation)

## Package Overview

| Simulation Description                            | Package Name |
|---------------------------------------------------|--------------|
| 1 drone (basic simulation)                        | `sim-1g`     |
| 2 drones (basic simulation)                       | `sim-2g`     |
| 3 drones (basic simulation)                       | `sim-3g`     |
| 1 drone with Kalman filter                        | `sim-1k`     |
| 1 drone with Kalman filter + sine wave control    | `sim-1ks`    |
| 1 drone with ArUco marker detection               | `sim-1a`     |
| 1 drone rotating around another agent             | `sim-1r`     |
| 1 drone rotating around agent + Kalman filter     | `sim-1rk`    |

## Usage

Each folder contains ROS launch files, scripts, and configuration for running the associated simulation. All packages depend on `rospy` and can be built and launched independently.

To build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

To launch a specific simulation (example):
```bash
roslaunch sim-1g drone.launch
```

---


