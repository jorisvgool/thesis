# Experiment Package Overview

This workspace contains the ROS packages used to run the real-world flight experiments on the Holybro X500 V2 quadrotor, as described in Chapter 6 of the thesis.

Each package corresponds to a different control strategy or sensing capability. You can build all packages at once and selectively launch them depending on the desired test.

---

## Package List

```
catkin_ws/
├── experiment1/     # Gradient-based baseline controller
├── experiment2/     # Kalman-based predictive controller with sinusoidal motion
└── experiment3/     # ArUco-based control in a virtual framework
```

---

## experiment1 — Baseline Gradient-Based Control

This experiment implements a basic gradient-based formation controller.

### Features:
- Designed for **formation flight** with fixed or moving anchor points.
- No state estimation (i.e., no Kalman filter).

### Usage:
In the `control_node`, there is a **parameter** to switch between:
- **Static anchors**
- **Moving anchors** (disturbance rejection test)

---

## experiment2 — Kalman-Based Predictive Control with Sinusoidal Input

This experiment extends the baseline controller by adding a **Kalman filter** for state prediction and introducing a **sinusoidal reference signal** for the quadrotor.

### Features:
- Kalman filter for predictive state estimation.
- Sinusoidal moving anchors.
- Switch between modes in code.

### Usage:
In the `control_node`, you can toggle:
- **Kalman predictive control ON**
- **Kalman predictive control OFF** (reverts to simpler control logic)

---

## experiment3 — ArUco Marker Detection in a Virtual Control Framework

This experiment uses **vision-based ArUco marker detection** to enable control in a **local frame**.

### Features:
- Local frame control (quadrotor body frame).
- Visual marker tracking with camera input.
- Virtual control logic.

---

## Building the Workspace

After copying these packages into `~/catkin_ws/src/` on your Raspberry Pi:

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

---

## Launching an Experiment

Use the appropriate launch file within each package:

```bash
roslaunch experiment1 run.launch      # Gradient-based control
roslaunch experiment2 run.launch      # Kalman + sinusoidal
roslaunch experiment3 run.launch      # ArUco-based virtual control
```

Refer to each package's `launch/` and `scripts/` folders for adjustable parameters and node configurations.

---

For full system installation instructions (ROS, MAVROS, PX4), see the [INSTALL.md](../INSTALL.md) file.
