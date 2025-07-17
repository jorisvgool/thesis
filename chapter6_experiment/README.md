# Chapter 6 – Real-World Quadrotor Experiments

This folder contains real-world flight experiments conducted using a **Holybro X500 V2 quadrotor** equipped with a **Pixhawk 6C flight controller** running **PX4**. The onboard computer is a **Raspberry Pi 4 Model B** running ROS Noetic and MAVROS.

These experiments are meant to demonstrate and evaluate the real-time performance of the quadrotor control system developed during this thesis.

> **IMPORTANT:** Before proceeding, please follow the [INSTALL.md](../chapter5_simulation/INSTALL.md) guide to install **ROS**, **MAVROS**, and **PX4** on the Raspberry Pi 4 Model B. This setup is mandatory to run any of the experiments successfully.

---

## Repository Structure

```
chapter6_experiments/
├── catkin_ws/           # ROS workspace containing the experiment packages
│   ├── experiment1/     # Experiment 1 package (details in its own README)
│   ├── experiment2/     # Experiment 2 package
│   └── experiment3/     # Experiment 3 package
├── raw_data/            # Logged raw data from real flights
├── videos/              # Video recordings of the experiments
└── README.md            # This file
```

---

## Running Experiments on the Quadrotor

### Prerequisites

1. A **Raspberry Pi 4 Model B** with Ubuntu 20.04 installed.
2. Follow the [INSTALL.md](../chapter5_simulation/INSTALL.md) to install:
   - ROS Noetic
   - MAVROS (via binary or source)
   - PX4 Autopilot integration
3. Ensure a stable **SSH connection** to the Raspberry Pi for launching the experiments remotely.

### Experiment Packages

Each experiment is defined as a standard ROS package inside `catkin_ws/`.  
To run an experiment:

```bash
# On your local machine (connected via SSH):
ssh pi@<raspberry-pi-address>

# Then on the Raspberry Pi:
cd ~/catkin_ws
catkin build
source devel/setup.bash

# Launch the desired experiment
roslaunch experiment1 control.launch
```

---

## Safety Notice

**Real-world quadrotor testing comes with serious safety risks. Please follow these guidelines strictly:**

- Always fly in a **controlled, enclosed indoor laboratory** environment.
- **Secure the quadrotor with a safety rope** during early test flights to prevent injury or damage.
- **Recharge LiPo batteries only in a fireproof safety pouch** and under supervision.

---

## Hardware Setup

- **Quadrotor frame**: Holybro X500 V2  
- **Flight controller**: Pixhawk 6C  
- **Companion computer**: Raspberry Pi 4 Model B (4 GB recommended)  
- **Firmware**: PX4 Autopilot

Official setup instructions and specs for this platform can be found here:  
[Holybro X500 V2 with Pixhawk 6C (PX4 Docs)](https://docs.px4.io/main/en/frames_multicopter/holybro_x500V2_pixhawk5x.html)

---

That's it. Happy flying!
