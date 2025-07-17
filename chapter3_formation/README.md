# Drone Simulation Package Index

This Catkin workspace contains multiple simulation packages, each representing a particular drone setup or algorithmic feature.  
Package names stay short and lowercase so you can read the intent at a glance—even though they keep their hyphens.

---

## Naming Convention

```
sim-[#drones][flags]
```

- **`#drones`** – exact number of drones in the scenario  
- **Flags** (one or many, in any order)

| Flag | Meaning                                                                                 |
|------|-----------------------------------------------------------------------------------------|
| **g** | **G**lobal / world-fixed frame                                                         |
| **b** | **B**ase (standard formation or “plain” behaviour)                                     |
| **r** | **R**otational motion around the team’s centroid                                       |
| **c** | **C**ircling – each agent rotates around its own moving centre                         |
| **l** | **L**ocal frame–centric behaviour                                                      |
| **a** | **A**rUco-marker detection                                                             |
| **k** | **K**alman filter enabled                                                              |
| **s** | Kalman-driven **s**ine-wave control                                                    |
| **t** | **T**ranslational (linear) team motion                                                 |

*Combine letters to stack behaviours (e.g. `gk` = global + Kalman filter).*

---

## Package Overview

| Simulation Description                                               | Package Name |
|----------------------------------------------------------------------|--------------|
| 1 drone — global, base behaviour                                     | `sim-1gb`    |
| 2 drones — global, base behaviour                                    | `sim-2gb`    |
| 3 drones — global, base behaviour                                    | `sim-3gb`    |
| 3 drones — global, rotational motion around centroid                 | `sim-3gr`    |
| 3 drones — global, agent-based circling                              | `sim-3gc`    |
| 3 drones — global, translational team motion                         | `sim-3gt`    |
| 2 drones — **local-frame** operation with ArUco detection            | `sim-2la`    |
| 1 drone  — local-frame operation with ArUco detection                | `sim-1la`    |
| 1 drone  — global frame with Kalman filter                           | `sim-1gk`    |
| 1 drone  — global frame, Kalman filter + sine-wave control           | `sim-1gs`    |

Feel free to mix additional flag letters when new behaviours are added; just append them to the package name.

---

## Usage

Every package contains its own launch files, scripts and configuration. All depend on **`rospy`** and can be built and run independently.

### Build the workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash        # re-source whenever you add new packages
```

### Launch a simulation (example)
```bash
roslaunch sim-1gb drone.launch
```
Adjust the package name and launch file to suit your test.

---

Happy flying!
