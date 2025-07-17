# Simulation Videos

This folder references video recordings of the simulation results described in **Chapter 5** of the thesis. Each video corresponds to a specific ROS + Gazebo simulation scenario.

Due to file size constraints on GitHub, the full-resolution `.mp4` files are hosted externally.

**Watch all simulation videos here:**  
[Google Drive – Simulation Videos](https://drive.google.com/drive/folders/1U8Y9nk2Hmu0BoirWT0podEZ7M6vjBbN0?usp=sharing)

---

## Video Naming Convention

Each video is named after the simulation package it represents. For example:

- `sim-1gs.mp4` → `sim-1gs` package (1 quadrotor, global frame, sinusoidal Kalman control)
- `sim-3gc.mp4` → `sim-3gc` package (3 quadrotors, global frame, agent-based circling)
- `sim-2la.mp4` → `sim-2la` package (2 quadrotors, local frame with ArUco detection)

Refer to the [simulation README](../README.md) for package descriptions.

---

## Notes

- Videos are compressed using `ffmpeg` with `CRF 18` for high visual quality.
- All videos were recorded directly from Gazebo with synchronized timing.
- Playback in Drive may downscale resolution; download for full quality if needed.

---

For additional recordings or raw simulation data, please contact the author.
