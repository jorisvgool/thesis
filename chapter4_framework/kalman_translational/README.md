# Predictive Translational Motion – Chapter 4.1

This folder contains MATLAB scripts for **predictive translational motion control** using Kalman filtering, as developed in **Section 4.1** of the thesis.

The scripts demonstrate how agents estimate and track different types of translational motion using only local measurements and a predictive state estimator.

## File Overview

- **README.md**  
  This file. Provides an overview of the scripts and their purposes.

### Simulation Scripts

- **kalman_constant.m**  
  Predictive control under **constant translational motion**. Demonstrates baseline Kalman filtering for steady-state movement.

- **kalman_timevarying.m**  
  Tracks **polynomial time-varying translational motion**, allowing the agent to follow more dynamic trajectories.

- **kalman_sine_simple.m**  
  Simulates tracking of a **simple harmonic sine motion**, representing periodic translation.

- **kalman_sine_complex.m**  
  Tracks a **complex sine-based motion** with varying amplitude or frequency — showcasing the EKF’s robustness under nonlinear dynamics.

---

## Requirements

- MATLAB (tested with R2024a)
- No additional toolboxes required

## How to Use

1. Begin with `kalman_constant.m` to see the basic filter operation.
2. Explore the more dynamic behaviors in `kalman_timevarying.m`, `kalman_sine_simple.m`, and `kalman_sine_complex.m`.
3. Modify motion profiles or noise settings to test robustness.

## Reference

See **Section 4.1** in the thesis for derivation and analysis of translational Kalman filtering in distributed formation control.

## Citation

If you use these scripts for your own work, please cite the following source:
```text
van Gool, J. P. (2025). Distributed rigid formation motion control of 
multi-agent systems (Master’s thesis, University of Groningen).
```
