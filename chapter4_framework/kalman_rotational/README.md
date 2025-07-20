# Predictive Rotational Motion – Chapter 4.5

This folder contains MATLAB scripts for **predictive rotational motion control** using Kalman filtering, as described in **Section 4.5** of the thesis.

The included examples demonstrate how agents estimate and follow **rotational motion trajectories** — both constant and time-varying — using only local measurements and predictive filtering.

## File Overview

- **README.md**  
  This file. Provides an overview of the scripts and their purposes.

### Core Function

- **mu_rotation.m**  
  Shared function to compute the **rotational motion parameters** for each agent, used as input to the predictive controller.

### Simulation Scripts

- **kalman_rotational_constant.m**  
  Simulates **rotational motion with constant angular change** (`γ` constant). Demonstrates how Kalman filtering can track and predict steady rotational behavior.

- **kalman_rotational_timevarying.m**  
  Simulates **time-varying rotational motion**, modeled as a polynomial function of time. Highlights how agents handle non-uniform angular velocity using prediction.

---

## Requirements

- MATLAB (tested with R2024a)
- No additional toolboxes required

## How to Use

1. Start with `kalman_rotational_constant.m` to understand the filter behavior for constant angular motion.
2. Run `kalman_rotational_timevarying.m` to explore how the system handles dynamic, time-varying rotation profiles.
3. Refer to `mu_rotation.m` for the motion parameter calculation used by both scripts.

## Reference

See **Section 4.5** in the thesis for the derivation of the predictive rotational control strategy using Kalman filtering.

## Citation

If you use these scripts for your own work, please cite the following source:
```text
van Gool, J. P. (2025). Distributed rigid formation motion control of 
multi-agent systems (Master’s thesis, University of Groningen).
```
