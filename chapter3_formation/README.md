# Chapter 3 - Formation Control Simulations

This folder contains MATLAB scripts and functions related to **gradient-based formation control** and **distributed motion control** for multi-agent systems, as discussed in Chapter 3 of the thesis.

## File Overview

- **README.md**  
  This file. Provides an overview of the simulation files and their purposes.

### Formation Control Simulations

- **gradient_control_2D.m**  
  Simulates gradient-based formation control for a **triangular formation** in 2D.

- **gradient_control_2D_sq.m**  
  Simulates gradient-based formation control for a **square formation** in 2D.

- **gradient_control_3D.m**  
  Simulates gradient-based formation control for a **triangular formation** in 3D.

### Distributed Motion Control

- **motion_control_example.m**  
  Demonstrates distributed control of formation **translation and rotation** using motion parameters. This is one of the **examples** featured in the thesis.

- **motion_control_sandbox.m**  
  A sandbox script to help understand and experiment with how the motion parameter functions work.

### Motion Parameter Functions

- **mu_rotation.m**  
  Computes the motion parameters for **rotational movement** of the formation.

- **mu_translation.m**  
  Computes the motion parameters for **translational movement** of the formation.

---

## Requirements

- MATLAB (tested with R2024a)
- No additional toolboxes are required.

## How to Use

1. Open any of the simulation scripts (e.g., `gradient_control_2D.m`) in MATLAB.
2. Run the script to visualize agent behavior under different control laws.
3. For distributed motion control, explore `motion_control_example.m` first.
4. Use `motion_control_sandbox.m` to learn about the motion parameters.

## Citation

If you use these scripts for your own work, please cite the following source:
```text
van Gool, J. P. (2025). Distributed rigid formation motion control of 
multi-agent systems (Master’s thesis, University of Groningen).
```
