# Agent-Centered Rotational Motion – Chapter 4.4

This folder contains MATLAB scripts and functions related to **agent-centered rotational motion control** in multi-agent systems, as developed in **Section 4.4** of the thesis.

## File Overview

- **README.md**  
  This file. Provides an overview of the scripts and their purposes.

### Core Function

- **mu_rotation.m**  
  Computes distributed **rotational motion parameters** for individual agents. These parameters are used to maintain rigid formation during rotational motion.

### Simulation Scripts

- **rotationA_example.m**  
  Example simulation of the **agent-centered rotational control algorithm** in a 2D setting. Demonstrates how agents maintain formation using only local sensing.

- **rotationA_sandbox.m**  
  A sandbox environment to experiment with the rotational control algorithm and visualize agent behavior.

---

## Requirements

- MATLAB (tested with R2024a)
- No additional toolboxes required

## How to Use

1. Run `rotationA_example.m` to see a standard example of the algorithm in action.
2. Use `rotationA_sandbox.m` to explore different initial conditions or modify formation settings.
3. Refer to `mu_rotation.m` to understand or modify how motion parameters are computed.

## Reference

See **Section 4.4** in the thesis for theoretical background and derivation.

## Citation

If you use these scripts for your own work, please cite the following source:
```text
van Gool, J. P. (2025). Distributed rigid formation motion control of 
multi-agent systems (Master’s thesis, University of Groningen).
```
