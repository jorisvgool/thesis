# Virtual-Agent-Based Local Control – Chapter 4.3

This folder contains MATLAB scripts related to **virtual-agent-based formation control** using onboard camera sensing, as described in **Section 4.3** of the thesis.

The simulations demonstrate how agents use local visual feedback (e.g., marker tracking) to maintain formation rigidity without requiring inter-agent communication.

## File Overview

- **README.md**  
  This file. Provides an overview of the scripts and their purposes.

### Simulation Scripts

- **virtual_step.m**  
  Simulates **step-wise angular motion** of a visual marker. Models a discrete change in orientation and how agents adjust based on local virtual feedback.

- **virtual_dynamic.m**  
  Simulates **smooth, dynamic rotational motion** of a marker. Highlights how the virtual control framework handles continuous, time-varying motion.

---

## Requirements

- MATLAB (tested with R2024a)
- No additional toolboxes required

## How to Use

1. Run `virtual_step.m` to understand the controller’s response to sudden orientation changes.
2. Run `virtual_dynamic.m` for a continuous visual tracking scenario.
3. Both simulations model local perception using a virtual agent concept.

## Reference

See **Section 4.3** in the thesis for the theoretical background on vision-based virtual agent control.

## Citation

If you use these scripts for your own work, please cite the following source:
```text
van Gool, J. P. (2025). Distributed rigid formation motion control of 
multi-agent systems (Master’s thesis, University of Groningen).
```
