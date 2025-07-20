# Chapter 4 Theoretical framework

This folder contains the implementation of the control and estimation strategies developed in **Chapter 4**.

Each module corresponds to a specific part of the unified formation control framework, covering predictive motion estimation, agent-based rotational control, and virtual-local coordination using onboard vision.

## Folder Structure

```
chapter4_framework/
├── agent_rotational/          # Section 4.4: Agent-centered rotational motion
├── kalman_rotational/         # Section 4.5: Predictive rotational control (Kalman filter)
├── kalman_translational/      # Section 4.1: Predictive translational control (Kalman filter)
├── virtual_local/             # Section 4.3: Virtual-agent-based local formation control
└── README.md
```

## Module Overview

- **`agent_rotational/`**  
  Implements local rotational control where each agent enforces rigidity through angular correction.  
  _Refer to Section 4.4 of the thesis._

- **`kalman_rotational/`**  
  Kalman filter-based prediction of rotational motion for distributed control under dynamic conditions.  
  _Refer to Section 4.5._

- **`kalman_translational/`**  
  Predictive translational motion control using Kalman filtering for formations with passive agents.  
  _Refer to Section 4.1._

- **`virtual_local/`**  
  Virtual-agent-based local formation control using only onboard camera sensing (no inter-agent communication).  
  _Refer to Section 4.3._

  **See the `README.md` files in the respective folders for more details on individual scripts and functions.**
