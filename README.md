# Vision-Based Decentralized Formation Motion Control with Predictive Methods

This repository contains supplementary materials for the Master's thesis **"Vision-Based Decentralized Formation Motion Control with Predictive Methods"** by **Joris van Gool**, submitted at the University of Groningen.

The repository is organized into folders that correspond to the main chapters of the thesis. Each folder includes a `README.md` file for further navigation and explanation.

## Repository Structure

```
/thesis
├── chapter3_formation      # MATLAB simulations for distributed motion control
├── chapter4_framework      # MATLAB simulations of theoretical control framework
├── chapter5_simulation     # Gazebo simulations, raw data, and simulation videos
├── chapter6_experiment     # Experimental setup, raw data, and video recordings
└── README.md               # This file
```

## Chapter Descriptions

- **chapter3_formation**  
  MATLAB simulations demonstrating gradient-based formation control and distributed rigid body motion (translational and rotational).

- **chapter4_framework**  
  MATLAB simulations and theoretical models for predictive control, Kalman filtering and virtual-agent-based formation control.

- **chapter5_simulation**  
  Contains Gazebo simulation models, raw log data, and rendered videos used to validate the distributed control strategies in high-fidelity environments.

- **chapter6_experiment**  
  Real-world quadrotor experiments. Includes setup documentation, raw data from the motion capture and onboard sensors, and experiment recordings.

## Getting Started

To explore the materials:

1. Clone the repository (not recommended):  
   `git clone https://github.com/jorisvgool/thesis.git`
2. Navigate into any chapter directory.
3. Open the corresponding `README.md` file for chapter-specific details and usage instructions.

## Citation

If you use this repository or any part of the thesis in your own work, please cite it as follows:

```bibtex
@mastersthesis{vangool2025thesis,
  author    = {{van Gool}, Joris},
  title     = {Vision-Based Decentralized Formation Motion Control with Predictive Methods},
  school    = {University of Groningen},
  year      = {2025},
  month     = {9},
  type      = {Master's Thesis},
  annote    = {Available at https://github.com/jorisvgool/thesis}
}
```

## License

© 2025 Joris van Gool. All rights reserved. This repository and its contents may not be reproduced, distributed, or used without the author's written consent.

---

_This repository serves as a digital appendix to the thesis, supporting transparency, reproducibility, and further research._
