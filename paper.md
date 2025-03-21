---
title: 'An unsteady Vortex Panel Method for Flapping Foils
tags:
  - Unsteady aerodynamics
  - Potential Flows
  - Vortex Panel
  - Kutta Condition
  - Kelvin Circulation Theorem
authors:
  - name: Rohit Chowdhury
    orcid: 0000-0001-7935-7907
    affiliation: 1
  - name: X.J. Xin
    affiliation: 1
affiliations:
 - name: Department of Mechanical Engineering, Indian Institute of Technology, Jodhpur, Rajasthan, India
   index: 1
date: 22 March 2025
bibliography: paper.bib
---

# Summary
# Statement of Need
Developing a low-fidelity solver is crucial for various aerodynamic applications where Applicable to low-Mach-number irrotational flows of homogeneous fluids, useful for initial analysis of macroscopic fluid flow away from solid boundaries. 

- **Computational Efficiency**: High-fidelity CFD solvers require significant computational resources, making them impractical for rapid design iterations. Potential flow solvers provide quick and reasonable approximations.

- **Preliminary Design & Analysis**: Early-stage aerodynamic studies often do not require full viscous modeling. A potential flow solver aids in rapid lift estimation, sensitivity analysis, and parametric studies.

- **Applicability to Low-Speed Aerodynamics**: In many engineering problems, viscous effects are confined to thin boundary layers and wakes. Outside these regions, potential flow assumptions hold well, making inviscid solvers a viable choice.

- **Versatility & Adaptability**: The developed solver can be extended to analyze various unsteady aerodynamic problems, including flapping wings and gust encounters, offering flexibility for research and engineering applications.

- **Coupling with viscous solver**: The solutions for incompressible visocus flows can also be obtained by coupling with visocus solvers. The preliminary invisicid solution is fed into the viscous solver 

# Methodology

# Acknowledgements

# References