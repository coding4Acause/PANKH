---
title: 'PANKH: An unsteady potential flow solver for hovering airfoils'
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
  - name: Nipun Arora
    orcid: 0000-0002-1835-1189
    affiliation: 1
  - name: Ashish Pathak 
    orcid: 0000-0002-6608-1830
    affiliation: 1
affiliations:
  - index: 1
    name: Department of Mechanical Engineering, Indian Institute of Technology, Jodhpur, Rajasthan, India
date: 4 April 2025
bibliography: paper.bib
---

# Summary
Analyzing the aerodynamics of unsteady airfoils is vital for decoding the flight dynamics of birds, insects, fixed-wing aircrafts and micro air vehicles (MAVs). Although computational fluid dynamics (CFD) delivers in-depth results, lower-fidelity approaches like potential flow solvers offer significant value due to their speed and capacity to model essential aerodynamic effects.

We introduce PANKH **(Panel Analysis of uNsteady Kinematics of Hovering airfoils)**, an open-source C++ tool that employs the unsteady vortex panel method to evaluate aerodynamic forces on airfoils in arbitrary motion. Its flexible, modular design enables users to specify custom kinematic patterns, ideal for exploring bio-inspired flapping flight and gust responses.

The solver’s source code, validation examples, and documentation are hosted on GitHub, guaranteeing ease of access and replicability. Upcoming enhancements might feature broader input capabilities, refined wake modeling approaches, and the integration of alternative singularity distributions.

# Statement of Need

For low-speed aerodynamic applications, where viscous effects are primarily confined to thin boundary layers and wakes, potential flow solvers provide a computationally efficient alternative to high-fidelity CFD. These solvers approximate the flow as inviscid, incompressible, and irrotational, reducing complexity while still offering accurate lift estimations. By solving Laplace’s equation with appropriate boundary conditions, potential flow methods enable rapid aerodynamic analysis, making them particularly useful for preliminary design, scaling studies, and parametric investigations. Despite their advantages, open-source tools for unsteady potential flow analysis remain scarce. While [XFOIL](https://web.mit.edu/drela/Public/web/xfoil/), developed by Mark Drela, is a widely used tool for steady-state airfoil aerodynamics, it lacks the capability to handle unsteady effects. Existing research codes for unsteady vortex panel methods are often inaccessible, undocumented, or not actively maintained.

To address this gap, we introduce PANKH, an open-source unsteady vortex panel solver designed to compute aerodynamic loads for airfoils undergoing arbitrary motion, such as pitching, plunging, and sudden acceleration. By leveraging a modular implementation, PANKH ensures computational efficiency while enabling users to explore unsteady aerodynamic phenomena with greater accessibility.

# Methodology

### Governing Equation and Boundary Conditions
The numerical framework solves Laplace’s equation, $\nabla^2\Phi=0$, using appropriate boundary conditions. These include the no-penetration boundary condition and the far-field boundary condition. The latter is inherently satisfied by the elementary solutions of the Laplace equation.
In this solver, we employ the Neumann condition, meaning that the problem is formulated in terms of velocity rather than directly solving for $\Phi$. The velocity field is then expressed in terms of singularity distributions placed along the airfoil surface.

### Discretization of Geometry and Singularity Element Distribution

The airfoil is discretized into n nodes (panel vertices), forming n-1 flat panels. Each panel has a control point at its midpoint where the no-penetration condition is enforced:

$$
[(\boldsymbol{V}_{bound}+ \boldsymbol{V}_{wake}+\boldsymbol{V}_{kin})_i\cdot\hat{\boldsymbol{n}}_i]_{t_k}=0\,,(1 \leq i \leq n-1)
$$

The total number of unknowns in the system is $n+1$:
 1) $n$ bound vortex strengths $\gamma_i\,(1 \leq i \leq n)$ on the airfoil.
 2) $\gamma_{wp}$, the strength of the latest shed wake panel. [@vezza1985method,@basu_1978]
 
As illustrated in **Figure 1**, the solver employs a *piecewise linearly varying vortex distribution* on the airfoil surface, while in the wake, a constant-strength vortex panel is shed from the trailing edge at each time step. These wake vortices are then convected with the local velocity field and influence the induced velocity at subsequent time steps.

![A schematic diagram of the panel discretization on an airfoil and wake modeling at time
instant $t_k$ in the inertial frame of reference xy. ](unsteady_model_at_tk_page-0001.jpg)

### Other Physical Considerations

#### Trailing Edge Condition(Kutta Condition)
Satisfying the boundary conditions alone do not yield a unique solution for $\gamma_i\,(1 \leq i \leq n)$. To obtain an unique solution, the flow must leave the airfoil's sharp trailing edge smoothly along the bisector line, which is the well known ***Kutta Condition*** [@eldredge2019mathematical].

$$
\Gamma_{\text{TE}}(t_k)=0
$$

$$
\gamma_1(t_k)+\gamma_n(t_k)+\gamma_{wp}(t_k)=0
$$

#### Kelvin’s Circulation Theorem
For unsteady flows, the motion of the airfoil causes wake formation. The reason for wake formation can be explained by surrounding the airfoil by a sufficiently large contour, and noting that only conservative forces, such are pressure act on the contour. [@katz2001low] Then Kelvin’s theorem states that the total circulation in the contour remains constant, i.e., 

$$
\frac{\mathrm{D}\Gamma}{\mathrm{D}t}= \frac{\mathrm{D}}{\mathrm{D}t}\left(\Gamma_{\text{airfoil}}+\Gamma_{\text{wake}}\right)=0
$$

The resulting system, formulated as shown in **Figure 2**, is expressed as a single matrix equation and solved using the QR decomposition method, specifically the *ColPivHouseholderQR* class from the *Eigen library*.

![Equations formulated to determine the set of unknowns at a time instant $t_k$.](ax=b.jpg)

---

The entire numerical framework discussed above is implemented in the code. The implementation is structured into separate .cpp files, each containing functions dedicated to specific numerical tasks. The function and variable names are chosen to align with their respective roles in the numerical framework, ensuring clarity and ease of use for the user. A detailed guide on input paramters, motion specification, and output interpretation is provided in the project's [README](https://github.com/coding4Acause/2d_UnsteadyVortexPanel/blob/main/README.md). An extensive testing and validation suite has been used to ensure the accuracy of the results obtained from the source code. All validation results [@dimitri] are available in the project's GitHub repository, specifically in the [`examples`](https://github.com/coding4Acause/PANKH/tree/main/examples) directory.


# Acknowledgements
This work was supported by ABC Transformers Pvt. Ltd. under grant no. C/ABC/NPA/20220065

# References
