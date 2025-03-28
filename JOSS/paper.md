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
  - name: Department of Mechanical Engineering, Indian Institute of Technology, Jodhpur,   Rajasthan, India
  - index: 1
date: 28 March 2025
bibliography: paper.bib
---

# Summary
Aerodynamic analysis of unsteady airfoils plays a crucial role in understanding the flight mechanics of birds, insects, and micro air vehicles (MAVs). While computational fluid dynamics (CFD) provides detailed insights, lower-fidelity models such as potential flow solvers remain valuable due to their efficiency and ability to capture key aerodynamic phenomena. In particular, two-dimensional (2D) inviscid simulations serve as an essential tool for rapid analysis of unsteady aerodynamics, reducing computational costs while retaining fundamental flow physics.

This work presents **PANKH (Panel Analysis of uNsteady Kinematics of Hovering airfoils)**, an open-source C++ solver implementing the unsteady vortex panel method for analyzing the aerodynamic forces on airfoils undergoing arbitrary motion. The solver discretizes the airfoil surface into panels with a piecewise linear vortex distribution and enforces the no-penetration boundary condition at control points. The Kutta condition ensures smooth trailing-edge flow, while the Kelvin circulation theorem governs the shedding of wake vortices. The wake is dynamically modeled using constant-strength vortex panels, which are convected as discrete point vortices in subsequent time steps. The resulting system of equations is solved using the Eigen library, providing an efficient and robust numerical framework.

PANKH computes unsteady aerodynamic forces such as lift, drag, and moment coefficients for airfoils undergoing pitching, plunging, or combined motions. The modular structure of the code allows users to define custom kinematic profiles, making it suitable for studying bio-inspired flapping flight, rotor aerodynamics, and gust interactions. The solver is designed for high computational efficiency, making it a valuable tool for researchers and engineers investigating unsteady aerodynamics without the computational expense of full CFD simulations.

The source code, validation cases, and documentation are available on GitHub, ensuring accessibility and reproducibility. Future developments may include expanded input handling, additional wake modeling techniques, and extensions to three-dimensional flow analysis.


# Statement of Need

High-fidelity solvers based on the Navier-Stokes equations discretize the entire fluid domain and solve the nonlinear, coupled partial differential equations using various numerical techniques such as finite difference, finite volume, finite element, and spectral methods. These solvers provide a comprehensive analysis of flow physics, capturing effects such as boundary layer development, turbulence, and wake interactions. However, they are computationally expensive and require significant numerical resources. Additionally, the workflow involves extensive pre-processing steps, including grid generation, mesh refinement, and convergence studies, making them both time-consuming and resource-intensive. As a result, their application is typically restricted to cases where high accuracy is critical, such as final design validation and complex aerodynamic studies.

For low-speed aerodynamic applications, where viscous effects are confined to thin boundary layers and wakes, an alternative approach is to approximate the flow outside these regions as inviscid, incompressible, and irrotational. Under these conditions, the problem simplifies to a potential flow problem, which significantly reduces computational complexity while still providing valuable aerodynamic insights. Potential flow solvers are particularly useful for preliminary design, scaling analyses, and selecting input parameters for higher-fidelity simulations. Since lift is primarily influenced by pressure forces, potential flow solvers can provide accurate lift estimations, often closely matching real-world results. The key advantage of potential flow models arises from their mathematical formulation. The general continuity equation simplifies to a second-order homogeneous linear partial differential equation known as Laplace’s equation. Due to the linear nature of Laplace’s equation, elementary solutions can be superimposed to construct more complex flow fields. The problem then reduces to determining the distribution and strengths of these elementary solutions (such as source, sink, and vortex distributions) on the surface of a given geometry to satisfy boundary conditions.

To solve unsteady potential flow problems, researchers have developed both analytical and numerical methods. Early analytical models by Wagner (1925) and Theodorsen (1935) provided closed-form solutions but were limited to simplified airfoil geometries. Numerical techniques emerged to address these limitations, beginning with Hess and Smith’s (1967) [@hess1967calculation] steady panel method, later extended for unsteady aerodynamics by Geising (1968) [@giesing1968nonlinear]. In Basu and Hancock (1978) [@basu_1978], a Kutta condition with zero loading across the trailing edge was introduced, which was later refined by Vezza and Galbraith (1985) [@vezza1985method] using a zero trailing-edge vorticity Kutta condition for improved accuracy and numerical stability.

While these methods provide a foundation, no widely available open-source tool currently implements an unsteady vortex panel method for practical aerodynamic analysis. Notably, XFOIL, developed by Mark Drela, is limited to steady aerodynamics, leaving a gap in publicly available tools for unsteady cases. This work aims to bridge that gap by developing an in-house unsteady vortex panel solver, following Vezza et al. (1985), to efficiently compute aerodynamic loads for prescribed airfoil motions such as sudden acceleration, pitching, and plunging.

Key Motivations for the low-fidelity solver can be summarised as follows:

- **Computational Efficiency**: High-fidelity CFD solvers require significant computational resources, making them impractical for rapid design iterations. Potential flow solvers provide quick and reasonable approximations.

- **Preliminary Design & Analysis**: Early-stage aerodynamic studies often do not require full viscous modeling. A potential flow solver aids in rapid lift estimation, sensitivity analysis, and parametric studies.

- **Applicability to Low-Speed Aerodynamics**: In many engineering problems, viscous effects are confined to thin boundary layers and wakes. Outside these regions, potential flow assumptions hold well, making inviscid solvers a viable choice.

- **Versatility & Adaptability**: The developed solver can be extended to analyze various unsteady aerodynamic problems, including flapping wings and gust encounters, offering flexibility for research and engineering applications. 

- **Lack of Existing Open-Source Software**: While [XFOIL](https://web.mit.edu/drela/Public/web/xfoil/) is a powerful tool for steady-state airfoil aerodynamics, it does not handle unsteady aerodynamic effects. Although some research codes for unsteady vortex panel methods exist, there is currently no widely available, well-documented, and actively maintained open-source implementation. This highlights the need for an accessible and reliable solver for unsteady airfoil aerodynamics.

# Methodology

### Governing Equation and Boundary Conditions
The numerical framework reduces Laplace’s equation, $\nabla^2\Phi=0$, to a system of linear algebraic equations using appropriate boundary conditions. These include the no-penetration boundary condition and the far-field boundary condition. The latter is inherently satisfied by the elementary solutions of the Laplace equation, such as sources, doublets, and point vortices. The linear nature of Laplace’s equation allows the use of the superposition principle, enabling the construction of solutions for different flow geometries by combining elementary solutions.

The no-penetration boundary condition can be imposed using either:
 1) The Dirichlet condition (specifying $\Phi$ along the boundary), or
 2)  The Neumann condition (specifying $\partial\Phi/\partial\boldsymbol{n}$ along the boundary).

In this solver, we employ the Neumann condition, meaning that the problem is formulated in terms of velocity rather than directly solving for $\Phi$. The velocity field is then expressed in terms of singularity distributions placed along the airfoil surface.

### Discretization of Geometry and Singularity Element Distribution

The airfoil is discretized into n nodes (panel vertices), forming n-1 flat panels. Each panel has a control point at its midpoint where the no-penetration condition is enforced. The singularity element used in this solver is a *piecewise linearly varying vortex distribution*.
 - The bound vorticity on the airfoil is represented by vortex strengths 
$\gamma_i$ defined at the $n$ nodes.

- In the wake, a *constant-strength vortex panel* is shed from the trailing edge at each time step, representing the vorticity shed into the wake due to unsteady effects. These wake vortices are convected with the local velocity field and contribute to the total induced velocity at each time step.
The total number of unknowns in the system is $n+1$:
 1) $n$ bound vortex strengths $\gamma_i\,(1 \leq i \leq n)$ on the airfoil.
 2) $\gamma_{wp}$, the strength of the latest shed wake panel. 
 
 This discretization and singularity element distribution are illustrated in **Figure 1**.

![A schematic diagram of the panel discretization on an airfoil and wake modeling at time
instant $t_k$ in the inertial frame of reference xy. ](unsteady_model_at_tk_page-0001.jpg)

### Other Physical Considerations

#### Trailing Edge Condition(Kutta Condition)
Satisfying the boundary conditions alone do not yield a unique solution for $\gamma_i\,(1 \leq i \leq n)$. To obtain an unique solution, the flow must leave the airfoil's sharp trailing edge smoothly along the bisector line, which is the well known ***Kutta Condition*** [@eldredge2019mathematical].

#### Kelvin’s Circulation Theorem and Wake Modeling
For unsteady flows, the motion of the airfoil causes wake formation. The reason for wake formation can be explained by surrounding the airfoil by a sufficiently large contour, and noting that only conservative forces, such are pressure act on the contour. Then Kelvin’s theorem states that the total circulation in the contour remains constant, i.e., 

\begin{equation}\label{eq2}
\frac{\mathrm{D}\Gamma}{\mathrm{D}t}= \frac{\mathrm{D}}{\mathrm{D}t}\left(\Gamma_{\text{airfoil}}+\Gamma_{\text{wake}}\right)=0
\end{equation}

Wake Modeling: The latest shed vorticity in the wake is represented using constant-strength vortex panel, which is convected as point vortices in subsequent time steps. The wake influences the bound vortex distribution, however the orientation $\theta_{wp}$ and length $l_{wp}$ of the wake panel are unknown at the beginning of each time step, since they are dependent on the strength of the bound vortices[@basu_1978;@dimitri]. This introduces non-linearity in the equations. To resolve it, a Jacobian matrix and a residual vector are constructed to solve for $l_{wp}$ and $\theta_{wp}$ using the Newton-Raphson iterative method. After finding these values are substituted in a set of linear algebraic equations as expressed in the following paragraph and **Figure 2**.

### Development of the system of algebraic equations
The unknown vortex strengths are determined by solving a system of $n+1$ equations:
1) The $n-1$ conditions of zero normal flow are enforced by applying the outer Neumann normal velocity boundary condition at the midpoint (control point or collocation point) of each panel.

    \begin{equation}
    [(\boldsymbol{V}_{bound}+ \boldsymbol{V}_{wake}+\boldsymbol{V}_{kin})_i\cdot\hat{\boldsymbol{n}}_i]_{t_k}=0\,,(1 \leq i \leq n-1) \label{eq:1}
    \end{equation}
    
2)  The $n^{th}$ equation is derived from the conditon of zero vorticity at the trailing edge (Kutta Conditon) to achieve smooth outflow at the trailing edge,
    \begin{gather}
      \Gamma_{\text{TE}}(t_k)=0\\ \label{eq:2}
      \gamma_1(t_k)+\gamma_n(t_k)+\gamma_{wp}(t_k)=0      
    \end{gather}

3)  The $(n+1)^{th}$ equation is obtained from Kelvin circulation theorem, which states that the total circulation in the flow must be conserved,  
  \begin{equation}
  \frac{D\Gamma}{Dt}=0\label{eq:3}
  \end{equation}

The resulting system, formulated as shown in **Figure 2**, is expressed as a single matrix equation and solved using the QR decomposition method, specifically the *ColPivHouseholderQR* class from the *Eigen library*.

![Equations formulated to determine the set of unknowns at a time instant $t_k$.](ax=b.jpg)

### Force Computation
Once the vortex strengths are determined, the unsteady Bernoulli equation is used to compute the pressure difference between the upper and lower airfoil surfaces. The aerodynamic force coefficients are then obtained by integrating the pressure distribution along the airfoil.

---

The entire numerical framework discussed above is implemented in the code. The implementation is structured into separate .cpp files, each containing functions dedicated to specific numerical tasks. The function and variable names are chosen to align with their respective roles in the numerical framework, ensuring clarity and ease of use for the user. A detailed guide on input paramters, motion specification, and output interpretation is provided in the project's [README](https://github.com/coding4Acause/2d_UnsteadyVortexPanel/blob/main/README.md). An extensive testing and validation suite has been used to ensure the accuracy of the results obtained from the source code. All validation results are available in the project's GitHub repository, specifically in the [`examples`](https://github.com/coding4Acause/PANKH/tree/main/examples) directory.


# Acknowledgements
This work was supported by ABC Transformers Pvt. Ltd. under grant no. C/ABC/NPA/20220065
# References