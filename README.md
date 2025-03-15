# 2D Unsteady Vortex Panel method

## Description
This project implements a 2D unsteady vortex panel method to study the aerodynamics of flapping foils. It is a **low-fidelity solver** that employs the **Laplace equation** to compute the velocity distribution in the flow field while enforcing the *Neumann boundary condition* to satisfy the *no-penetration (zero normal flux)* condition on the airfoil surface. The unsteady form of Bernoulli equation is applied to calculate the pressure difference across the airfoil, enabling the computation of aerodynamic loads. 

The essence of this approach lies in transforming a partial differential equation (PDE) into a system of linear algebraic equations, reducing the problem to a linear algebra problem. The real geometry is discretized into a series of flat panels, and a piecewise linearly varying vortex panel method is used to simulate the flow around the airfoil. At each time instant, a constant-strength vortex panel is shed from the trailing edge to satisfy *Kelvin's Circulation Theorem*, ensuring the correct representation of unsteady effects.

This solver is implemented in C++ and utilizes the **Eigen library** for efficient matrix operations. The code computes unsteady aerodynamic forces acting on an airfoil undergoing prescribed motion. A detailed document explaining the numerical framework will be provided separately.

## Installation

### Prerequisites
Before compiling and running the code, ensure the installation of the following dependencies:

**Compiler support**:- C++ compiler is needed that supports C++ 11 or later. Some recommended options:

  - GCC (GNU Compiler Collection)
  - Intel C++ Compiler (ICPC/ICX/IPCX)

 **Eigen Library** (Required for matrix operations):
  - Install via package manager:
    ```bash
    sudo apt install libeigen3-dev  # Ubuntu
    ```
  - Or download manually from [Eigen's official website](https://eigen.tuxfamily.org/).
  - Install, compile and run a code which uses Eigen Library ==> [Getting started](https://eigen.tuxfamily.org/dox/GettingStarted.html)

### Getting the Source Code

You can obtain the source code in two ways:

#### Cloning the Repository (Recommended for Development)
If you want to contribute or track changes, clone the repository using Git:
```bash
git clone https://github.com/coding4Acause/2d_UnteadyVortexPanel.git
cd 2d_UnsteadyVortexPanel
```
#### Downloading as a ZIP (For Direct Usage)

If you only need the code without version control:

1) Go to the GitHub repository.
2) Click the "Code" button.
3) Select "Download ZIP".
4) Extract the ZIP file and navigate to the extracted folder.

## Documentation
