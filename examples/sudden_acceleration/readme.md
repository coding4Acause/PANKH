# Description of the Code for Sudden Acceleration (Impulsive Start) of an Airfoil

The code `impulsive_start.cpp`, located in this directory, is designed for the sudden acceleration case, which is a special scenario of the main solver `main.cpp`. Unlike `main.cpp`, which is a generalized implementation for plunge and pitching kinematics, this version is standalone and not separated into .cpp and .h files.

At **t = 0**, the airfoil is at a fixed angle of attack, immersed in a quiescent fluid. The moment the simulation starts, the freestream suddenly attains a velocity parallel to the x-axis of the inertial frame.

---
 # Code Overview

- This code follows the same structure as the main solver used for the pitch-plunge case, with a few key differences:
- The functions `h_instantaneous` and `h_dot_instantaneous` return zero vectors — there is no plunge.
  - The function `alpha_instantaneous()` returns a constant angle of attack — the airfoil does not pitch.
  - `alpha_dot_instantaneous()` function returns zero — there is no angular velocity.
- Before time loop starts the steady states calculations are performed and lift is evaluated using **Kutta-Joukowsky theorem** and also using **pressure integration**.
- The test case was run for a symmetric NACA 0012 airfoil at an angle of attack of 5 degrees, initially placed in a quiescent fluid at t <= 0, and then the freestream velocity is suddenly set to $Q_\infty = 10\,m/s$. The plots for lift and drag coefficients (`Cl` and `Cd`) are present in this same repository. These results are currently used in one of our [publications](https://www.researchgate.net/profile/Rohit-Chowdhury-5/publication/393158131_Development_of_an_unsteady_vortex_panel_method_for_a_flapping_airfoil/links/68623dae92697d42903bdee0/Development-of-an-unsteady-vortex-panel-method-for-a-flapping-airfoil.pdf).
- When the user runs the code, real-time plots of the wake evolution and lift forces are displayed using Gnuplot in the background. This feature is also present in the main solver.

---

# Important guidlines

- To run this code, ensure that:

  - The Eigen library is installed (for matrix computations).

  - Gnuplot is available (for real-time visualization).

- Detailed installation instructions are provided in the prerequisites section of the main
[`README.md`](https://github.com/coding4Acause/2d_UnsteadyVortexPanel/blob/main/README.md) 

