# Impulsive Start: Sudden Acceleration of an Airfoil

The code `impulsive_start.cpp`, located in this directory, is designed for simulating the **sudden acceleration** (impulsive start) of an airfoil in a quiescent fluid. This is a **special case** of the more general solver implemented in `main.cpp`, which handles pitch and plunge motions.

Unlike `main.cpp`, this version is **standalone** and not separated into `.cpp` and `.h` files, allowing easier readability and compilation for this specific scenario.

---

## ✨ Problem Description

At **t = 0**, the airfoil is at a **fixed angle of attack** (e.g., 5°) in a **stationary fluid**. As the simulation begins, the **freestream velocity** is suddenly set to a constant value, simulating an **impulsive start** parallel to the x-axis of the inertial frame.

---

## 🧠 Code Overview

This code follows the same numerical framework as the main solver (`main.cpp`) but with the following simplifications:

- ✅ `h_instantaneous()` and `h_dot_instantaneous()` return zero vectors — there is **no plunge**.
- ✅ `alpha_instantaneous()` returns a constant angle — the airfoil **does not pitch**.
- ✅ `alpha_dot_instantaneous()` returns zero — there is **no angular velocity**.
- ✅ Before the unsteady time loop begins, **steady-state calculations** are performed:
  - Lift is evaluated using:
    - **Kutta–Joukowsky theorem**
    - **Pressure integration**
- ✅ Real-time plotting of:
  - Wake evolution
  - Lift force (via GNUplot)

---

## 🧪 Test Case Setup

- **Airfoil**: NACA 0012 (symmetric)
- **Angle of Attack**: 5 degrees
- **Initial Condition**: Quiescent fluid at $(t <= 0)$
- **Freestream Velocity**: $Q_\infty=10\,m/s$
- **Execution Time**: ~**23.31 seconds** for the complete simulation
- ✅ The code displays the **total computational time** upon completion.

### 📊 Validation & Publication

The lift and drag coefficient plots (Cl and Cd) from this test case are included in this repository and were used in one of our [publications](https://www.researchgate.net/profile/Rohit-Chowdhury-5/publication/393158131_Development_of_an_unsteady_vortex_panel_method_for_a_flapping_airfoil/links/68623dae92697d42903bdee0/Development-of-an-unsteady-vortex-panel-method-for-a-flapping-airfoil.pdf).

---

## ⚙️ Requirements

To compile and run the code, ensure the following are installed:

### ➤ Dependencies
- [Eigen](https://eigen.tuxfamily.org/) — for linear algebra operations
- [Gnuplot](http://www.gnuplot.info/) — for real-time visualizations
- For complete dependencies guidline see [README.md](https://github.com/coding4Acause/PANKH/blob/main/README.md)

---

## 🚀 Running the Code

From terminal:

Compile:

```bash
g++ -o PANKH_impulsive impulsive_start.cpp 
```
Run:

```bash
./PANKH_impulsive  impulsive_start_input.json
```
PANKH_impulsive is the executable and impulsive_start_input.json is tailored input file for impulsive_start.cpp solver.