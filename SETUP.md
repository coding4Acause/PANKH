# Setup Guide

## Output Directories
The results of the solver are stored in the following directory:

- `output_files/` → Contains all the output files, organized as follows:

  - `output_files/a_matrix_file/` → Stores the A matrix (coefficient matrix of Ax = b) at each time step.
  - `output_files/airfoil_normal_file/` → Stores the unit normal vector of all panels at each time step.
  - `output_files/b_vector_file/` → Stores the b vector (right-hand side of Ax = b) in separate files for each time step.
  - `output_files/gamma_vector_file/` → Stores the unknown vector (vector(of circulation strengths), or x in Ax = b) in separate files for each time step.
  - `output_files/potential_file/` → Stores the potential $\phi$ over the entire airfoil at each time step.
  - `output_files/pressure_file/` → Stores the pressure distribution over the airfoil's control points at each time step.
  - `output_files/vortex_shedding/` → Contains:
    - `motion_i` files (`i = 0:iterMax`) → Represent the airfoil’s position in the inertial frame at each time step.
    - `wake_i` files (`i = 0:iterMax`) → Contain the positions of wake vortices at corresponding time steps.

> **Note:** Users can modify `main.cpp` to disable printing these files if not needed.

---

## How to Ensure Proper Setup
If the `output_files/` directory and its subdirectories do not exist, create them before running the solver using:

```sh
mkdir -p output_files/{a_matrix_file,airfoil_normal_file,b_vector_file,gamma_vector_file,potential_file,pressure_file,vortex_shedding}
