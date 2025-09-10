# PANKH: A Potential Flow Solver for Hovering Airfoils

[![License](https://img.shields.io/github/license/coding4Acause/PANKH?color=blue)](LICENSE)
![Latest Release](https://img.shields.io/github/v/release/coding4Acause/PANKH?style=flat-square)
![Repo Size](https://img.shields.io/github/repo-size/coding4Acause/PANKH?color=blue)
![C++](https://img.shields.io/badge/C%2B%2B-11-blue)
![Dependencies](https://img.shields.io/badge/Dependencies-Eigen%2C%20Gnuplot-orange)
![GitHub issues](https://img.shields.io/github/issues/coding4Acause/PANKH)
![GitHub stars](https://img.shields.io/github/stars/coding4Acause/PANKH?style=social)
![Last Commit](https://img.shields.io/github/last-commit/coding4Acause/PANKH)
[![CI](https://github.com/coding4Acause/PANKH/actions/workflows/CI.yml/badge.svg)](https://github.com/coding4Acause/PANKH/actions/workflows/CI.yml)
[![API Docs](https://img.shields.io/badge/API--Docs-Click%20Here-blue?style=flat-square)](https://coding4Acause.github.io/PANKH/)

## Description
This project implements **PANKH** (Panel Analysis of uNsteady Kinematics of Hovering airfoils), a solver designed to study the aerodynamics of flapping foils. PANKH is a **low-fidelity solver** that solves the **Laplace equation** to determine the velocity distribution in the flow field. It enforces the **Neumann boundary condition** to satisfy the **no-penetration condition** (zero normal flux) on the airfoil surface.   The **unsteady Bernoulli equation** is then applied to compute the pressure difference across the airfoil, enabling the calculation of aerodynamic loads.

The essence of this approach lies in transforming a partial differential equation (PDE) into a system of linear algebraic equations, reducing the problem to a linear algebra problem. The real geometry is discretized into a series of flat panels, and a piecewise linearly varying vortex panel method is used to simulate the flow around the airfoil. At each time instant, a constant-strength vortex panel is shed from the trailing edge to satisfy *Kelvin's Circulation Theorem*, ensuring the correct representation of unsteady effects.

This solver is implemented in C++ and utilizes the **Eigen library** for efficient matrix operations. The code computes unsteady aerodynamic forces acting on an airfoil undergoing prescribed motion. A detailed document explaining the numerical framework will be provided separately.
![unsteady_model_at_tk-1](https://github.com/user-attachments/assets/b764fcf8-4402-4e61-bb78-6558aa271894)

## Features
- **Real-time wake visualization** using Gnuplot for enhanced flow analysis.

- **Live tracking** of Cl vs time plot to observe aerodynamic force variations dynamically.

- Support for any NACA 4-digit series airfoil, allowing user-defined airfoil selection.

- **Flexible motion simulation** — the code can be easily modified to analyze various kinematic motions

- **User-controlled wake modeling** – The `input.json` file allows users to choose between prescribed wake and free wake analysis.

- **Flexible panel discretization** – The code supports both *even* and *odd* numbers of panels with *cosine clustering* for improved resolution near the leading and trailing edges. For details, refer to the `nodal_coordinates_initial` function in `geometry.cpp`.


## Installation

### Prerequisites
Before compiling and running the code, ensure the installation of the following dependencies:

<details>
  <summary> Install a C++ Compiler</summary>
  
   A C++ compiler supporting the C++11 standard or later is required for building the project. Recommended compilers include:
    
  - **Clang**: A high-performance, LLVM-based compiler with robust C++ support.
  - **GCC**: The GNU Compiler Collection, widely used for C++ development.
  - **Intel oneAPI DPC++/C++ Compiler (icpx)**: Optimized for high-performance computing
  - To install GCC on Ubuntu:
    ```bash
    sudo apt install g++
    ```
  - To install Clang on Ubuntu and ensure compatibility with the GNU C++ standard library (libstdc++):

    ```bash
    sudo apt install clang libstdc++-8-dev
    ```
    - clang: Provides the Clang compiler (clang++).
    - libstdc++-8-dev: Installs the GNU C++ standard library headers (e.g., `<iostream>`, `<cmath>`, `<vector>`) required for Clang to compile C++ code using libstdc++. 
    - For Ubuntu 18.04, libstdc++-8-dev is typically compatible; for other versions, use libstdc++-dev or the version matching your GCC installation (e.g., libstdc++-10-dev for Ubuntu 20.04).
</details>

<details>
  <summary> Install Eigen Library (Required for matrix operations:)</summary>
     
- On Ubuntu:
  - Install via package manager (Recommended):
    ```bash
    sudo apt install libeigen3-dev  # Ubuntu
    ```
    This installs Eigen headers typically in `/usr/include/eigen3`.

  - Manual Installation: Download Eigen from the official website and extract it to a directory `(e.g., /usr/local/include/eigen3)`. Update the include path during compilation if necessary. [Eigen's official website](https://eigen.tuxfamily.org/).

- On macOS:
  - Install Eigen via Homebrew:
  ```bash
     brew install eigen
  ```
  - When compiling, you may need to specify the Eigen include path explicitly:
  ```bash
  g++ -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/ -Iinclude src/*.cpp -o PANKH_solver
  ```
  Note: When executing the program, avoid extra whitespace:
  ```bash
  ./PANKH_solver    # Correct
  ./ PANKH_solver   # Incorrect
  ```
- For further help: [Getting started with Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
</details>

<details>
  <summary>Install Gnuplot (Required for real-time in-situ plotting)</summary>
  
  - **Gnuplot** is needed for real-time visualization of results.
  - Install Gnuplot on Ubuntu:
    ```bash
    sudo apt install gnuplot
    ```
  - To test if Gnuplot is working, run:
    ```bash
    gnuplot
    ```
    If it opens a terminal, Gnuplot is installed correctly.
</details>

<details>
  <summary>Install X11 (Required for Gnuplot visualization)</summary>
  
  - Install X11 support on Ubuntu:
    ```bash
    sudo apt install x11-apps
    ```
  - Verify installation:
    ```bash
    xeyes  # Should open a graphical window with moving eyes
    ```
  - You can also check the Gnuplot terminal type:
    ```bash
    gnuplot
    set terminal
    ```
    If `x11` is missing, install **X11** as shown above.
</details>
  
### Getting the Source Code

You can obtain the source code in two ways:
<details>
  <summary>  Cloning the Repository (Recommended for Development)</summary>

If you want to contribute or track changes, clone the repository using Git:
```bash
git clone https://github.com/coding4Acause/PANKH.git 
cd PANKH  # this is simply the name of the local(host system) directory
```
<!-- cd 2d_UnsteadyVortexPanel is the name of the local(in the host system) directory for the project -->
</details>

<details>
 <summary> Downloading as a ZIP (For Direct Usage) </summary>

If you only need the code without version control:

1) Go to the GitHub repository.
2) Click the "Code" button.
3) Select "Download ZIP".
4) Extract the ZIP file and navigate to the extracted folder.
</details>

## Compilation
To compile the code, ensure all .cpp and .h files are in the appropriate directories.
<details>
<summary> The typical structure is: </summary>

PANKH  # the name of the local repository
- │── /src          # Contains all .cpp source files
- │── /include      # Contains all .h header files     
- │── README.md        
- │── LICENSE 
- │── /output_files  
- │── input.json    # the input file 
</details>

<details>
<summary> clang compiler </summary>
To compile with Clang, use the following command to link all source files and include necessary headers:

```bash 
clang++ -o PANKH_solver src/*.cpp -Iinclude -std=c++11 
````
</details>

<details>
<summary> GCC compilers </summary>
If you are using g++, compile everything together with:

```bash 
g++ -o PANKH_solver src/*.cpp -Iinclude -std=c++11 
````
</details>

<details>
<summary> Intel compilers </summary>

```bash 
icpx -o PANKH_solver src/*.cpp -Iinclude -std=c++11 
```
</details>

##  Usage

1. **Prepare the Input File:**
   - Modify simulation parameters in the `input.json` file as per your requirements (e.g., freestream conditions, kinematic motion(e.g. pitch,plunge), total simulation time, airfoil geometry, panel discretization, etc.).
   - For parameters that are set to null in `input.json`, their values are automatically computed within the code during runtime. It is recommended to review the relevant section in `main.cpp` that handles JSON parsing for a complete understanding of how default values are derived and assigned.  

2. **Ensure Output Directories Exist:**
   - The solver generates multiple output files (e.g., pressure coefficients, lift and drag coefficients, wake data). These are written to subdirectories under `output_files/`.
   - If these directories do not already exist, refer to [`SETUP.md`](https://github.com/coding4Acause/PANKH/blob/main/SETUP.md) to create them or use the provided setup script.

3. **Run the Solver:**
   After successful compilation, execute the solver from the terminal:
   ```bash
   ./PANKH_solver input.json
   ```
   > Note: The solver expects the `input.json` file as a command-line argument. Ensure this file exists in the same directory or provide the correct path.

---

##  API Documentation

###  Overview

The source code has been extensively documented using **Doxygen-style comments**, enabling automatic generation of API documentation in HTML format. This documentation includes:
- Function descriptions
- Call graphs and dependency trees
- Input/output descriptions
- Hyperlinked navigation for easy access

###  View Documentation Online

The generated API documentation is hosted using **GitHub Pages** and can be accessed at:

 **[https://coding4Acause.github.io/PANKH/](https://coding4Acause.github.io/PANKH/)**

> This will open the `index.html` of the Doxygen-generated documentation directly in your browser.

---

###  Generate Locally (Optional)

If you want to regenerate the documentation on your system:

#### 1. Install Required Tools:
```bash
sudo apt install doxygen graphviz
```

#### 2. Verify Installation:
```bash
doxygen -v   # Check Doxygen version
dot -V       # Check Graphviz version
```

#### 3. Run Doxygen:
From the root of your repository (where the `Doxyfile` is located), run:
```bash
doxygen Doxyfile
```

This will generate a folder (`docs/` or `docs/html/`) containing the full documentation suite.

## License
This project is licensed under the terms of the MIT License. See [License](https://github.com/coding4Acause/2d_UnsteadyVortexPanel/blob/main/LICENSE) for details.

## Future Work

**Incorporating the viscous effects** -  A hybrid approach can be implemented by first solving the inviscid potential flow to obtain velocity and pressure distributions. These will serve as inputs for a 2D boundary layer solver to estimate local wall friction and boundary layer thickness. If displacement thickness effect is sought, the airfoil geometry is iteratively updated by adjusting the body panels based on local boundary layer displacement, and the potential flow solution is recomputed until convergence is achieved.

**Airfoil Geometry** – Currently, the code supports only NACA 4-digit series airfoils. Future improvements could extend its capability to handle other NACA series airfoils, as well as non-NACA airfoils, providing greater flexibility in airfoil selection.

**Parallelization for Improved Performance** – The current implementation runs sequentially, but performance can be significantly enhanced using parallel computing techniques. Since the code utilizes the Eigen library, enabling multi-threading with OpenMP and leveraging Eigen’s built-in vectorization (SIMD) can accelerate matrix operations. 


## Contributers
- [Nipun Arora](https://sites.google.com/view/nipun-arora/home)
- [Ashish Pathak](http://home.iitj.ac.in/~apathak/)
- Rohit Chowdhury 

