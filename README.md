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

![Vortex shedding animation](JOSS/vortex_shedding.gif)

The essence of this approach lies in transforming a partial differential equation (PDE) into a system of linear algebraic equations, reducing the problem to a linear algebra problem. The real geometry is discretized into a series of flat panels, and a piecewise linearly varying vortex panel method is used to simulate the flow around the airfoil. At each time instant, a constant-strength vortex panel is shed from the trailing edge to satisfy *Kelvin's Circulation Theorem*, ensuring the correct representation of unsteady effects.

This solver is implemented in C++ and utilizes the **Eigen library** for efficient matrix operations. The code computes unsteady aerodynamic forces acting on an airfoil undergoing prescribed motion. A detailed document explaining the numerical framework will be provided separately.
![unsteady_model_at_tk-1](https://github.com/user-attachments/assets/b764fcf8-4402-4e61-bb78-6558aa271894)

## Purpose and Motivation

<details>
<summary><strong> What does PANKH do?</strong></summary>

Studying and analyzing the aerodynamic forces acting on bodies immersed in a fluid is central to a wide range of applications — from the design of fixed-wing aircraft and flexible morphing wings (HAL), to understanding bird and insect flight, developing flapping-wing drones and ornithopters, and optimizing rotor blades in helicopters and wind turbines.  

Before addressing the complete three-dimensional problem, it is often essential to study the two-dimensional airfoil cross-section, which provides critical insights into the underlying flow physics and lift generation mechanisms.

Traditionally, aerodynamic problems are investigated through:
1. Experimental studies – accurate but time-consuming and costly due to setup design and instrumentation.
2. Analytical studies – provide closed-form solutions that offer exact mathematical results and deep physical insight, but are generally limited to highly idealized or simplified versions of real-world problems.
3. Numerical simulations – increasingly popular since the advent of modern computing, offering flexibility and control over complex configurations.

Numerical approaches can be broadly classified into:
- **High-fidelity solvers** that resolve the Navier–Stokes equations with all flow physics, and  
- **Reduced-order models** based on simplifying assumptions that retain key aerodynamic behavior at a fraction of the computational cost.

**PANKH** falls into the second category. It solves the **Laplace equation** under **potential flow assumptions**, making it ideal for low-speed, inviscid, incompressible, and irrotational flows.  

High-fidelity CFD solvers demand substantial computational resources — often running for **days on HPC clusters** across multiple nodes and cores. Moreover, **commercial CFD packages** are expensive, opaque “black-box” systems, making benchmark validations and modifications challenging.  

This motivated the development of **PANKH** — an open-source, reduced-order aerodynamic solver for **unsteady potential flows** that is:
- Fast and lightweight, running within minutes on a single-core desktop,  
- Reasonably accurate, suitable for early-stage aerodynamic analysis,  
- Easy to compile, modify, and extend, supporting researchers and educators alike.  

PANKH aims to help users focus on their **state-of-the-art aerodynamic problems** without depending on GUI-based proprietary tools or reinventing in-house codes.
</details>

---

<details>
<summary><strong> Why PANKH?</strong></summary>

Several open-source tools exist for simulating flow around airfoils — such as **FoilSim III**, **JavaFoil**, and **XFOIL** — but they are **limited to steady-state aerodynamics**.  
In contrast, open-source solvers that handle **unsteady potential flows** are scarce, and those that do exist are often **proprietary, undocumented, or poorly maintained**.

**PANKH bridges this gap.**  
It provides a **modular C++ implementation** capable of simulating:
- **Impulsive starts**,  
- **Pitching and plunging motions**, and  
- **Arbitrary user-defined unsteady kinematics.**

The solver computes **aerodynamic loads**, including **lift** and **inviscid drag** due to pressure forces, and visualizes the **real-time evolution** of both the aerodynamic coefficients and the **vortex wake roll-up**.  

Remarkably, all this computation can be performed **within minutes on a standard single-core processor**.  

While **PANKH does not claim to replace high-fidelity CFD or experiments**, it **complements them effectively** — offering an excellent **trade-off between accuracy, computational cost, and setup complexity**.  

In doing so, PANKH lays the foundation for a new generation of **open-source tools** dedicated to **unsteady aerodynamics** in the potential flow regime.
</details>

---

<details>
<summary><strong> Who is PANKH for?</strong></summary>

**PANKH** is designed with versatility in mind, serving a diverse user base:

- Research and Academia:  
  PANKH can be seamlessly integrated into research workflows for preliminary aerodynamic assessments, parametric studies, or as a component in coupled two-dimensional fluid–structure interaction (FSI) simulations. Its modular design allows researchers to adapt the solver to specific unsteady flow problems, reducing development time while maintaining flexibility for extensions and custom studies.
  PANKH provides a clear, modular, and well-documented framework, making it an ideal tool for classroom demonstrations, student projects, and graduate-level coursework in unsteady aerodynamics and aeroelasticity. Its simplicity and transparency allow students to observe aerodynamic forces, visualize vortex evolution, and explore unsteady phenomena hands-on, bridging the gap between theoretical learning and practical simulation experience.


- Hobbyists and Independent Developers:  
  With its minimal dependencies and easy compilation process, PANKH provides a hands-on platform for aerodynamic exploration, enabling enthusiasts to simulate and visualize unsteady flow phenomena on their personal computers.

With continued development, PANKH aims to evolve into an industry-grade research tool — balancing open accessibility, scientific rigor, and educational utility.
</details>


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
  <summary>Real-time / In-situ Visualization of Aerodynamic Forces and Vortex Shedding</summary>

   Comprehensive platform-specific prerequisites, installation procedures, and verification protocols for enabling real-time visualization are detailed in [visualization_setup.md](https://github.com/coding4Acause/PANKH/blob/main/visualization_setup.md). This includes instructions for Gnuplot terminals, X11 support, and validation of plotting functionality across Linux, macOS, and Windows.
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
<details><summary> Prepare the Input File </summary>

   - Modify simulation parameters in the `input.json` file as per your requirements (e.g., freestream conditions, kinematic motion(e.g. pitch,plunge), total simulation time, airfoil geometry, panel discretization, etc.).
   - For parameters that are set to null in `input.json`, their values are automatically computed within the code during runtime. It is recommended to review the relevant section in `main.cpp` that handles JSON parsing for a complete understanding of how default values are derived and assigned.  
</details>

<details>
  <summary>Ensure Output Directories Exist</summary>
  
  - The solver produces multiple output files, including **pressure coefficients**, **lift and drag coefficients**, and **wake data**. These files are written to the appropriate subdirectories under `output_files/`.

  - If the required directories are absent, consult [`SETUP.md`](https://github.com/coding4Acause/PANKH/blob/main/SETUP.md) for detailed instructions or execute the provided setup script to automatically generate the directory structure.
</details>


 <details><summary>Run the Solver</summary>

   After successful compilation, execute the solver from the terminal:
   ```bash
   ./PANKH_solver input.json
   ```
   > Note: The solver expects the `input.json` file as a command-line argument. Ensure this file exists in the same directory or provide the correct path.
</details>

## Running Tests
To verify the correctness of **PANKH**, you can run the automated test locally. The user does not need to modify anything unless explicitly desired. Simply ensure the prerequisites are satisfied, then compile and execute the test script.

<details><summary>Prerequisites</summary>

- Compile the main solver (PANKH_solver) from the root directory before executing the test   script. The test relies on this executable to run the simulations internally.
- Ensure there is a folder named `tests/` in the project root.
- Inside the `tests/` folder, you will find:

  - **`test.cpp`** – The test script.
  - **`input_test.json`** – The input file designed for the test case.
  - **Reference solution file** – Contains published data used for verification.

</details>

<details><summary> Compilation</summary>

- From the project root directory, compile the test script: 
 ```bash
  g++ -o test_exec tests/test.cpp
  ```
</details>

<details><summary> Run</summary>

 ```bash
  ./test_exec tests/input_test.json
  ```
- The script will automatically run the main solver, compare the computed lift coefficient with the reference solution, and print the results.

</details>

<details><summary> Functionality of test.cpp</summary>

- Internally, `test.cpp` executes the main solver using the provided test input.
- It collects the solver’s output (aerodynamic loads data) from the `output_files/` directory.
- Compares the computed lift coefficient against the reference solution.
- Prints clear messages indicating whether the test passed or failed.
</details>

##  API Documentation

###  Overview

The source code has been extensively documented using **Doxygen-style comments**, enabling automatic generation of API documentation in HTML format. This documentation includes:
- Function descriptions
- Call graphs and dependency trees
- Input/output descriptions
- Hyperlinked navigation for easy access

<details><summary>View Documentation Online</summary>

 The generated API documentation is hosted using **GitHub Pages** and can be accessed at:

 **[https://coding4Acause.github.io/PANKH/](https://coding4Acause.github.io/PANKH/)**

> This will open the `index.html` of the Doxygen-generated documentation directly in your browser.
</details>

<details><summary> Generate Locally (Optional)</summary>

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

</details>

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

