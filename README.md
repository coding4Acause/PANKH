# PANKH: A potential flow solver for hovering airfoils

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

- **User-controlled wake modeling** – The `constants.cpp` file allows users to choose between prescribed wake and free wake analysis.

- **Flexible panel discretization** – The code supports both *even* and *odd* numbers of panels with *cosine clustering* for improved resolution near the leading and trailing edges. For details, refer to the `nodal_coordinates_initial` function in `geometry.cpp`.


## Installation

### Prerequisites
Before compiling and running the code, ensure the installation of the following dependencies:

<details>
  <summary> Install a C++ Compiler</summary>
  
  - You need a compiler that supports C++11 or later.
  - Recommended options:
    - GCC (GNU Compiler Collection)
    - Intel C++ Compiler (ICPC/ICX/IPCX)
  - To install GCC on Ubuntu:
    ```bash
    sudo apt install g++
    ```
</details>

<details>
  <summary> Install Eigen Library(Required for matrix operations:)</summary>
  
- Install via package manager:
    ```bash
    sudo apt install libeigen3-dev  # Ubuntu
    ```
  - Or download manually from [Eigen's official website](https://eigen.tuxfamily.org/).
  - Install, compile and run a code which uses Eigen Library ==> [Getting started](https://eigen.tuxfamily.org/dox/GettingStarted.html)
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
git clone https://github.com/coding4Acause/2d_UnsteadyVortexPanel.git 
cd 2d_UnsteadyVortexPanel  
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

2d_UnsteadyVortexPanel   # the name of the local repository
- │── /src          # Contains all .cpp source files
- │── /include      # Contains all .h header files     
- │── README.md        
- │── LICENSE 
- │── /output_files    
</details>

<details>
<summary> GCC compilers </summary>
If you are using g++, compile everything together with:

```bash 
g++ -o 2d_uvpm_solver src/*.cpp -Iinclude -std=c++11 
````
</details>

<details>
<summary> Intel compilers </summary>

```bash 
icpx -o 2d_uvpm_solver src/*.cpp -Iinclude -std=c++11 
```
</details>

## Usage
- Before compiling the code, modify the input parameters in `constants.cpp` as needed.
- Ensure that the required output directories exist before running the code. Refer to `SETUP.md` for details on directory structure.  
- After compiling the code, run the solver using:
```bash
./ 2d_uvpm_solver
```
## API Documentation  

### Generating Source Code Documentation with Doxygen  
The source code is already documented using **Doxygen comments**, making it easy to generate **HTML-based** documentation.

### Prerequisites (Install Doxygen and Graphviz)  
Before running Doxygen, ensure the following dependencies are installed:

- **Doxygen** (Required for generating documentation)  
- **Graphviz** (Required for function call graphs in Doxygen)  

#### Install on Ubuntu  
```bash
sudo apt install doxygen graphviz
```
#### Verify Installation
```
doxygen -v   # Should print the installed Doxygen version
dot -V       # Should print the installed Graphviz version
```
#### Navigate to local repository containing the Doxyfile
```
cd path/to/your/repository
```
#### Run Doxygen
```
doxygen Doxyfile
```

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

