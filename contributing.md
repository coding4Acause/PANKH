# Contributing to PANKH

First off, thank you for considering contributing to **PANKH**!   
Your contributions—whether in the form of code, documentation, bug reports, feature requests, or testing—are all highly appreciated.

>  If you're not ready to contribute code but want to support the project:
> -  Star the repository  
> -  Share it with others in your field  
> -  Cite it in your academic work  
> -  Use it in your classroom or lab  

---

## Table of Contents

- [Code of Conduct](#code-of-conduct)  
- [How to Contribute](#how-to-contribute)  
  - [Reporting Bugs](#reporting-bugs)  
  - [Suggesting Features](#suggesting-features)  
  - [Adding New Code or Examples](#adding-new-code-or-examples)  
  - [Improving Documentation](#improving-documentation)  
- [Style Guide](#style-guide)  
- [Commit Messages](#commit-messages)  
- [Setting Up Locally](#setting-up-locally)  
- [Running Tests](#running-tests)  
- [Project Vision and Contribution Opportunities](#project-vision-and-contribution-opportunities)



---

## Code of Conduct

Please read our [Code of Conduct](https://github.com/coding4Acause/PANKH/blob/main/Code_of_Conduct.md) before contributing. By participating in this project, you agree to follow its terms.

---

## How to Contribute

### Reporting Bugs

If you encounter a bug:

- Check if it’s already reported in [Issues](https://github.com/coding4Acause/PANKH/issues)
- If not, [open a new issue](https://github.com/coding4Acause/PANKH/issues/new)
- Include:
  - What you expected to happen
  - What actually happened
  - Steps to reproduce it
  - System info: OS, compiler version, etc.
  - Input files, if possible

### Suggesting Features

If you have an idea for a feature:

- First, check [existing issues](https://github.com/coding4Acause/PANKH/issues)
- If it's not there, open a new issue with a clear title and description
- Explain why it would be useful, and any ideas on implementation


### Adding New Code or Examples

If you're submitting new code (e.g. motion modules, solvers, visualization):

- Fork the repository
- Create a new branch (`feature/your-feature-name`)
- Keep functions modular and avoid hardcoding
- If you're adding examples (e.g. sudden acceleration, pitch–plunge):
  - Place input files in `examples/your_case/`
  - Write a short README for the new example

### Improving Documentation

You can improve:

- The `README.md` (installation, usage, references)
- The `examples/` documentation
- The input JSON format explanation
- Add doc comments in the code

---

## Style Guide

- Use **C++11 or newer**
- No classes/structs (for now) – keep procedural
- Maintain consistent formatting:
  - 4-space indentation
  - `snake_case` for variable names
  - Use `Eigen::VectorXd`, `Eigen::MatrixXd`, etc. when applicable
- Keep the `main.cpp` clean – parse input in a separate file
- Place `.h` files in `includes/`, `.cpp` in `src/`

---

## Commit Messages

Please follow this format:

```
<type>: <short summary>

<body - optional, wrap at 72 chars>
```

Example:

```
fix: correct vortex strength update in unsteady solver

The Kutta condition wasn’t enforced properly for high reduced frequencies.
```

Types: `fix`, `feat`, `docs`, `refactor`, `test`, `chore`

---

## Setting Up Locally

```bash
git clone https://github.com/coding4Acause/PANKH.git
cd PANKH
g++ -o PANKH_solver src/*.cpp -Iincludes -std=c++11
```

Input files are in JSON format and passed to the solver like:

```bash
./PANKH_solver input.json
```

---

## Running Tests

We use basic `test.cpp` files to check solver outputs against reference data.

```bash
g++ -o test_solver test.cpp -std=c++11
./test_solver
```

The test compares `Cl` values against `*_ref.dat` files within a defined tolerance.

> GitHub Actions CI runs these tests automatically on every push.

---

## Project Vision and Contribution Opportunities

PANKH is an open-source C++ framework for unsteady aerodynamic simulations using the unsteady vortex panel method. It aims to be **lightweight**, **modular**, and **research-friendly**—serving as a platform for both educational purposes and high-fidelity aerodynamic investigations.

We welcome contributions from researchers, students, developers, and enthusiasts who are passionate about unsteady flow simulations and numerical methods in fluid dynamics.

###  Contribution Opportunities

Here are several ways you can contribute to the development of PANKH:

- **Fix bugs**: Help identify, report, and resolve bugs to improve the robustness of the codebase.
- **Improve documentation**: Contribute to better usage instructions, inline documentation, or example cases.
- **Suggest or implement enhancements**: Propose new features or optimization techniques to improve performance and usability.
- **Cross-platform support**: Ensure smooth compilation and execution across different operating systems (Linux, Windows, macOS).
- **Add tests**: Contribute unit tests or regression tests to ensure correctness and stability.
- **Performance tuning**: Optimize computational efficiency, solver speed, or memory usage.
- **New modules or algorithms**: Add aerodynamic models, improve numerical stability, or include other simulation features.
-  **Example simulations**: Submit useful cases in the `examples/` directory along with proper documentation and expected results.
- **Geometry module improvements**: Help extend the current airfoil module to support arbitrary airfoil shapes via `.dat` files and automate panel generation.

### Special Areas of Interest

The PANKH core team is actively seeking contributors in the following areas:

- **Development of a GUI or CLI**: We welcome contributors to build a graphical user interface or an interactive command-line tool similar to *XFOIL*, allowing users to control simulation parameters and view flow visualizations more intuitively.
- **Improved airfoil support**: Extend the geometry module to handle *arbitrary airfoil geometries*, including NACA or imported `.dat` formats, and automatically generate surface panels with smooth curvature handling.
- **Fluid–Structure Interaction (FSI)**: We are working on incorporating structural flexibility into the solver for *2-way FSI* simulations with deformable airfoils.
- **3D extension**: The roadmap includes extending PANKH from 2D unsteady panel methods to full *3D vortex-lattice-based simulations* in the near future.

If any of these areas interest you, please reach out or open a discussion—we’d love to collaborate!

You can check the [Issues](https://github.com/your_username/PANKH/issues) page to find tasks labeled `good first issue` or `help wanted`, or suggest new ideas of your own.

Thank you for helping make **PANKH** better!  
— *The Maintainers*

