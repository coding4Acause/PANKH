# PURPOSE
This directory contains test cases where the source code is run for different unsteady aerodynamic scenarios. The computed aerodynamic loads (**lift coefficient** and **drag coefficient**) are validated against the results from **Grigorios Dimitriadis' work**.  

Each test case includes a [`kinematics.md`](https://github.com/coding4Acause/PANKH/blob/main/examples/pitch_plunge/kinematics.md) file, which describes the kinematic equations and parameters used to generate the results. Users can refer to this file to understand the motion characteristics associated with each results.  

To run any test case, users are encouraged to read the **documentation section of the main [`README.md`](https://github.com/coding4Acause/2d_UnsteadyVortexPanel/blob/main/README.md)**. Parameters can be modified in:  
- `constants.cpp` → For global simulation parameters.  
- `kinematics.cpp` → For modifying kinematic equations. 

# IMPORTANT NOTE

**The `kinematics.md` file does not interact with the source code!**  
- It is purely for **informational purposes**.  
- Changing values in `kinematics.md` **will NOT** affect the simulation results.  
- This file simply documents the kinematic parameters and equations that were used to generate the results presented in this directory.  

---

For more details, refer to the main [`README.md`](https://github.com/coding4Acause/2d_UnsteadyVortexPanel/blob/main/README.md).