# Kinematic Parameters for Pitch-Plunge Motion

The validation results in this directory correspond to the parameters used when running the source code.  
This is an example case—users can modify parameters in the following files to run different cases:  

- `constants.cpp`
- `kinematics.cpp`  

> **Note:** Editing this `kinematics.md` file will not affect the source code. It serves only as documentation for the validation cases.  

---

## Flow Properties

- **Density of water** ($\rho$) = 998.2 kg/m³  
- **Viscosity of water** ($\mu$) = 1.0016 × 10⁻³ Pa·s  
- **Reynolds number** (Re) = 40,000  
- **Freestream velocity** ($Q_\infty$) = $\frac{\text{Re} \cdot \mu}{\rho \cdot c}$  
- **Dynamic pressure** ($q_\infty$) = $\frac{1}{2} \rho Q_\infty^2$  

 *The rationale for these values can be found in Chapter 4 of* **Unsteady Aerodynamics by Dimitriadis**.  

---

## Airfoil Geometry

- **Chord length** (c) = 0.1 m  
- **Half-chord length** (b) = c / 2  
- **Pitch axis location** ($x_f$) = c / 3  
- **Non-dimensional pitch axis position** (a) = $\frac{x_f - b}{b}$  
- **Airfoil type** = NACA 0012  
- **Number of panels** = 100  

---

## Motion Parameters

- **Reduced frequency** (k) = 1.5  
- **Oscillation frequency** ($\omega$) = $\frac{2 k Q_\infty}{c}$ [rad/sec]  
- **Time period of oscillation** (T) = $\frac{2\pi}{\omega}$ [sec]  
- **Plunge amplitude** ($h_1$) = 0.25c  
- **Pitch amplitude** ($\alpha_1$) = $15^\circ - \tan^{-1} \left(\frac{2 k h_1}{c} \right)$  
- **Mean plunge** ($h_0$) = 0.0 m  
- **Mean pitch** ($\alpha_0$) = 0.0°  
- **Plunge phase** ($\phi_h$) = 0.0°  
- **Pitch phase** ($\phi_\alpha$) = $90^\circ + \phi_h$  

---

## Instantaneous Kinematics

For time **t**, the instantaneous plunge and pitch angles are given by:  

- **Plunge displacement:**  
  $$ h(t) = h_0 + h_1 \sin(\omega t + \phi_h) $$  

- **Pitch angle:**  
  $$ \alpha(t) = \alpha_0 + \alpha_1 \sin(\omega t + \phi_\alpha) $$  

---

