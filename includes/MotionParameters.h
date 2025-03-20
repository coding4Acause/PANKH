#ifndef MOTION_PARAMETERS_H
#define MOTION_PARAMETERS_H

#include <Eigen/Dense>
using Eigen::VectorXd;

/* Declare global motion parameters (defined elsewhere) */
extern double h1, phi_h, alpha0, alpha1, omega, phi_alpha;

/**
 * @brief Computes the instantaneous vertical displacement.
 * 
 * This function calculates the instantaneous heaving motion of 
 * the airfoil at a given time `t` and angular frequency `omega`.
 *
 * @param t Current time instant.
 * @param omega Angular frequency of motion.
 * @return VectorXd The vertical displacement of the airfoil.
 */
VectorXd h_instantaneous(double t, double omega);

/**
 * @brief Computes the instantaneous velocity in the vertical direction.
 * 
 * This function determines the velocity corresponding to 
 * the heaving motion of the airfoil at time `t` and angular frequency `omega`.
 *
 * @param t Current time instant.
 * @param omega Angular frequency of motion.
 * @return VectorXd The vertical velocity.
 */
VectorXd h_dot_instantaneous(double t, double omega);

/**
 * @brief Computes the instantaneous angle of attack.
 *
 * This function calculates the pitch angle of the airfoil 
 * as a function of time `t`.
 *
 * @param t Current time instant.
 * @return double The instantaneous angle of attack in radians.
 */
double alpha_instantaneous(double t);

/**
 * @brief Computes the instantaneous rate of change of angle of attack.
 *
 * This function calculates the angular velocity of the airfoil's 
 * pitching motion at time `t`.
 *
 * @param t Current time instant.
 * @return double The instantaneous pitch rate in radians per second.
 */
double alpha_dot_instantaneous(double t);


/* Function declarations */
// VectorXd h_instantaneous(double t, double omega);
// VectorXd h_dot_instantaneous(double t, double omega);
// double alpha_instantaneous(double t);
// double alpha_dot_instantaneous(double t);

#endif // MOTION_PARAMETERS_H
