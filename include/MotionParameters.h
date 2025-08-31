#ifndef MOTIONPARAMETERS_H
#define MOTIONPARAMETERS_H

#include <Eigen/Dense>

using namespace Eigen;

/**
 * @brief Computes the instantaneous heaving displacement vector.
 *
 * Calculates the heaving displacement of an airfoil at a given time, based on a sinusoidal motion
 * model. The displacement is returned as a 3D vector, with the y-component representing the
 * heaving motion (negative sign indicates downward motion is positive).
 *
 * @param h0 Heaving offset (meters).
 * @param h1 Heaving amplitude (meters).
 * @param phi_h Heaving phase angle (radians).
 * @param t Current time (seconds).
 * @param omega Angular frequency (radians/second).
 * @return VectorXd A 3D vector [0, -h, 0] where h is the heaving displacement.
 */
VectorXd h_instantaneous(double h0, double h1, double phi_h, double t, double omega);

/**
 * @brief Computes the instantaneous heaving velocity vector.
 *
 * Calculates the heaving velocity of an airfoil at a given time, based on the derivative of the
 * sinusoidal heaving motion. The velocity is returned as a 3D vector, with the y-component
 * representing the heaving velocity (negative sign indicates downward velocity is positive).
 *
 * @param h1 Heaving amplitude (meters).
 * @param phi_h Heaving phase angle (radians).
 * @param t Current time (seconds).
 * @param omega Angular frequency (radians/second).
 * @return VectorXd A 3D vector [0, -h_dot, 0] where h_dot is the heaving velocity.
 */
VectorXd h_dot_instantaneous(double h1, double phi_h, double t, double omega);

/**
 * @brief Computes the instantaneous pitch angle.
 *
 * Calculates the pitch angle of an airfoil at a given time, based on a sinusoidal motion model.
 *
 * @param alpha0 Pitch angle offset (radians).
 * @param alpha1 Pitch angle amplitude (radians).
 * @param phi_alpha Pitch phase angle (radians).
 * @param t Current time (seconds).
 * @param omega Angular frequency (radians/second).
 * @return double The instantaneous pitch angle (radians).
 */
double alpha_instantaneous(double alpha0, double alpha1, double phi_alpha, double t, double omega);

/**
 * @brief Computes the instantaneous pitch angular velocity.
 *
 * Calculates the pitch angular velocity of an airfoil at a given time, based on the derivative of
 * the sinusoidal pitch motion.
 *
 * @param alpha1 Pitch angle amplitude (radians).
 * @param phi_alpha Pitch phase angle (radians).
 * @param t Current time (seconds).
 * @param omega Angular frequency (radians/second).
 * @return double The instantaneous pitch angular velocity (radians/second).
 */
double alpha_dot_instantaneous(double alpha1, double phi_alpha, double t, double omega);

#endif // MOTIONPARAMETERS_H