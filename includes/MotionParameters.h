#ifndef MOTION_PARAMETERS_H
#define MOTION_PARAMETERS_H

#include <Eigen/Dense>
using Eigen::VectorXd;

/* Declare global motion parameters (defined elsewhere) */
extern double h1, phi_h, alpha0, alpha1, omega, phi_alpha;

/* Function declarations */
VectorXd h_instantaneous(double t, double omega);
VectorXd h_dot_instantaneous(double t, double omega);
double alpha_instantaneous(double t);
double alpha_dot_instantaneous(double t);

#endif // MOTION_PARAMETERS_H
