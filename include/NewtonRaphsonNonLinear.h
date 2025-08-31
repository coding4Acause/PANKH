#ifndef NEWTONRAPHSONNONLINEAR_H
#define NEWTONRAPHSONNONLINEAR_H

#include <Eigen/Dense>
#include <vector>
#include "VectorOperations.h"
#include "InfluenceMatrix.h"
#include "velocity.h"

using namespace Eigen;
using namespace std;

/**
 * @brief Computes the residuals for the Newton-Raphson solver.
 * 
 * @details This function calculates the wake panel's influence, constructs the A_unsteady 
 * matrix, and solves for the unsteady circulation. It also determines the total velocity at 
 * the wake panel control point, including contributions from bound vortices, wake vortices, 
 * and the freestream. The function returns the residuals used for iterative correction.
 *
*
 * @param n Number of nodes on the airfoil.
 * @param dt Time step (seconds).
 * @param t Current time (seconds).
 * @param lwp Length of the wake panel (meters).
 * @param theta_wp Orientation angle of the wake panel (radians).
 * @param freestream Freestream velocity vector [u, v] in the inertial frame (meters/second).
 * @param vtotal_wp_cp Output vector for the total velocity at the wake panel’s control point [u, v] (meters/second).
 * @param x_pp Vector of airfoil panel node x-coordinates (size n, meters).
 * @param y_pp Vector of airfoil panel node y-coordinates (size n, meters).
 * @param x_cp Vector of airfoil control point x-coordinates (size n-1, meters).
 * @param y_cp Vector of airfoil control point y-coordinates (size n-1, meters).
 * @param l Vector of airfoil panel lengths (size n-1, meters).
 * @param B_unsteady Right-hand side vector for the unsteady system (size n+1).
 * @param gamma_unsteady Output vector of vortex strengths for airfoil and wake panels (size n+1).
 * @param gamma_old Circulation from the previous time step (meters²/second).
 * @param gamma_bound Output vector of airfoil vortex strengths (size n).
 * @param gamma_wake_strength Vector of wake vortex strengths (meters²/second).
 * @param gamma_wake_x_location Vector of wake vortex x-coordinates (meters).
 * @param gamma_wake_y_location Vector of wake vortex y-coordinates (meters).
 * @param wake_panel_cp Output vector for the wake panel’s control point [x, y] (meters).
 * @param wake_panel_normal Output vector for the wake panel’s unit normal [nx, ny].
 * @param A_unsteady Output matrix for the unsteady influence coefficients (size (n+1) x (n+1)).
 * @param unit_normal Matrix of airfoil panel unit normals (size (n-1) x 2).
 * @param wake_panel_coordinates Matrix of wake panel node coordinates (size 2 x 2, [x0, y0; x1, y1], meters).
 * @return VectorXd A 2D vector of residuals [length_residual, angle_residual] for the Newton-Raphson solver.
 * @see influence_matrix, velocity_bound_vortices, velocity_induced_due_to_discrete_vortex, dot, magnitude
 */
VectorXd newton_raphson(int n, double dt, double t, double lwp, double theta_wp, VectorXd freestream, VectorXd &vtotal_wp_cp, VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp, VectorXd &l, VectorXd &B_unsteady, VectorXd &gamma_unsteady, double gamma_old, VectorXd &gamma_bound, vector<double> &gamma_wake_strength, vector<double> &gamma_wake_x_location, vector<double> &gamma_wake_y_location, VectorXd &wake_panel_cp, VectorXd &wake_panel_normal, MatrixXd &A_unsteady, MatrixXd &unit_normal, MatrixXd &wake_panel_coordinates);

#endif // NEWTONRAPHSONNONLINEAR_H