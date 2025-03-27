#ifndef NEWTON_RAPHSON_NONLINEAR_H
#define NEWTON_RAPHSON_NONLINEAR_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>

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
 * @param dt Time step size.
 * @param t Current time instant value.
 * @param lwp Wake panel length.
 * @param theta_wp Wake panel angle.
 * @param vtotal_wp_cp Total velocity at the wake panel control point.
 * @param x_pp VectorXd x_pp stores the x-coordinates of panel points on airfoil.
 * @param y_pp VectorXd y_pp stores the y-coordinates of panel points on airfoil.
 * @param x_cp VectorXd x_cp stores the x-coordinates of control points on airfoil.
 * @param y_cp VectorXd y_cp stores the y-coordinates of control points on airfoil.
 * @param l VectorXd l stores the Panel lengths.
 * @param B_unsteady Right-hand side vector for the unsteady system of equations.
 * @param gamma_unsteady Solution vector for circulation values.
 * @param gamma_old Previous time step's total circulation.
 * @param gamma_bound Bound vortex circulations.
 * @param gamma_wake_strength Strength of previously shed wake vortices.
 * @param gamma_wake_x_location x-coordinates of previously shed wake vortices.
 * @param gamma_wake_y_location y-coordinates of previously shed wake vortices.
 * @param wake_panel_cp x- and y-coordinates of wake panel control point.
 * @param wake_panel_normal Normal vector of the wake panel.
 * @param A_unsteady Influence coefficient matrix for unsteady flow.
 * @param unit_normal Normal vectors at control points.
 * @param wake_panel_coordinates Coordinates of wake panel endpoints.
 * 
 * @return VectorXd Residuals for the Newton-Raphson solver.
 */



VectorXd newton_raphson(double dt, double t, double lwp, double theta_wp, VectorXd &vtotal_wp_cp, VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp, VectorXd &l, VectorXd &B_unsteady, VectorXd &gamma_unsteady, double gamma_old, VectorXd &gamma_bound, vector<double> &gamma_wake_strength, vector<double> &gamma_wake_x_location, vector<double> &gamma_wake_y_location, VectorXd &wake_panel_cp, VectorXd &wake_panel_normal, MatrixXd &A_unsteady, MatrixXd &unit_normal, MatrixXd &wake_panel_coordinates);

#endif
