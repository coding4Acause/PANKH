#ifndef NEWTON_RAPHSON_NONLINEAR_H
#define NEWTON_RAPHSON_NONLINEAR_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;

VectorXd newton_raphson(double dt, double t, double lwp, double theta_wp, VectorXd &vtotal_wp_cp, VectorXd &x_pp, VectorXd &y_pp, VectorXd &x_cp, VectorXd &y_cp, VectorXd &l, VectorXd &B_unsteady, VectorXd &gamma_unsteady, double gamma_old, VectorXd &gamma_bound, vector<double> &gamma_wake_strength, vector<double> &gamma_wake_x_location, vector<double> &gamma_wake_y_location, VectorXd &wake_panel_cp, VectorXd &wake_panel_normal, MatrixXd &A_unsteady, MatrixXd &unit_normal, MatrixXd &wake_panel_coordinates);

#endif
