#ifndef VELOCITY_H
#define VELOCITY_H

#include <Eigen/Dense>
#include <cmath>
#include "InfluenceMatrix.h"

using namespace Eigen;
using namespace std;

// Function declarations
VectorXd velocity_bound_vortices(VectorXd &x_pp, VectorXd &y_pp, double x, double y, VectorXd G_bound);
VectorXd velocity_induced_due_to_discrete_vortex(double gamma, double vor_point_x, double vor_point_y, double des_point_x, double des_point_y);

#endif  // VELOCITY_H
